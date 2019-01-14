#include <EBV_Stereo_Calibration.h>

IntrinsicsData::IntrinsicsData()
{
    m_K.resize(3*3);
    m_D.resize(1*5);
}

StereoCalibrator::StereoCalibrator(LaserController* laser,
                                   Filter* filter0,
                                   Filter* filter1,
                                   Triangulator* triangulator)
    : m_laser(laser),
      m_filter{filter0,filter1},
      m_triangulator(triangulator),
      m_last_frame_captured{std::chrono::high_resolution_clock::time_point{},
                            std::chrono::high_resolution_clock::time_point{}},
      m_camera_intrinsics{IntrinsicsData{},IntrinsicsData{}}
{
    for (auto& m: m_R) { m.resize(3*3); }
    for (auto& m: m_T) { m.resize(3*1); }
    for (auto& m: m_E) { m.resize(3*3); }
    for (auto& m: m_F) { m.resize(3*3); }
}

StereoCalibrator::~StereoCalibrator(){}

// DEPRECATED -> Use calibrateCamLaser instead
void StereoCalibrator::calibrateCameras(cv::Mat &frame, const int id)
{
    // Assess enough time between two frame captures
    const auto now = std::chrono::high_resolution_clock::now();
    if (now-m_last_frame_captured[id]<m_min_capture_delay) { return; }
    m_last_frame_captured[id] = now;

    // Extract checkerboard corner points
    if (m_calib_image_points[id].size()<m_min_frames_to_capture)
    {
        std::vector<cv::Point2f> points;
        bool ret = cv::findChessboardCorners(frame, m_pattern_size, points);
        if (ret)
        {
            cv::cornerSubPix(frame,points,cv::Size(5,5),cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::COUNT
                             + cv::TermCriteria::EPS,30, DBL_EPSILON));
            m_calib_image_points[id].push_back(points);
            cv::drawChessboardCorners(frame,m_pattern_size,points,ret);
            printf("Calibrating camera %d: %d frames retrieved. \n\r",
                   id, m_calib_image_points[id].size());
        }
    }

    // Extract intrinsics once enough points    
    if (   m_calib_image_points[0].size()>=m_min_frames_to_capture
        && m_calib_image_points[1].size()>=m_min_frames_to_capture)
    {
        const std::vector<std::vector<cv::Point3f>> worldPoints0
        {
            m_calib_image_points[0].size(),
            calculateWorldPoints()
        };
        cv::Mat rvecs0, tvecs0;
        cv::calibrateCamera(worldPoints0,m_calib_image_points[0],
                            m_resolution,
                            m_camera_intrinsics[0].m_K,
                            m_camera_intrinsics[0].m_D,
                            rvecs0, tvecs0,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        const std::vector<std::vector<cv::Point3f>> worldPoints1
        {
            m_calib_image_points[1].size(),
            calculateWorldPoints()
        };
        cv::Mat rvecs1, tvecs1;
        cv::calibrateCamera(worldPoints1,m_calib_image_points[1],
                            m_resolution,
                            m_camera_intrinsics[1].m_K,
                            m_camera_intrinsics[1].m_D,
                            rvecs1, tvecs1,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        // Extract extrinsics
        float rms = cv::stereoCalibrate(
            worldPoints1,
            m_calib_image_points[0],
            m_calib_image_points[1],
            m_camera_intrinsics[0].m_K,
            m_camera_intrinsics[0].m_D,
            m_camera_intrinsics[1].m_K,
            m_camera_intrinsics[1].m_D,
            m_resolution, m_R[0], m_T[0], m_E[0], m_F[0],
            cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::COUNT
                             + cv::TermCriteria::EPS, 60,
                             DBL_EPSILON));

        // Save parameters
        saveCamerasCalibration();
        m_triangulator->importCalibration();
        printf("Finished extracting the extrinsics with a rms per frame of %f \n\r",
               rms/m_min_frames_to_capture);
        return;
    }
}

void StereoCalibrator::calibrateCamLaser(cv::Mat& frame0,
                                         cv::Mat& frame1,
                                         const int id)
{   
    // Reset laser position (pointing forward)
    m_laser->setPos(m_laser->m_max_x/2,m_laser->m_max_y/2);

    // Extract checkerboard corner points and corresponding laser commands
    if (   m_calib_image_points[0].size()<m_min_frames_to_capture
        && m_calib_image_points[1].size()<m_min_frames_to_capture)
    {
        std::vector<cv::Point2f> points0;
        std::vector<cv::Point2f> points1;
        cv::equalizeHist(frame0,frame0);
        bool ret0 = cv::findChessboardCorners(frame0,
                                             m_pattern_size,
                                             points0);
        cv::equalizeHist(frame1,frame1);
        bool ret1 = cv::findChessboardCorners(frame1,
                                             m_pattern_size,
                                             points1);

        if (ret0 && ret1)
        {
            // Corner detection - Camera 0
            cv::cornerSubPix(frame0,points0,cv::Size(5,5),cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::COUNT
                             + cv::TermCriteria::EPS,30, DBL_EPSILON));
            m_calib_image_points[0].push_back(points0);
            cv::drawChessboardCorners(frame0,m_pattern_size,points0,ret0);

            // Corner detection - Camera 1
            cv::cornerSubPix(frame1,points1,cv::Size(5,5),cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::COUNT
                             + cv::TermCriteria::EPS,30, DBL_EPSILON));
            m_calib_image_points[1].push_back(points1);
            cv::drawChessboardCorners(frame1,m_pattern_size,points1,ret1);

            printf("Calibrating laser-camera: %d frames retrieved. \n\r",
                   m_calib_image_points[0].size());

            // Finding laser command correspondance to each corner point of frame 0
            std::vector<cv::Point2f> laserPoints;
            for (auto &point : points0)
            {
                pointLaserToPixel(int(point.y),int(point.x),0);
                int xLaser = 180*(m_laser->getX()-m_laser->getMinX())/(m_laser->getMaxX() - m_laser->getMinX());
                int yLaser = 240*(m_laser->getY()-m_laser->getMinY())/(m_laser->getMaxY() - m_laser->getMinY());
                laserPoints.emplace_back(yLaser, xLaser);
                printf("Pointing to (%d,%d) with commands (%d,%d)->(%d,%d). \n\r",
                       int(point.x),int(point.y),m_laser->getX(),m_laser->getY(),
                       xLaser,yLaser);
            }
            m_calib_laser_points.push_back(laserPoints);
        }
        {
            printf("Checkboard not detected. Please move it in the field of view of both cameras./n/r");
        }
    }

    // Extract intrinsics & extrinsics once enough data
    if (   m_calib_image_points[0].size()>=m_min_frames_to_capture
        && m_calib_image_points[1].size()>=m_min_frames_to_capture
        && m_calib_laser_points.size()>=m_min_frames_to_capture)
    {
        m_camera_intrinsics[0].m_K = m_triangulator->m_K[0];
        m_camera_intrinsics[0].m_D = m_triangulator->m_D[0];

        m_camera_intrinsics[1].m_K = m_triangulator->m_K[1];
        m_camera_intrinsics[1].m_D = m_triangulator->m_D[1];

        // Extract camera intrinsics
        const std::vector<std::vector<cv::Point3f>> worldPoints
        {
            m_calib_image_points[0].size(),
            calculateWorldPoints()
        };
        cv::Mat rvecs0, tvecs0;
        cv::calibrateCamera(worldPoints,m_calib_image_points[0],
                            m_resolution,
                            m_camera_intrinsics[0].m_K,
                            m_camera_intrinsics[0].m_D,
                            rvecs0, tvecs0,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        cv::Mat rvecs1, tvecs1;
        cv::calibrateCamera(worldPoints,m_calib_image_points[1],
                            m_resolution,
                            m_camera_intrinsics[1].m_K,
                            m_camera_intrinsics[1].m_D,
                            rvecs1, tvecs1,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        cv::Mat rvecs2, tvecs2;
        cv::calibrateCamera(worldPoints,m_calib_laser_points,
                            m_resolution,
                            m_camera_intrinsics[2].m_K,
                            m_camera_intrinsics[2].m_D,
                            rvecs2, tvecs2,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        // Extract extrinsics (camera0-camera1)
        double rms0 = cv::stereoCalibrate(
            worldPoints,
            m_calib_image_points[0],
            m_calib_image_points[1],
            m_camera_intrinsics[0].m_K,
            m_camera_intrinsics[0].m_D,
            m_camera_intrinsics[1].m_K,
            m_camera_intrinsics[1].m_D,
            m_resolution, m_R[0], m_T[0], m_E[0], m_F[0],
            cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::COUNT
                             + cv::TermCriteria::EPS, 60,
                             DBL_EPSILON));

        // Extract extrinsics (Camera0-Laser)
        double rms1 = cv::stereoCalibrate(
            worldPoints,
            m_calib_image_points[0],
            m_calib_laser_points,
            m_camera_intrinsics[0].m_K,
            m_camera_intrinsics[0].m_D,
            m_camera_intrinsics[2].m_K,
            m_camera_intrinsics[2].m_D,
            m_resolution, m_R[1], m_T[1], m_E[1], m_F[1],
            cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::COUNT
                             + cv::TermCriteria::EPS, 60,
                             DBL_EPSILON));

        // Extract extrinsics (Laser-Camera1)
        double rms2 = cv::stereoCalibrate(
            worldPoints,
            m_calib_image_points[1],
            m_calib_laser_points,
            m_camera_intrinsics[1].m_K,
            m_camera_intrinsics[1].m_D,
            m_camera_intrinsics[2].m_K,
            m_camera_intrinsics[2].m_D,
            m_resolution, m_R[2], m_T[2], m_E[2], m_F[2],
            cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::COUNT
                             + cv::TermCriteria::EPS, 60,
                             DBL_EPSILON));

        printf("Finished extracting the extrinsics with rms %.3f - %.3f - %.3f. \n\r",
               rms0,rms1,rms2);

        // Save all
        saveCamerasCalibration();
        saveLaserCalibration();
        m_triangulator->importCalibration();
    }
    return;
}

void StereoCalibrator::saveCamerasCalibration()
{
    cv::FileStorage fs(m_triangulator->m_path_calib_cam, cv::FileStorage::WRITE);
    fs << "camera_matrix0" << m_camera_intrinsics[0].m_K;
    fs << "dist_coeffs0" << m_camera_intrinsics[0].m_D;
    fs << "camera_matrix1" << m_camera_intrinsics[1].m_K;
    fs << "dist_coeffs1" << m_camera_intrinsics[1].m_D;
    fs << "R" << m_R[0]; // Cam2 -> Cam1
    fs << "T" << m_T[0];
    fs << "F" << m_F[0];
    fs.release();
    printf("Camera calibration saved ! \n\r");

    m_calib_image_points[0].clear();
    m_calib_image_points[1].clear();

    return;
}

void StereoCalibrator::saveLaserCalibration()
{
    cv::FileStorage fs(m_triangulator->m_path_calib_laser, cv::FileStorage::WRITE);
    fs << "camera_matrix_laser" << m_camera_intrinsics[2].m_K;
    fs << "dist_coeffs_laser" << m_camera_intrinsics[2].m_D;
    fs << "R1" << m_R[1];
    fs << "T1" << m_T[1];
    fs << "F1" << m_F[1];
    fs << "R2" << m_R[2];
    fs << "T2" << m_T[2];
    fs << "F2" << m_F[2];
    fs.release();
    printf("Laser calibration saved ! \n\r");

    m_calib_image_points[0].clear();
    m_calib_image_points[1].clear();
    m_calib_laser_points.clear();

    return;
}

const std::vector<cv::Point3f> StereoCalibrator::calculateWorldPoints()
{
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<uint64_t>(m_pattern_size.x * m_pattern_size.y));

    for (int i = 0; i < m_pattern_size.y; i++)  // row index
    {
        for (int j = 0; j < m_pattern_size.x; j++) // column index
        {
            points.emplace_back(cv::Point3f(j * m_pattern_square_size.x,
                                            i * m_pattern_square_size.y, 0));
        }
    }
    return points;
}

void StereoCalibrator::pointLaserToPixel(const int rGoal, const int cGoal, const int id)
{
    int r = m_filter[id]->getX();
    int c = m_filter[id]->getY();

    // Compute error
    std::array<int,2> diff = {rGoal-r,cGoal-c};
    int mse = std::sqrt(diff[0]*diff[0] + diff[1]*diff[1]);

    for (int iter=0; iter<1000; iter++)
    {
        // Detect current laser position in camera
        r = m_filter[id]->getX();
        c = m_filter[id]->getY();

        // Compute error
        diff = {rGoal-r,cGoal-c};
        mse = std::sqrt(diff[0]*diff[0] + diff[1]*diff[1]);
        if (mse<m_threshConverged) { return; }

        // Update laser command
        int dx;
        if (m_learningRate*diff[0]>m_step) { dx = m_learningRate*diff[0]; }
        else { dx = (diff[0]>0?m_step:-m_step); }

        int dy;
        if (m_learningRate*diff[1]>m_step)
        {
            dy = m_learningRate*diff[1];
        }
        else { dy = (diff[1]>0?m_step:-m_step); }

        int x = m_laser->getX() + dx;
        int y = m_laser->getY() + dy;
        m_laser->setPos(x,y);
    }

    //=== DEBUG ===//
//    printf("Current laser position (%d,%d). \n\r",r,c);
//    printf("Diff: (%d,%d) - MSE: %d. \n\r", diff[0],diff[1],mse);
//    printf("Laser commands: (%d,%d). \n\r",m_laser->getX(),m_laser->getY());
    //=== END DEBUG ===//
    return;
}


DLT::DLT()
{
    m_U.resize(3*3);
    m_T.resize(4*4);
}

DLT::~DLT() {}

void DLT::normalizePoints(cv::Mat& points2D,
                          cv::Mat& points3D)
{
    // Compute mean and std of points array
    cv::Scalar mean2D, mean3D, sigma2D, sigma3D;
    cv::meanStdDev(points2D,mean2D,sigma2D);
    cv::meanStdDev(points3D,mean3D,sigma3D);

    // Compute normalization matrices
    cv::Scalar s_2D = sigma2D/std::sqrt(2);
    cv::Mat U = (cv::Mat_<double>(3,3,CV_64F) << s_2D[0],       0, mean2D[0],
                                                       0, s_2D[1], mean2D[1],
                                                       0,       0,         1);
    cv::invert(U,m_U);

    cv::Scalar s_3D = sigma3D/std::sqrt(3);
    cv::Mat T = (cv::Mat_<double>(4,4,CV_64F) << s_3D[0],       0,       0, mean3D[0],
                                                       0, s_3D[1],       0, mean3D[1],
                                                       0,       0, s_3D[2], mean3D[2],
                                                       0,       0,       0,         1);
    cv::invert(T,m_T);

    std::cout << "m_T: " << m_T.size << std::endl;
    std::cout << "points: " << points2D.size << std::endl;

    // Normalize points
    points2D = m_U * points2D;
    points3D = m_T * points3D;
}


// Code from https://github.com/lagadic/camera_localization/blob/master/opencv/pose-basis/pose-dlt-opencv.cpp
void DLT::extractProjectionMatrix(//const cv::Mat wX, const cv::Mat x,
        const std::vector<cv::Point3d>& wX,
                                  const std::vector<cv::Point2d>& x,
                                  cv::Mat& t, cv::Mat& R, cv::Mat& K)
{
    int nbpoints = (int)wX.size();
    cv::Mat A(2*nbpoints, 12, CV_64F, cv::Scalar(0));
    for (int i=0; i<nbpoints; i++)
    {
        A.at<double>(2*i, 0) = wX[i].x;
        A.at<double>(2*i, 1) = wX[i].y;
        A.at<double>(2*i, 2) = wX[i].z;
        A.at<double>(2*i, 3) = 1 ;

        A.at<double>(2*i+1, 4) = wX[i].x;
        A.at<double>(2*i+1, 5) = wX[i].y;
        A.at<double>(2*i+1, 6) = wX[i].z;
        A.at<double>(2*i+1, 7) = 1 ;

        A.at<double>(2*i, 8)  = -x[i].x * wX[i].x;
        A.at<double>(2*i, 9)  = -x[i].x * wX[i].y;
        A.at<double>(2*i, 10) = -x[i].x * wX[i].z;
        A.at<double>(2*i, 11) = -x[i].x;

        A.at<double>(2*i+1, 8)  = -x[i].y * wX[i].x;
        A.at<double>(2*i+1, 9)  = -x[i].y * wX[i].y;
        A.at<double>(2*i+1, 10) = -x[i].y * wX[i].z;
        A.at<double>(2*i+1, 11) = -x[i].y;
    }

    // Apply SVD on A
    cv::Mat S, U, Vt;
    cv::SVD::compute(A, S, U, Vt);

    std::cout << "Helloooo" << std::endl;
    // Find smallest eigenvalue in S
    double smallestSv = S.at<double>(0, 0);
    unsigned int indexSmallestSv = 0 ;
    for (int i = 1; i < S.rows; i++)
    {
        if ((S.at<double>(i, 0) < smallestSv) )
        {
            smallestSv = S.at<double>(i, 0);
            indexSmallestSv = i;
        }
    }

    // Right null vector extraction
    cv::Mat rnv = Vt.row(indexSmallestSv);
    if (rnv.at<double>(0, 11) < 0) // tz < 0
        rnv *=-1;

    // Reshape projection matrix
    cv::Mat P = cv::Mat_<double>(3,4,CV_64F);
    P = rnv.reshape(1,3);

    // Denormalize
    //cv::Mat Tinv;
    //cv::invert(m_T,Tinv);
    //P = Tinv*P*m_U;

    // Extract rotation and intrinsics matrix
    cv::decomposeProjectionMatrix(P,K,R,t);

    // Scale intrinsic matrix
    K = K/K.at<double>(2,2);

    // Extract translation vector (right-null vector of P)
    cv::SVD::compute(P, S, U, Vt);
    smallestSv = S.at<double>(0, 0);
    indexSmallestSv = 0 ;
    for (int i = 1; i < S.rows; i++)
    {
        if ((S.at<double>(i, 0) < smallestSv) )
        {
            smallestSv = S.at<double>(i, 0);
            indexSmallestSv = i;
        }
    }
    t = Vt.row(indexSmallestSv);
    t = t/t.at<double>(0,3);
}

void DLT::test()
{
    //std::vector<cv::Point3d> wX;
    //std::vector<cv::Point2d>  x;
    std::vector<cv::Vec4d> wX;
    std::vector<cv::Vec3d> x;

    // Ground truth pose used to generate the data
    cv::Mat ctw_truth = (cv::Mat_<double>(3,1) << -0.1, 0.1, 1.2); // Translation vector
    cv::Mat crw_truth = (cv::Mat_<double>(3,1) << CV_PI/180*(5), CV_PI/180*(0), CV_PI/180*(45)); // Rotation vector
    cv::Mat cRw_truth(3,3,cv::DataType<double>::type); // Rotation matrix
    cv::Rodrigues(crw_truth, cRw_truth);

    // Intrinsics
    cv::Mat K = ( cv::Mat_<double>(3,3) << 2.23616547e+02, 0., 1.12623787e+02, 0., 2.23256653e+02, 6.52757416e+01, 0., 0., 1.);

    // Input data: 3D coordinates of at least 6 non coplanar points
    double L = 0.2;
    wX.push_back(cv::Vec4d(-L, -L, 0, 1)); // wX_0 ( -L, -L, 0  )^T
    wX.push_back(cv::Vec4d(2*L, -L, 0.2, 1)); // wX_1 (-2L, -L, 0.2)^T
    wX.push_back(cv::Vec4d(L, L, 0.2, 1)); // wX_2 (  L,  L, 0.2)^T
    wX.push_back(cv::Vec4d(-L, L, 0, 1)); // wX_3 ( -L,  L, 0  )^T
    wX.push_back(cv::Vec4d(-2*L, L, 0, 1)); // wX_4 (-2L,  L, 0  )^T
    wX.push_back(cv::Vec4d(0, 0, 0.5, 1)); // wX_5 (  0,  0, 0.5)^T

//    wX.push_back( cv::Point3d(-L, -L, 0) ); // wX_0 ( -L, -L, 0  )^T
//    wX.push_back( cv::Point3d( 2*L, -L, 0.2) ); // wX_1 (-2L, -L, 0.2)^T
//    wX.push_back( cv::Point3d(   L,  L, 0.2) ); // wX_2 (  L,  L, 0.2)^T
//    wX.push_back( cv::Point3d(  -L,  L, 0  ) ); // wX_3 ( -L,  L, 0  )^T
//    wX.push_back( cv::Point3d(-2*L,  L, 0  ) ); // wX_4 (-2L,  L, 0  )^T
//    wX.push_back( cv::Point3d(   0,  0, 0.5) ); // wX_5 (  0,  0, 0.5)^T



    // Input data: 2D coordinates of the points on the image plane
    for(int i = 0; i < wX.size(); i++)
    {
        cv::Mat cX = K*(cRw_truth*cv::Mat(wX[i]) + ctw_truth); // Update cX, cY, cZ
        std::cout << "Helloooo" << std::endl;
        //x.push_back( cv::Point2d( cX.at<double>(0, 0)/cX.at<double>(2, 0),
        //                          cX.at<double>(1, 0)/cX.at<double>(2, 0) ) ); // x = (cX/cZ, cY/cZ)
        x.push_back(cv::Vec3d(cX.at<double>(0, 0)/cX.at<double>(2, 0),
                              cX.at<double>(1, 0)/cX.at<double>(2, 0),1)); // x = (cX/cZ, cY/cZ)
    }

    // Homogeneous coordinates
    //cv::convertPointsToHomogeneous(x,x);
    //cv::convertPointsToHomogeneous(wX,wX);

    std::cout << "Helloooo" << std::endl;

    // Normalize points
    cv::Mat points2D = cv::Mat(x).reshape(1,3);
    cv::Mat points3D = cv::Mat(wX).reshape(1,4);
    normalizePoints(points2D,points3D);

    std::cout << "Helloooo" << std::endl;

    // Apply DLT
    cv::Mat t(3, 1, CV_64F); // Translation vector
    cv::Mat R(3, 3, CV_64F); // Rotation matrix
    cv::Mat K0(3, 3, CV_64F); // Intrinsics matrix
    DLT dlt;
    dlt.extractProjectionMatrix(points3D, points2D, t, R, K0);

    std::cout << "K (ground truth):\n" << K << std::endl;
    std::cout << "K (computed with DLT):\n" << K0 << std::endl;
    std::cout << "t (ground truth):\n" << ctw_truth << std::endl;
    std::cout << "t (computed with DLT):\n" << t << std::endl;
    std::cout << "R (ground truth):\n" << cRw_truth << std::endl;
    std::cout << "R (computed with DLT):\n" << R << std::endl;
}


void DLT::extractCalibration()
{
    std::vector<cv::Point3d>  wX;
    std::vector<cv::Point2d>  x_cam0;
    std::vector<cv::Point2d>  x_cam1;
    std::vector<cv::Point2d>  x_laser;

    std::ifstream data;
    data.open ("../experiments/pointwise_biplanar_1.txt");

    double x0,y0,x1,y1,x2,y2,X,Y,Z;
    while (data >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> X >> Y >> Z)
    {
        wX.push_back(cv::Point3d(X,Y,Z));
        x_cam0.push_back(cv::Point2d(y0,x0));
        x_cam1.push_back(cv::Point2d(y1,x1));
        x_laser.push_back(cv::Point2d(y2,x2));

        //std::cout << x0 << ' ' << y0 << ' ' << x1 << ' ' <<
        //             y1 << ' ' << x2 << ' ' << y2 << ' ' <<
        //             X << ' ' << Y << ' ' << Z << std::endl;
    }

    cv::Mat t0(3, 1, CV_64F); // Translation vector
    cv::Mat R0(3, 3, CV_64F); // Rotation matrix
    cv::Mat K0(3, 3, CV_64F); // Intrinsics matrix
    cv::Mat t1(3, 1, CV_64F); // Translation vector
    cv::Mat R1(3, 3, CV_64F); // Rotation matrix
    cv::Mat K1(3, 3, CV_64F); // Intrinsics matrix
    cv::Mat t2(3, 1, CV_64F); // Translation vector
    cv::Mat R2(3, 3, CV_64F); // Rotation matrix
    cv::Mat K2(3, 3, CV_64F); // Intrinsics matrix
    DLT dlt;
    dlt.extractProjectionMatrix(wX, x_cam0, t0, R0, K0);
    dlt.extractProjectionMatrix(wX, x_cam1, t1, R1, K1);
    dlt.extractProjectionMatrix(wX, x_laser,t2, R2, K2);

    cv::Mat R0t;
    cv::transpose(R0,R0t);
    cv::Mat R1_bis = R1*R0t;
    cv::Mat t1_bis = t1 - t0;

    //std::cout << "K (computed with DLT):\n" << K1 << std::endl;
    std::cout << "t (computed with DLT):\n" << t0 << std::endl;
    std::cout << "t (computed with DLT):\n" << t1 << std::endl;
    std::cout << "t (computed with DLT):\n" << t2 << std::endl;
    //std::cout << "R (computed with DLT):\n" << R1 << std::endl;
}
