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
      m_path_calib_cam(m_triangulator->m_path_calib_cam),
      m_path_calib_laser(m_triangulator->m_path_calib_laser),
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

void StereoCalibrator::calibrateCameras(cv::Mat &frame, const int id)
{
    // Assess enough time between two frame captures
    const auto now = std::chrono::high_resolution_clock::now();
    if (now-m_last_frame_captured[id]<m_min_capture_delay) { return; }
    m_last_frame_captured[id] = now;

    // Extract checkerboard corner points
    if (m_intrinsic_calib_image_points[id].size()<m_min_frames_to_capture)
    {
        std::vector<cv::Point2f> points;
        bool ret = cv::findChessboardCorners(frame, m_pattern_size, points);
        if (ret)
        {
            cv::cornerSubPix(frame,points,cv::Size(5,5),cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::COUNT
                             + cv::TermCriteria::EPS,30, DBL_EPSILON));
            m_intrinsic_calib_image_points[id].push_back(points);
            cv::drawChessboardCorners(frame,m_pattern_size,points,ret);
            printf("Calibrating camera %d: %d frames retrieved. \n\r",
                   id, m_intrinsic_calib_image_points[id].size());
        }
    }

    // Extract intrinsics once enough points    
    if (   m_intrinsic_calib_image_points[0].size()>=m_min_frames_to_capture
        && m_intrinsic_calib_image_points[1].size()>=m_min_frames_to_capture)
    {
        const std::vector<std::vector<cv::Point3f>> worldPoints0
        {
            m_intrinsic_calib_image_points[0].size(),
            calculateWorldPoints()
        };
        cv::Mat rvecs0, tvecs0;
        cv::calibrateCamera(worldPoints0,m_intrinsic_calib_image_points[0],
                            m_resolution,
                            m_camera_intrinsics[0].m_K,
                            m_camera_intrinsics[0].m_D,
                            rvecs0, tvecs0,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        const std::vector<std::vector<cv::Point3f>> worldPoints1
        {
            m_intrinsic_calib_image_points[1].size(),
            calculateWorldPoints()
        };
        cv::Mat rvecs1, tvecs1;
        cv::calibrateCamera(worldPoints1,m_intrinsic_calib_image_points[1],
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
            m_intrinsic_calib_image_points[0],
            m_intrinsic_calib_image_points[1],
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
        m_calibrate_cameras = false;
        printf("Finished extracting the extrinsics with a rms per frame of %f \n\r",
               rms/m_min_frames_to_capture);
        return;
    }
}

void StereoCalibrator::calibrateLaser(cv::Mat& frame0,
                                      cv::Mat& frame1,
                                      const int id)
{   
    // Reset laser pos
    m_laser->setPos(m_laser->m_max_x/2,m_laser->m_max_y/2);

    // Extract checkerboard corner points and corresponding laser commands
    if (   m_intrinsic_calib_image_points[0].size()<m_min_frames_to_capture
        && m_intrinsic_calib_image_points[1].size()<m_min_frames_to_capture)
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
            m_intrinsic_calib_image_points[0].push_back(points0);
            cv::drawChessboardCorners(frame0,m_pattern_size,points0,ret0);

            // Corner detection - Camera 1
            cv::cornerSubPix(frame1,points1,cv::Size(5,5),cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::COUNT
                             + cv::TermCriteria::EPS,30, DBL_EPSILON));
            m_intrinsic_calib_image_points[1].push_back(points1);
            cv::drawChessboardCorners(frame1,m_pattern_size,points1,ret1);

            printf("Calibrating laser-camera: %d frames retrieved. \n\r",
                   m_intrinsic_calib_image_points[0].size());

            // Finding laser command correspondance to each corner point of frame 0
            std::vector<cv::Point2f> laserPoints;
            for (auto &point : points0)
            {
                pointLaserToPixel(int(point.y),int(point.x),0);
                int xLaser = 180*(m_laser->getX()-m_laser->m_min_x)/(m_laser->m_max_x - m_laser->m_min_x);
                int yLaser = 240*(m_laser->getY()-m_laser->m_min_y)/(m_laser->m_max_y - m_laser->m_min_y);
                laserPoints.emplace_back(yLaser, xLaser);
                printf("Pointing to (%d,%d) with commands (%d,%d)->(%d,%d). \n\r",
                       int(point.x),int(point.y),m_laser->getX(),m_laser->getY(),
                       xLaser,yLaser);
            }
            m_intrinsic_calib_laser_points.push_back(laserPoints);
        }
    }

    // Extract intrinsics & extrinsics once enough data
    if (   m_intrinsic_calib_image_points[0].size()>=m_min_frames_to_capture
        && m_intrinsic_calib_image_points[1].size()>=m_min_frames_to_capture
        && m_intrinsic_calib_laser_points.size()>=m_min_frames_to_capture)
    {
        m_camera_intrinsics[0].m_K = m_triangulator->m_K[0];
        m_camera_intrinsics[0].m_D = m_triangulator->m_D[0];

        m_camera_intrinsics[1].m_K = m_triangulator->m_K[1];
        m_camera_intrinsics[1].m_D = m_triangulator->m_D[1];

        // Extract camera intrinsics
        const std::vector<std::vector<cv::Point3f>> worldPoints0
        {
            m_intrinsic_calib_image_points[0].size(),
            calculateWorldPoints()
        };
        cv::Mat rvecs0, tvecs0;
        cv::calibrateCamera(worldPoints0,m_intrinsic_calib_image_points[0],
                            m_resolution,
                            m_camera_intrinsics[0].m_K,
                            m_camera_intrinsics[0].m_D,
                            rvecs0, tvecs0,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        const std::vector<std::vector<cv::Point3f>> worldPoints1
        {
            m_intrinsic_calib_image_points[1].size(),
            calculateWorldPoints()
        };
        cv::Mat rvecs1, tvecs1;
        cv::calibrateCamera(worldPoints1,m_intrinsic_calib_image_points[1],
                            m_resolution,
                            m_camera_intrinsics[1].m_K,
                            m_camera_intrinsics[1].m_D,
                            rvecs1, tvecs1,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        // Extract laser intrinsics
        const std::vector<std::vector<cv::Point3f>> worldPoints2
        {
            m_intrinsic_calib_laser_points.size(),
            calculateWorldPoints()
        };
        cv::Mat rvecs2, tvecs2;
        cv::calibrateCamera(worldPoints2,m_intrinsic_calib_laser_points,
                            m_resolution,
                            m_camera_intrinsics[2].m_K,
                            m_camera_intrinsics[2].m_D,
                            rvecs2, tvecs2,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        // Extract extrinsics (camera0-camera1)
        float rms0 = cv::stereoCalibrate(
            worldPoints0,
            m_intrinsic_calib_image_points[0],
            m_intrinsic_calib_image_points[1],
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
        float rms1 = cv::stereoCalibrate(
            worldPoints1,
            m_intrinsic_calib_image_points[0],
            m_intrinsic_calib_laser_points,
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
        float rms2 = cv::stereoCalibrate(
            worldPoints2,
            m_intrinsic_calib_image_points[1],
            m_intrinsic_calib_laser_points,
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
               rms0/m_min_frames_to_capture,
               rms1/m_min_frames_to_capture,
               rms2/m_min_frames_to_capture);

        saveCamerasCalibration();
        saveLaserCalibration();
        m_triangulator->importCalibration();
        m_calibrate_laser = false;
    }
    return;
}

void StereoCalibrator::saveCamerasCalibration()
{
    cv::FileStorage fs(m_path_calib_cam, cv::FileStorage::WRITE);
    fs << "camera_matrix0" << m_camera_intrinsics[0].m_K;
    fs << "dist_coeffs0" << m_camera_intrinsics[0].m_D;
    fs << "camera_matrix1" << m_camera_intrinsics[1].m_K;
    fs << "dist_coeffs1" << m_camera_intrinsics[1].m_D;
    fs << "R" << m_R[0];
    fs << "T" << m_T[0];
    fs << "F" << m_F[0];
    fs.release();
    printf("Camera calibration saved ! \n\r");

    m_intrinsic_calib_image_points[0].clear();
    m_intrinsic_calib_image_points[1].clear();

    return;
}

void StereoCalibrator::saveLaserCalibration()
{
    cv::FileStorage fs(m_path_calib_laser, cv::FileStorage::WRITE);
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

    m_intrinsic_calib_image_points[0].clear();
    m_intrinsic_calib_image_points[1].clear();
    m_intrinsic_calib_laser_points.clear();

    return;
}

const std::vector<cv::Point3f> StereoCalibrator::calculateWorldPoints()
{
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<uint64_t>(m_pattern_size.x * m_pattern_size.y));

    for (int i = 0; i < m_pattern_size.y; i++)
    {
        for (int j = 0; j < m_pattern_size.x; j++)
        {
            points.emplace_back(cv::Point3f(j * m_pattern_square_size,
                                            i * m_pattern_square_size, 0));
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
    printf("Current laser position (%d,%d). \n\r",r,c);
    printf("Diff: (%d,%d) - MSE: %d. \n\r", diff[0],diff[1],mse);
    printf("Laser commands: (%d,%d). \n\r",m_laser->getX(),m_laser->getY());
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
    cv::Mat U = (cv::Mat_<double>(3,3) << s_2D[0],       0, mean2D[0],
                                                0, s_2D[1], mean2D[1],
                                                0,       0,         1);
    cv::invert(U,m_U);

    cv::Scalar s_3D = sigma3D/std::sqrt(3);
    cv::Mat T = (cv::Mat_<double>(4,4) << s_3D[0],       0,       0, mean3D[0],
                                                0, s_3D[1],       0, mean3D[1],
                                                0,       0, s_3D[2], mean3D[2],
                                                0,       0,       0,         1);
    cv::invert(T,m_T);

    // Normalize points
    //points2D = m_U * points2D;
    //points3D = m_T * points3D;
}


// Code from https://github.com/lagadic/camera_localization/blob/master/opencv/pose-basis/pose-dlt-opencv.cpp
void DLT::extractProjectionMatrix(const std::vector<cv::Point3d>& wX,
                                  const std::vector<cv::Point2d>& x,
                                  cv::Mat& ctw, cv::Mat& cRw)
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

    // Normalization to ensure that ||r3|| = 1
    double norm = sqrt(   rnv.at<double>(0,8)*rnv.at<double>(0,8)
                        + rnv.at<double>(0,9)*rnv.at<double>(0,9)
                        + rnv.at<double>(0,10)*rnv.at<double>(0,10));
    rnv /= norm;

    // Extract translation vector and rotation matrix
    for (int i = 0 ; i < 3 ; i++) {
        ctw.at<double>(i,0) = rnv.at<double>(0, 4*i+3); // Translation
        for (int j = 0 ; j < 3 ; j++)
          cRw.at<double>(i,j) = rnv.at<double>(0, 4*i+j); // Rotation
    }
}
