#include <EBV_Stereo_Calibration.h>

IntrinsicsData::IntrinsicsData()
{
    m_K.resize(0);
    m_D.resize(0);
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
      m_camera_intrinsics{IntrinsicsData{},IntrinsicsData{}},
      m_learningRate(0.3f)
{
    for (auto& m: m_R) { m.resize(0); }
    for (auto& m: m_T) { m.resize(0); }
    for (auto& m: m_E) { m.resize(0); }
    for (auto& m: m_F) { m.resize(0); }
}

StereoCalibrator::~StereoCalibrator(){}

void StereoCalibrator::calibrateCameras(cv::Mat &frame, const uint id)
{
    // Assess enough time between two frame captures
    const auto now = std::chrono::high_resolution_clock::now();
    if (now-m_last_frame_captured[id]<m_min_capture_delay) {return;}
    m_last_frame_captured[id] = now;

    // Extract checkerboard corner points
    if (m_intrinsic_calib_image_points[id].size()<m_min_frames_to_capture)
    {
        std::vector<cv::Point2f> points;
        bool ret = cv::findChessboardCorners(frame,
                                             m_pattern_size,
                                             points);
        if (ret)
        {
            cv::cornerSubPix(frame,points,cv::Size(5,5),cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::COUNT
                             + cv::TermCriteria::EPS,30, DBL_EPSILON));
            m_intrinsic_calib_image_points[id].push_back(points);
            cv::drawChessboardCorners(frame,m_pattern_size,points,ret);
            printf("Calibrating camera %d: %d frames retrieved. \n\r",id,
                   m_intrinsic_calib_image_points[id].size());
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
        cv::calibrateCamera(worldPoints0,m_intrinsic_calib_image_points[1],
                            m_resolution,
                            m_camera_intrinsics[1].m_K,
                            m_camera_intrinsics[1].m_D,
                            rvecs1, tvecs1,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        // Extract extrinsics
        double rms = cv::stereoCalibrate(
            worldPoints1,
            m_intrinsic_calib_image_points[0],
            m_intrinsic_calib_image_points[1],
            m_camera_intrinsics[0].m_K,
            m_camera_intrinsics[0].m_D,
            m_camera_intrinsics[1].m_K,
            m_camera_intrinsics[1].m_D,
            m_resolution, m_R, m_T, m_E, m_F,
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
                                      const uint id)
{   
    // Extract checkerboard corner points and corresponding laser commands
    if (   m_intrinsic_calib_image_points[0].size()<m_min_frames_to_capture
        && m_intrinsic_calib_image_points[1].size()<m_min_frames_to_capture)
    {
        std::vector<cv::Point2f> points0;
        std::vector<cv::Point2f> points1;
        bool ret0 = cv::findChessboardCorners(frame0,
                                             m_pattern_size,
                                             points0);
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

            // Finding laser command correspondance to each corner point
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

        // Extract laser intrinsics
        const std::vector<std::vector<cv::Point3f>> worldPoints
        {
            m_intrinsic_calib_laser_points.size(),
            calculateWorldPoints()
        };
        cv::Mat rvecs, tvecs;
        cv::calibrateCamera(worldPoints,m_intrinsic_calib_laser_points,
                            m_resolution,
                            m_camera_intrinsics[2].m_K,
                            m_camera_intrinsics[2].m_D,
                            rvecs, tvecs,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        // Extract extrinsics (Camera0-Laser)
        double rms0 = cv::stereoCalibrate(
            worldPoints,
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
        double rms1 = cv::stereoCalibrate(
            worldPoints,
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

        printf("Finished extracting the extrinsics with rms %.3f - %.3f. \n\r",
               rms0/m_min_frames_to_capture,rms1/m_min_frames_to_capture);

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
    return;
}

void StereoCalibrator::saveLaserCalibration()
{
    cv::FileStorage fs(m_path_calib_laser, cv::FileStorage::WRITE);
    fs << "camera_matrix0" << m_camera_intrinsics[0].m_K;
    fs << "dist_coeffs0" << m_camera_intrinsics[0].m_D;
    fs << "camera_matrix1" << m_camera_intrinsics[1].m_K;
    fs << "dist_coeffs1" << m_camera_intrinsics[1].m_D;
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

    for (int i = 0; i < m_pattern_size.y; i++) {
        for (int j = 0; j < m_pattern_size.x; j++) {
            points.emplace_back(cv::Point3f(j * m_pattern_square_size,
                                            i * m_pattern_square_size, 0));
        }
    }
    return points;
}

void StereoCalibrator::pointLaserToPixel(const int rGoal, const int cGoal, const uint id)
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
        if (m_learningRate*diff[0]>m_step)
        {
            dx = m_learningRate*diff[0];
        }
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
    printf("Diff: (%d,%d) - MSE: %d. \n\r",
           diff[0],diff[1],mse);
    printf("Laser commands: (%d,%d). \n\r",m_laser->getX(),m_laser->getY());
    //=== END DEBUG ===//
    return;
}
