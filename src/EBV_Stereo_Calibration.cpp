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
    : m_calibrate_cameras(false),
      m_calibrate_laser(false),
      m_camera_intrinsics{IntrinsicsData{},IntrinsicsData{}},
      m_last_frame_captured{std::chrono::high_resolution_clock::time_point{},
                            std::chrono::high_resolution_clock::time_point{}},
      m_laser(laser),
      m_filter{filter0,filter1},
      m_triangulator(triangulator),
      m_learningRate(0.3f)
{
    m_R.resize(0);
    m_T.resize(0);
    m_E.resize(0);
    m_F.resize(0);
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

        saveCamerasCalibration();
        m_triangulator->importCalibration();
        m_calibrate_cameras = false;
        printf("Finished extracting the extrinsics with a rms per frame of %f \n\r",
               rms/m_min_frames_to_capture);
        return;
    }
}

void StereoCalibrator::calibrateLaser(cv::Mat& frame, const uint id)
{
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
            printf("Calibrating laser-camera %d: %d frames retrieved. \n\r",id,
                   m_intrinsic_calib_image_points[id].size());

            // Finding laser command correspondance to each corner point
            std::vector<cv::Point2f> laserPoints;

            for (auto &point : points)
            {
                printf("Pointing to (%d,%d). \n\r",int(point.x),int(point.y));
                pointLaserToPixel(int(point.y),int(point.x),id);
                //auto laserPtn = cv::Point2f(m_laser->getY(), m_laser->getX());
                //laserPoints.push_back(laserPtn);
                laserPoints.emplace_back(m_laser->getY(), m_laser->getX());
            }
            m_intrinsic_calib_laser_points[id].push_back(laserPoints);
        }
    }

    // Extract intrinsics once enough points
    if (   m_intrinsic_calib_image_points[id].size()>=m_min_frames_to_capture
        && m_intrinsic_calib_laser_points[id].size()>=m_min_frames_to_capture)
    {
        // Extract camera intrinsics
        const std::vector<std::vector<cv::Point3f>> worldPoints0
        {
            m_intrinsic_calib_image_points[id].size(),
            calculateWorldPoints()
        };
        cv::Mat rvecs0, tvecs0;
        cv::calibrateCamera(worldPoints0,m_intrinsic_calib_image_points[id],
                            m_resolution,
                            m_camera_intrinsics[id].m_K,
                            m_camera_intrinsics[id].m_D,
                            rvecs0, tvecs0,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        // Extract laser intrinsics
        const std::vector<std::vector<cv::Point3f>> worldPoints1
        {
            m_intrinsic_calib_laser_points[id].size(),
            calculateWorldPoints()
        };
        cv::Mat rvecs1, tvecs1;
        cv::calibrateCamera(worldPoints0,m_intrinsic_calib_laser_points[id],
                            m_resolution,
                            m_camera_intrinsics[2].m_K,
                            m_camera_intrinsics[2].m_D,
                            rvecs1, tvecs1,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        // Extract extrinsics
        double rms = cv::stereoCalibrate(
            worldPoints1, m_intrinsic_calib_image_points[id],
            m_intrinsic_calib_laser_points[id],
            m_camera_intrinsics[id].m_K,
            m_camera_intrinsics[id].m_D,
            m_camera_intrinsics[2].m_K,
            m_camera_intrinsics[2].m_D,
            m_resolution, m_R, m_T, m_E, m_F,
            cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::COUNT
                             + cv::TermCriteria::EPS, 60,
                             DBL_EPSILON));

        saveLaserCalibration(id);
        m_triangulator->importCalibration();
        m_calibrate_laser = false;
        printf("Finished extracting the extrinsics with a rms per frame of %f \n\r",
               rms/m_min_frames_to_capture);

        return;
    }
}

void StereoCalibrator::saveCamerasCalibration()
{
    cv::FileStorage fs(m_path_calib_cam, cv::FileStorage::WRITE);
    fs << "camera_matrix0" << m_camera_intrinsics[0].m_K;
    fs << "dist_coeffs0" << m_camera_intrinsics[0].m_D;
    fs << "camera_matrix1" << m_camera_intrinsics[1].m_K;
    fs << "dist_coeffs1" << m_camera_intrinsics[1].m_D;
    fs << "R" << m_R;
    fs << "T" << m_T;
    fs << "F" << m_F;
    fs.release();
    printf("Camera calibration saved ! \n\r");
    return;
}

void StereoCalibrator::saveLaserCalibration(const int id)
{
    cv::FileStorage fs(m_path_calib_laser, cv::FileStorage::WRITE);
    fs << "camera_matrix " + std::to_string(id) << m_camera_intrinsics[id].m_K;
    fs << "dist_coeffs " + std::to_string(id) << m_camera_intrinsics[id].m_D;
    fs << "camera_matrix_laser" << m_camera_intrinsics[2].m_K;
    fs << "dist_coeffs_laser" << m_camera_intrinsics[2].m_D;
    fs << "R" << m_R;
    fs << "T" << m_T;
    fs << "F" << m_F;
    fs.release();
    printf("Laser calibration saved ! \n\r");
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
    m_laser->start();
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

        int m_step = 4;

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
