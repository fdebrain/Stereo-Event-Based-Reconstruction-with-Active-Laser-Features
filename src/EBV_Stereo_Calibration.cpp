#include <EBV_Stereo_Calibration.h>

IntrinsicsData::IntrinsicsData()
{
    m_K.resize(0);
    m_D.resize(0);
}

StereoCalibrator::StereoCalibrator(Triangulator* triangulator)
    : m_calibrateCameras(false),
      m_camera_intrinsics{IntrinsicsData{},IntrinsicsData{}},
      m_last_frame_captured{std::chrono::high_resolution_clock::time_point{},
                            std::chrono::high_resolution_clock::time_point{}},
      m_triangulator(triangulator)
{
    m_R.resize(0);
    m_T.resize(0);
    m_E.resize(0);
    m_F.resize(0);
}

StereoCalibrator::~StereoCalibrator(){}

void StereoCalibrator::calibrate(cv::Mat &frame, const int id)
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
        { m_intrinsic_calib_image_points[0].size(), calculateWorldPoints() };
        cv::Mat rvecs0, tvecs0;
        cv::calibrateCamera(worldPoints0,
                            m_intrinsic_calib_image_points[0],
                            m_resolution,
                            m_camera_intrinsics[0].m_K,
                            m_camera_intrinsics[0].m_D,
                            rvecs0, tvecs0,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        const std::vector<std::vector<cv::Point3f>> worldPoints1
        { m_intrinsic_calib_image_points[1].size(), calculateWorldPoints() };
        cv::Mat rvecs1, tvecs1;
        cv::calibrateCamera(worldPoints1,
                            m_intrinsic_calib_image_points[1],
                            m_resolution,
                            m_camera_intrinsics[1].m_K,
                            m_camera_intrinsics[1].m_D,
                            rvecs1, tvecs1,0,
                            cv::TermCriteria(cv::TermCriteria::COUNT
                                             + cv::TermCriteria::EPS,
                                             60, DBL_EPSILON));

        // Extract extrinsics
        double rms = cv::stereoCalibrate(
            worldPoints1, m_intrinsic_calib_image_points[0],
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

        saveCalibration();
        m_triangulator->importCalibration();
        m_calibrateCameras = false;
        printf("Finished extracting the extrinsics with a rms per frame of %f \n\r",
               rms/m_min_frames_to_capture);
        return;
    }
}

void StereoCalibrator::saveCalibration()
{
    cv::FileStorage fs(m_pathCalib, cv::FileStorage::WRITE);
    fs << "camera_matrix0" << m_camera_intrinsics[0].m_K;
    fs << "dist_coeffs0" << m_camera_intrinsics[0].m_D;
    fs << "camera_matrix1" << m_camera_intrinsics[1].m_K;
    fs << "dist_coeffs1" << m_camera_intrinsics[1].m_D;
    fs << "R" << m_R;
    fs << "T" << m_T;
    fs << "F" << m_F;
    fs.release();
    printf("Calibration saved ! \n\r");
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
