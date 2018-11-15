#ifndef EBV_STEREO_CALIBRATION_H
#define EBV_STEREO_CALIBRATION_H

#include <array>
#include <chrono>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <EBV_LaserController.h>
#include <EBV_Triangulator.h>

class IntrinsicsData
{
public:
    cv::Mat_<double> m_K;
    cv::Mat_<double> m_D;
    IntrinsicsData();
};

class StereoCalibrator
{
public:
    StereoCalibrator(Triangulator* triangulator=nullptr);
    ~StereoCalibrator();

    void calibrate(cv::Mat &frame, const int id);
    void saveCalibration();
    const std::vector<cv::Point3f> calculateWorldPoints();

    const cv::Size2i m_resolution{240,180};
    bool m_calibrateCameras{false};
    std::string m_pathCalib = "../calibration/calib2.yaml";

    // Intrinsics
    const cv::Point2i m_pattern_size{8, 5};
    const float m_pattern_square_size = 3.0f;
    const size_t m_min_frames_to_capture{25};
    const std::chrono::milliseconds m_min_capture_delay{1000};
    std::array<IntrinsicsData,2> m_camera_intrinsics;
    std::array<std::chrono::high_resolution_clock::time_point, 2> m_last_frame_captured;
    std::vector<std::vector<cv::Point3d>> m_intrinsic_calib_world_points;
    std::array<std::vector<std::vector<cv::Point2f>>, 2> m_intrinsic_calib_image_points;

    // Stereo extrinsics
    cv::Mat_<double> m_R, m_T, m_E, m_F;

    Triangulator* m_triangulator;
};

#endif // EBV_STEREO_CALIBRATION_H