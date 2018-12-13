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
    cv::Mat_<float> m_K;
    cv::Mat_<float> m_D;
    IntrinsicsData();
};


class DLT
{
public:
    DLT();
    ~DLT();
    void normalizePoints(cv::Mat& points2D,
                         cv::Mat& points3D);
    void extractProjectionMatrix(//cv::Mat wX, cv::Mat x,
            const std::vector<cv::Point3d>& points3D,
                                 const std::vector<cv::Point2d>& points2D,
                                 cv::Mat& t, cv::Mat& R, cv::Mat& K);
    void test();
    void extractCalibration();

    // Normalization matrices
    cv::Mat m_U;
    cv::Mat m_T;

    //
    cv::Mat m_R;
    cv::Mat m_t;
};

class StereoCalibrator
{
public:
    StereoCalibrator(LaserController* laser=nullptr,
                     Filter* filter0=nullptr,
                     Filter* filter1=nullptr,
                     Triangulator* triangulator=nullptr);
    ~StereoCalibrator();

    void calibrateCameras(cv::Mat& frame, const int id);
    void calibrateLaser(cv::Mat& frame0, cv::Mat &frame1, const int id);
    void saveCamerasCalibration();
    void saveLaserCalibration();
    const std::vector<cv::Point3f> calculateWorldPoints();
    void pointLaserToPixel(const int u, const int v, const int id);

    float getLearningRate() const { return m_learningRate; }
    void setLearningRate(const float lr) { m_learningRate=lr; }

    // Who listens to
    LaserController* m_laser;
    std::array<Filter*,2> m_filter;
    Triangulator* m_triangulator;

    //
    const cv::Size2i m_resolution{240,180};
    bool m_calibrate_cameras{false};
    bool m_calibrate_laser{false};
    const std::string m_path_calib_cam;
    const std::string m_path_calib_laser;

    // Intrinsics
    const cv::Point2i m_pattern_size{8,5}; // 8-5
    const float m_pattern_square_size{0.03f};
    const size_t m_min_frames_to_capture{10};
    const int m_step{3};
    const std::chrono::milliseconds m_min_capture_delay{1000};
    std::array<std::chrono::high_resolution_clock::time_point,2> m_last_frame_captured;
    std::vector<std::vector<cv::Point3d>> m_intrinsic_calib_world_points;
    std::array<std::vector<std::vector<cv::Point2f>>, 2> m_intrinsic_calib_image_points;
    std::vector<std::vector<cv::Point2f>> m_intrinsic_calib_laser_points;
    std::array<IntrinsicsData,3> m_camera_intrinsics;

    // Stereo extrinsics
    std::array<cv::Mat,3> m_R, m_T, m_E, m_F;

    // Laser
    int m_threshConverged{2};
    float m_learningRate{0.3f};
};

#endif // EBV_STEREO_CALIBRATION_H
