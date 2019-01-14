#ifndef EBV_VISUALIZER_H
#define EBV_VISUALIZER_H

#include <fstream>
#include <string>
#include <mutex>

#include <EBV_DAVIS240C.h>
#include <EBV_MagneticMirrorLaser.h>
#include <EBV_Filter.h>
#include <EBV_Triangulator.h>
#include <EBV_LaserController.h>
#include <EBV_Matcher.h>
#include <EBV_Stereo_Calibration.h>

// Forward declaration
namespace cv { class Mat;}

class Visualizer : public DAVIS240CEventListener,
                   public DAVIS240CFrameListener,
                   public FilterListener,
                   public TriangulatorListener
{
public:
    Visualizer(const int nbCams,
               DAVIS240C* davis0 = nullptr,
               DAVIS240C* davis1 = nullptr,
               Filter* filter0 = nullptr,
               Filter* filter1 = nullptr,
               StereoCalibrator* calibrator = nullptr,
               Triangulator* triangulator = nullptr,
               LaserController* laser = nullptr);
    ~Visualizer();

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const int id) override;
    void receivedNewDAVIS240CFrame(DAVIS240CFrame& f,
                                   const int id) override;
    void receivedNewFilterEvent(DAVIS240CEvent& e,
                                const int id) override;
    void receivedNewDepth(const int &u,
                          const int &v,
                          const double &X,
                          const double &Y,
                          const double &Z) override;
    void run();

public:
    // Camera settings
    const int          m_rows{180};
    const int          m_cols{240};
    const int          m_nbCams{2};

    // Who visualizer listens to
    std::array<DAVIS240C*,2>          m_davis;
    LaserController*                  m_laser;
    std::array<Filter*,2>             m_filter;
    StereoCalibrator*                 m_calibrator;
    Triangulator*                     m_triangulator;

    // Parameters related to filtering events above threshold age
    std::array<int,2>           m_currenTime{};
    std::string m_depthmap_path{"../experiments/Depthmaps/depthmap_scene_Cam0Cam1"};

    // Structures for data storing
    std::array<std::vector<bool>,2>    m_pol_evts;
    std::array<std::vector<int>,2>     m_age_evts;
    std::array<cv::Mat,2>              m_frame;
    std::array<std::vector<int>,2>     m_filt_evts;
    std::vector<float>                 m_depthmap;
    cv::Mat                            m_mask;

    // Display window related variables
    std::array<std::string,2>   m_pol_win{};
    std::array<std::string,2>   m_age_win{};
    std::array<std::string,2>   m_frame_win{};
    std::array<std::string,2>   m_filt_win{};
    std::string                 m_depth_win{};
    std::string                 m_depth_inpainted_win{};

    // Trackbar parameters (events age)
    int        m_age_thresh = int(40e3);
    int        m_max_trackbar_val = 1e6;

    // Trackbar parameters (laser)
    std::array<int,2> m_laser_pos{};
    std::array<int,2> m_laser_vel{};
    int m_lrInt{};
    int m_laser_freq{};
    int m_laser_step{};
    int m_laser_ratio_int{};
    std::array<int,4> m_laser_boundaries{};

    // Trackbar parameters (filter)
    std::array<int,2> m_filter_freq{};
    std::array<int,2> m_filter_eps{};
    std::array<int,2> m_filter_etaInt{};
    std::array<int,2> m_filter_sigma{};
    std::array<int,2> m_filter_max_t{};

    // Trackbar parameters
    int        m_min_depth{34};
    int        m_max_depth{62};
    int        m_matcherEps{};
    int        m_alphaInt{50};
};

#endif // EBV_VISUALIZER_H
