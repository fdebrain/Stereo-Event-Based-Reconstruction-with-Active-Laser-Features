#ifndef EBV_VISUALIZER_H
#define EBV_VISUALIZER_H

#include <fstream>
#include <string>
#include <mutex>

#include <EBV_DVS128USB.h>
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
    Visualizer(const uint nbCams,
               DAVIS240C* davis0 = nullptr,
               DAVIS240C* davis1 = nullptr,
               Filter* filter0 = nullptr,
               Filter* filter1 = nullptr,
               StereoCalibrator* calibrator = nullptr,
               Triangulator* triangulator = nullptr,
               LaserController* laser = nullptr);
    ~Visualizer();

    //void receivedNewDVS128USBEvent(DVS128USBEvent& e);
    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const uint id) override;
    void receivedNewDAVIS240CFrame(DAVIS240CFrame& f,
                                   const uint id) override;
    void receivedNewFilterEvent(DAVIS240CEvent& e,
                                const uint id) override;
    void receivedNewDepth(const int &u,
                          const int &v,
                          const double &X,
                          const double &Y,
                          const double &Z) override;

    void run();

public:
    // Parameters for camera settings
    const int          m_rows;
    const int          m_cols;
    const uint         m_nbCams;

    // Parameters related to filtering events above threshold age
    std::array<int,2>           m_currenTime{};

    // Structures for data storing
    std::array<std::vector<bool>,2>    m_polEvts;
    std::array<std::vector<int>,2>     m_ageEvts;
    std::array<cv::Mat,2>              m_grayFrame;
    std::array<std::vector<int>,2>     m_filtEvts;
    std::vector<double>                m_depthMap;

    // Display window related variables
    std::array<std::string,2>   m_polWin{};
    std::array<std::string,2>   m_ageWin{};
    std::array<std::string,2>   m_frameWin{};
    std::array<std::string,2>   m_filtWin{};
    std::string                 m_depthWin{};

    // Trackbar parameters (events age)
    int        m_ageThresh{};
    int        m_max_trackbar_val{};

    // Trackbar parameters (laser)
    int m_laserX{};
    int m_laserY{};
    int m_laserVx{};
    int m_laserVy{};
    int m_lrInt{};
    int m_stepInt{};
    int m_laserFreq{};

    // Trackbar parameters (filter)
    std::array<int,2> m_filter_freq{};
    std::array<int,2> m_filter_eps{};
    std::array<int,2> m_filter_neighborSize{};
    std::array<int,2> m_filter_threshA{};
    std::array<int,2> m_filter_threshB{};
    std::array<int,2> m_filter_threshAnti{};
    std::array<int,2> m_filter_etaInt{};
    std::array<int,2> m_filter_sigma{};
    std::array<int,2> m_filter_max_t{};

    // Trackbar parameters (depth in c,)
    int        m_min_depth{};
    int        m_max_depth{};
    int        m_matcherEps{};
    int        m_matcherMaxBuffer{};

    // Event recorder
    std::ofstream m_recorder;

    // Davis object
    std::array<DAVIS240C*,2>          m_davis;

    // Laser object
    LaserController* m_laser;

    // Filter object
    std::array<Filter*,2>             m_filter;
    std::mutex                        m_laserMutex;

    // Triangulator (depth estimator)
    StereoCalibrator*    m_calibrator;
    Triangulator*        m_triangulator;
};

#endif // EBV_VISUALIZER_H
