#ifndef EBV_VISUALIZER_H
#define EBV_VISUALIZER_H

#include <EBV_DVS128USB.h>
#include <EBV_DAVIS240C.h>
#include <EBV_MagneticMirrorLaser.h>
#include <EBV_Filter.h>
#include <EBV_Triangulator.h>
#include <EBV_LaserController.h>
#include <EBV_Matcher.h>
#include <EBV_Stereo_Calibration.h>

#include <fstream>
#include <string>
#include <mutex>

// Forward declaration
namespace cv { class Mat;}

class Visualizer : //public DVS128USBListener,
                   public DAVIS240CEventListener,
                   public DAVIS240CFrameListener,
                   public FilterListener,
                   public TriangulatorListener
{

public:
    Visualizer(const unsigned int nbCams,
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
                                   const unsigned int id) override;
    void receivedNewDAVIS240CFrame(DAVIS240CFrame& f,
                                   const unsigned int id) override;
    void receivedNewFilterEvent(DAVIS240CEvent& e,
                                const unsigned int id) override;
    void receivedNewDepth(const unsigned int &u,
                          const unsigned int &v,
                          const double &X,
                          const double &Y,
                          const double &Z) override;

    void run();

public:
    // Parameters for camera settings
    const int                   m_rows;
    const int                   m_cols;
    const unsigned int          m_nbCams;

    // Parameters related to filtering events above threshold age
    std::array<int,2>           m_currenTime;

    // Structures for data storing
    std::array<std::vector<bool>,2>        m_polEvts;
    std::array<std::vector<int>,2>         m_ageEvts;
    std::array<std::vector<int>,2>         m_filtEvts;
    std::array<cv::Mat_<uchar>,2>          m_grayFrame;
    std::vector<double>                    m_depthMap;

    // Thread-safety
    std::mutex                  m_evtMutex0;
    std::mutex                  m_frameMutex0;
    std::mutex                  m_filterEvtMutex0;
    std::mutex                  m_evtMutex1;
    std::mutex                  m_frameMutex1;
    std::mutex                  m_filterEvtMutex1;
    std::mutex                  m_depthMutex;

    // Display window related variables
    std::array<std::string,2>     m_polWin;
    std::array<std::string,2>     m_ageWin;
    std::array<std::string,2>     m_frameWin;
    std::array<std::string,2>     m_filtWin;
    std::string                   m_depthWin;

    // Trackbar parameters (events age)
    int        m_ageThresh;
    int        m_max_trackbar_val;

    // Trackbar parameters (laser)
    int m_cx;
    int m_cy;
    int m_r;
    int m_stepInt;
    int m_laserFreq;

    // Trackbar parameters (filter)
    std::array<int,2> m_freq;
    std::array<int,2> m_eps;
    std::array<int,2> m_neighborSize;
    std::array<int,2> m_threshA;
    std::array<int,2> m_threshB;
    std::array<int,2> m_threshAnti;
    std::array<int,2> m_etaInt;

    // Trackbar parameters (depth in c,)
    int        m_min_depth;
    int        m_max_depth;
    int        m_matcherEps;
    int        m_matcherMaxBuffer;

    // Event recorder
    std::ofstream       m_recorder;

    // Laser object
    LaserController*    m_laser;

    // Davis object
    std::array<DAVIS240C*,2>  m_davis;

    // Filter object
    std::array<Filter*,2>     m_filter;

    // Triangulator (depth estimator)
    Triangulator*       m_triangulator;
    StereoCalibrator*   m_calibrator;
};

#endif // EBV_VISUALIZER_H
