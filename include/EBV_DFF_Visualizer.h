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
    Visualizer(const unsigned int rows,
               const unsigned int cols,
               const unsigned int nbCams,
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
    const unsigned int          m_rows;
    const unsigned int          m_cols;
    const unsigned int          m_nbCams;

    // Parameters related to filtering events above threshold age
    std::array<int,2>           m_currenTime;

    // Structures for data storing (flattened matrices)
    std::array<std::vector<int>,2>        m_polEvts;
    std::array<std::vector<int>,2>        m_ageEvts;
    std::array<std::vector<int>,2>        m_filtEvts;
    //std::vector<int>            m_polEvts1;
    //std::vector<int>            m_ageEvts1;
    //std::vector<int>            m_filtEvts1;
    std::vector<double>         m_depthMap;

    // Thread-safety
    std::mutex                  m_evtMutex0;
    std::mutex                  m_frameMutex0;
    std::mutex                  m_filterEvtMutex0;
    std::mutex                  m_evtMutex1;
    std::mutex                  m_frameMutex1;
    std::mutex                  m_filterEvtMutex1;
    std::mutex                  m_depthMutex;

    // Structures for displaying camera frames
    std::array<cv::Mat,2>         m_grayFrame;

    // Display window related variables
    std::string                 m_polWin0;
    std::string                 m_ageWin0;
    std::string                 m_frameWin0;
    std::string                 m_filtWin0;
    std::string                 m_polWin1;
    std::string                 m_ageWin1;
    std::string                 m_frameWin1;
    std::string                 m_filtWin1;
    std::string                 m_depthWin;

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
    int m_freq0;
    int m_eps0;
    int m_neighborSize0;
    int m_threshA0;
    int m_threshB0;
    int m_threshAnti0;
    int m_etaInt0;

    int m_freq1;
    int m_eps1;
    int m_neighborSize1;
    int m_threshA1;
    int m_threshB1;
    int m_threshAnti1;
    int m_etaInt1;

    // Trackbar parameters (depth in c,)
    int        m_min_depth;
    int        m_max_depth;
    int        m_matcherEps;
    int        m_matcherMaxBuffer;

    // Event recorder
    std::ofstream m_recorder;

    // Laser object
    LaserController* m_laser;

    // Davis object
    DAVIS240C*          m_davis0;
    DAVIS240C*          m_davis1;

    // Filter object
    Filter*             m_filter0;
    Filter*             m_filter1;

    // Triangulator (depth estimator)
    StereoCalibrator*    m_calibrator;
    Triangulator*       m_triangulator;
};

#endif // EBV_VISUALIZER_H
