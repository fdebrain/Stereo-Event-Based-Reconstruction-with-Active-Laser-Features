#ifndef EBV_VISUALIZER_H
#define EBV_VISUALIZER_H

#include <EBV_DVS128USB.h>
#include <EBV_DAVIS240C.h>
#include <EBV_MagneticMirrorLaser.h>
#include <EBV_Filter.h>
#include <EBV_Triangulator.h>

#include <fstream>
#include <string>
#include <mutex>

namespace cv { class Mat;}

class Visualizer : public DVS128USBListener,
                   public DAVIS240CListener,
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
               Triangulator* triangulator = nullptr);
    ~Visualizer();

    void receivedNewDVS128USBEvent(DVS128USBEvent& e);
    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const unsigned int id);
    void receivedNewDAVIS240CFrame(DAVIS240CFrame& f,
                                   const unsigned int id);
    void receivedNewFilterEvent(DAVIS240CEvent& e,
                                const unsigned int id);
    void receivedNewDepth(const unsigned int &u,
                          const unsigned int &v,
                          const double &depth);

    void run();

public:
    // Parameters for camera settings
    const unsigned int          m_rows;
    const unsigned int          m_cols;
    const unsigned int          m_nbCams;

    // Parameters related to filtering events above threshold age
    int        m_currenTime0;
    int        m_currenTime1;
    int        m_thresh;
    int        m_max_trackbar_val=1e6;
    int        m_min_depth = 1100; // = 10 cm
    int        m_max_depth = 1600; // = 20 cm

    // Structures for data storing (flattened matrices)
    std::vector<int>            m_polEvts0;
    std::vector<int>            m_ageEvts0;
    std::vector<int>            m_filtEvts0;
    std::vector<int>            m_polEvts1;
    std::vector<int>            m_ageEvts1;
    std::vector<int>            m_filtEvts1;
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
    cv::Mat                     m_grayFrame0;
    cv::Mat                     m_grayFrame1;

    // Display window related variables
    std::string         m_polWin0;
    std::string         m_ageWin0;
    std::string         m_frameWin0;
    std::string         m_filtWin0;
    std::string         m_polWin1;
    std::string         m_ageWin1;
    std::string         m_frameWin1;
    std::string         m_filtWin1;
    std::string         m_depthWin;

    // Trackbar parameters
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

    // Event recorder
    std::ofstream m_recorder;

    // Frame recordere

    // Laser object
    MagneticMirrorLaser m_laser;

    // Davis object
    DAVIS240C*          m_davis0;
    DAVIS240C*          m_davis1;

    // Filter object
    Filter*             m_filter0;
    Filter*             m_filter1;

    // Triangulator (depth estimator)
    Triangulator*       m_triangulator;
};

#endif // EBV_VISUALIZER_H
