#ifndef EBV_VISUALIZER_H
#define EBV_VISUALIZER_H

#include <EBV_DVS128USB.h>
#include <EBV_DAVIS240C.h>
#include <EBV_MagneticMirrorLaser.h>
#include <EBV_Filter.h>
#include <EBV_Triangulator.h>

#include <string>
#include <mutex>

namespace cv { class Mat; }

class Visualizer : public DVS128USBListener,
                   public DAVIS240CListener,
                   public FilterListener,
                   public TriangulatorListener

{

public:
    Visualizer(int rows, int cols, int nbCams,
               DAVIS240C* davis0 = nullptr,
               DAVIS240C* davis1 = nullptr,
               Filter* filter0 = nullptr,
               Filter* filter1 = nullptr,
               Triangulator* triangulator = nullptr);
    ~Visualizer();

    void receivedNewDVS128USBEvent(DVS128USBEvent& e);
    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e, int id);
    void receivedNewDAVIS240CFrame(DAVIS240CFrame& f, int id);
    void receivedNewFilterEvent(DAVIS240CEvent& e, int id);
    void receivedNewDepth(int &u, int &v, float &depth);

    void run();

public:
    // Parameters for camera settings
    int                 m_rows;
    int                 m_cols;
    int                 m_nbCams;

    // Parameters related to filtering events above threshold age
    unsigned int        m_currenTime0;
    unsigned int        m_currenTime1;
    int                 m_thresh;
    int                 m_max_trackbar_val=1e6;
    //====
    float                 m_min_depth = 0.1;
    float                 m_max_depth = 0.3;
    //====

    // Structures for data storing (flattened matrices)
    std::vector<int>            m_polEvts0;
    std::vector<unsigned int>   m_ageEvts0;
    std::vector<int>            m_filtEvts0;
    std::vector<int>            m_polEvts1;
    std::vector<unsigned int>   m_ageEvts1;
    std::vector<int>            m_filtEvts1;
    std::vector<float>          m_depthMap;

    // Thread-safety
    std::mutex                  m_evtMutex;
    std::mutex                  m_frameMutex;
    std::mutex                  m_filterEvtMutex;
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

    // Laser object
    MagneticMirrorLaser m_laser;

    // Davis object
    DAVIS240C* m_davis0;
    DAVIS240C* m_davis1;

    // Filter object
    Filter*             m_filter0;
    Filter*             m_filter1;

    // Triangulator (depth estimator)
    Triangulator*       m_triangulator;
};

#endif // EBV_VISUALIZER_H
