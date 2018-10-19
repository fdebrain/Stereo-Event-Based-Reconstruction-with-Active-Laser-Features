#ifndef EBV_VISUALIZER_H
#define EBV_VISUALIZER_H

#include <EBV_DVS128USB.h>
#include <EBV_DAVIS240C.h>
#include <EBV_MagneticMirrorLaser.h>
#include <EBV_Filter.h>

#include <string>
#include <mutex>

class DepthEstimator;

namespace cv {
    class Mat;
}

class Filter;

class Visualizer : public DVS128USBListener,
                   public DAVIS240CListener,
                   public FilterListener
{

public:
    Visualizer(int rows, int cols, int nbCams,
               Filter* filter0 = nullptr,
               Filter* filter1 = nullptr);
    ~Visualizer();

    void receivedNewDVS128USBEvent(DVS128USBEvent& e);
    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e, int id);
    void receivedNewDAVIS240CFrame(DAVIS240CFrame& f, int id);
    void receivedNewFilterEvent(DAVIS240CEvent& e, int id);
    void run();
    void setFilter(Filter* filter, int id);

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

    // Structures for data storing (flattened matrices)
    std::vector<int>            m_polEvts0;
    std::vector<unsigned int>   m_ageEvts0;
    std::vector<int>            m_filtEvts0;
    std::vector<int>            m_polEvts1;
    std::vector<unsigned int>   m_ageEvts1;
    std::vector<int>            m_filtEvts1;

    // Thread-safety
    std::mutex                  m_evtMutex;

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

    // Filter object
    Filter*             m_filter0;
    Filter*             m_filter1;
};

#endif // EBV_VISUALIZER_H
