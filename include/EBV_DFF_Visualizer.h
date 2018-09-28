#ifndef EBV_VISUALIZER_H
#define EBV_VISUALIZER_H

#include <EBV_DVS128USB.h>
#include <EBV_DAVIS240C.h>
#include <string>
#include <mutex>

class DepthEstimator;

namespace cv {
    class Mat;
}

class Visualizer : public DVS128USBListener,
                   public DAVIS240CListener
{

public:
    Visualizer(int rows, int cols, int id);
    ~Visualizer();

    void receivedNewDVS128USBEvent(DVS128USBEvent& e);
    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e);
    void receivedNewDAVIS240CFrame(DAVIS240CFrame& f);

    void run();

public:
    int                 m_rows;
    int                 m_cols;

    //
    unsigned int        m_currenTime;
    int                 m_thresh;
    int                 m_max_trackbar_val=1e6;

    //
    std::vector<int>            m_polEvts;
    std::vector<unsigned int>   m_ageEvts;
    std::mutex                  m_evtMutex;
    cv::Mat                     m_grayFrame;

    // Display window related variables
    std::string         m_polWin;
    std::string         m_ageWin;
    std::string         m_frameWin;

};

#endif // EBV_VISUALIZER_H
