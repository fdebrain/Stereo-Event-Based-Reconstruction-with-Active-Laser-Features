#ifndef EBV_TRIANGULATOR_H
#define EBV_TRIANGULATOR_H

#include <EBV_Matcher.h>

//====
class StereoRectificationData {
  public:
    std::array<cv::Mat_<double>, 2> R;
    std::array<cv::Mat_<double>, 2> P;
    cv::Mat_<double> Q;
    /**
     * For each camera, contains the map for x and y coordinates
     */
    std::array<std::pair<cv::Mat_<float>, cv::Mat_<float>>, 2> maps;

    StereoRectificationData();
    bool is_valid() const;
};
//====

class TriangulatorListener
{
public:
    TriangulatorListener(void) {}
    virtual void receivedNewDepth(const unsigned int &u,
                                  const unsigned int &v,
                                  const double &depth) = 0;
};

class Triangulator : public MatcherListener
{
public:
    Triangulator(const unsigned int rows,
                 const unsigned int cols,
                 Matcher* matcher = nullptr);
    ~Triangulator();

    void importCalibration(std::string path);
    void run();
    void receivedNewMatch(const DAVIS240CEvent& event1,
                          const DAVIS240CEvent& event2) override;
    void process(const DAVIS240CEvent& event0,
                 const DAVIS240CEvent& event1);

    void registerTriangulatorListener(TriangulatorListener* listener);
    void deregisterTriangulatorListener(TriangulatorListener* listener);
    void warnDepth(const unsigned int u,
                   const unsigned int v,
                   const double depth);

private:
    const unsigned int m_rows;
    const unsigned int m_cols;

    // Calibration related
    std::string m_pathCalib = "../calibration/calib.xml";
    cv::Mat m_K0 = cv::Mat(3,3,CV_32FC1);
    cv::Mat m_D0 = cv::Mat(1,5,CV_32FC1);
    cv::Mat m_P0 = cv::Mat(3,4,CV_32FC1);

    cv::Mat m_R = cv::Mat(3,3,CV_32FC1);
    cv::Mat m_T = cv::Mat(1,3,CV_32FC1);
    cv::Mat m_F = cv::Mat(3,3,CV_32FC1);

    cv::Mat m_K1 = cv::Mat(3,3,CV_32FC1);
    cv::Mat m_D1 = cv::Mat(1,5,CV_32FC1);
    cv::Mat m_P1 = cv::Mat(3,4,CV_32FC1);

    // Datastructure containing rectify transforms (parallel epipolar lines)
    StereoRectificationData m_rect{};

    // Thread this object runs in.
    std::thread m_thread;

    // List of incoming filtered events for each camera (FIFO)
    std::list<DAVIS240CEvent> m_evtQueue0;
    std::list<DAVIS240CEvent> m_evtQueue1;

    // Mutex to access the queue
    std::mutex m_queueAccessMutex0;
    std::mutex m_queueAccessMutex1;

    // Wait when no processing has to be done
    std::condition_variable m_condWait;
    std::mutex m_condWaitMutex;

    // Who triangulator is listening to
    Matcher* m_matcher;

    // List of triangulator listeners
    std::list<TriangulatorListener*> m_triangulatorListeners;
};

#endif // EBV_TRIANGULATOR_H


