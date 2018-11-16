#ifndef EBV_TRIANGULATOR_H
#define EBV_TRIANGULATOR_H

#include <EBV_DAVIS240C.h>
#include <EBV_Matcher.h>
#include <EBV_LaserController.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

class TriangulatorListener
{
public:
    TriangulatorListener(void) {}
    virtual void receivedNewDepth(const int &u,
                                  const int &v,
                                  const double &X,
                                  const double &Y,
                                  const double &Z) = 0;
};

class Triangulator : public MatcherListener
{
public:
    Triangulator(Matcher* matcher = nullptr,
                 LaserController* laser = nullptr);
    ~Triangulator();

    void calibrateLaser();
    void importCalibration();
    void run();
    void receivedNewMatch(const DAVIS240CEvent& event1,
                          const DAVIS240CEvent& event2) override;
    void process(const DAVIS240CEvent& event0,
                 const DAVIS240CEvent& event1);

    void registerTriangulatorListener(TriangulatorListener* listener);
    void deregisterTriangulatorListener(TriangulatorListener* listener);
    void warnDepth(const int u,
                   const int v,
                   const double X,
                   const double Y,
                   const double Z);

    const int m_rows;
    const int m_cols;

private:
    // Who triangulator is listening to
    Matcher* m_matcher;
    LaserController* m_laser;

    // Calibration matrices (camera0,camera1,laser)
    std::string m_pathCalib = "../calibration/calib.yaml";
    std::string m_pathCalibLaser = "../calibration/laserCalib.yaml";
    std::array<cv::Mat, 3> m_K;
    std::array<cv::Mat, 3> m_D;
    std::array<cv::Mat, 3> m_P;
    std::array<cv::Mat, 3> m_R;
    std::array<cv::Mat, 3> m_T;
    std::array<cv::Mat, 3> m_F;
    std::array<cv::Mat, 3> m_Rect;
    std::array<cv::Mat, 3> m_Q;

    // Event recordings
    const std::string m_eventRecordFile = "../calibration/laserCalibPoints.txt";
    std::ofstream m_recorder;

    // List of incoming filtered events for each camera (FIFO)
    std::array<std::list<DAVIS240CEvent>,2> m_evtQueue;

    // Mutex to access the queue
    std::array<std::mutex,2> m_queueAccessMutex;
    std::mutex m_mutex;

    // Wait when no processing has to be done
    std::condition_variable m_condWait;
    std::mutex m_condWaitMutex;

    // List of triangulator listeners
    std::list<TriangulatorListener*> m_triangulatorListeners;

    // Thread this object runs in.
    std::thread m_thread;
};

#endif // EBV_TRIANGULATOR_H


