#ifndef EBV_TRIANGULATOR_H
#define EBV_TRIANGULATOR_H

#include <EBV_DAVIS240C.h>
#include <EBV_Matcher.h>
#include <EBV_LaserController.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

class StereoRectificationData {
  public:
    std::array<cv::Mat_<double>, 3> R;
    std::array<cv::Mat_<double>, 3> P;
    cv::Mat_<double> Q;
    /**
     * For each camera, contains the map for x and y coordinates
     */
    std::array<std::pair<cv::Mat_<float>, cv::Mat_<float>>, 2> maps;

    StereoRectificationData();
    bool is_valid() const;
};

class TriangulatorListener
{
public:
    TriangulatorListener(void) {}
    virtual void receivedNewDepth(const unsigned int &u,
                                  const unsigned int &v,
                                  const double &X,
                                  const double &Y,
                                  const double &Z) = 0;
};

class Triangulator : public DAVIS240CFrameListener,
                     public MatcherListener
{
public:
    Triangulator(const unsigned int rows,
                 const unsigned int cols,
                 DAVIS240C* davis0 = nullptr,
                 DAVIS240C* davis1 = nullptr,
                 Matcher* matcher = nullptr,
                 LaserController* laser = nullptr);
    ~Triangulator();

    void importCalibration();
    void run();
    void receivedNewDAVIS240CFrame(DAVIS240CFrame& f,
                                   const unsigned int id) override;
    void receivedNewMatch(const DAVIS240CEvent& event1,
                          const DAVIS240CEvent& event2) override;
    void process(const DAVIS240CEvent& event0,
                 const DAVIS240CEvent& event1);

    void registerTriangulatorListener(TriangulatorListener* listener);
    void deregisterTriangulatorListener(TriangulatorListener* listener);
    void warnDepth(const unsigned int u,
                   const unsigned int v,
                   const double X,
                   const double Y,
                   const double Z);

    // Who triangulator is listening to
    Matcher* m_matcher;
    LaserController* m_laser;
    DAVIS240C* m_davis0;
    DAVIS240C* m_davis1;

private:
    const unsigned int m_rows;
    const unsigned int m_cols;

    // Camera calibration
    std::string m_pathCalib = "../calibration/calib.yaml";
    std::string m_pathCalibLaser = "../calibration/laserCalib.yaml";
    cv::Mat m_K0 = cv::Mat(3,3,CV_32FC1);
    cv::Mat m_D0 = cv::Mat(1,5,CV_32FC1);
    cv::Mat m_P0 = cv::Mat(3,4,CV_32FC1);

    cv::Mat m_K1 = cv::Mat(3,3,CV_32FC1);
    cv::Mat m_D1 = cv::Mat(1,5,CV_32FC1);
    cv::Mat m_P1 = cv::Mat(3,4,CV_32FC1);

    cv::Mat m_R = cv::Mat(3,3,CV_32FC1);
    cv::Mat m_T = cv::Mat(1,3,CV_32FC1);
    cv::Mat m_F = cv::Mat(3,3,CV_32FC1);

    cv::Mat m_KLaser = cv::Mat(3,3,CV_32FC1);
    cv::Mat m_DLaser = cv::Mat(1,5,CV_32FC1);
    cv::Mat m_PLaser = cv::Mat(3,4,CV_32FC1);

    cv::Mat m_R0 = cv::Mat(3,3,CV_32FC1);
    cv::Mat m_T0 = cv::Mat(1,3,CV_32FC1);
    cv::Mat m_F0 = cv::Mat(3,3,CV_32FC1);

    cv::Mat m_R1 = cv::Mat(3,3,CV_32FC1);
    cv::Mat m_T1 = cv::Mat(1,3,CV_32FC1);
    cv::Mat m_F1 = cv::Mat(3,3,CV_32FC1);


    // Laser calibration
    //bool m_calibrateLaser; Error when using getter of laser
    const std::string m_eventRecordFile = "../calibration/laserCalibPoints.txt";
    std::ofstream m_recorder;

    // Camera calibration
    bool m_calibrateCamera;
    const std::string m_frameRecordFile0 = "../calibration/data/calib0.avi";
    const std::string m_frameRecordFile1 = "../calibration/data/calib1.avi";
    StereoRectificationData m_rect{};
    cv::VideoWriter m_video0;
    cv::VideoWriter m_video1;

    // List of incoming filtered events for each camera (FIFO)
    std::list<DAVIS240CEvent> m_evtQueue0;
    std::list<DAVIS240CEvent> m_evtQueue1;

    // Mutex to access the queue
    std::mutex m_queueAccessMutex0;
    std::mutex m_queueAccessMutex1;
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


