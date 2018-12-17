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

    void switchMode();
    void resetCalibration();
    void importCalibration();
    bool isValid(const int id) const;
    void run();
    void receivedNewMatch(const DAVIS240CEvent& event1,
                          const DAVIS240CEvent& event2) override;
    void process(const DAVIS240CEvent& event0,
                 const DAVIS240CEvent& event1);

    void registerTriangulatorListener(TriangulatorListener* listener);
    void deregisterTriangulatorListener(TriangulatorListener* listener);
    void warnDepth(const int u, const int v,
                   const double X, const double Y, const double Z);
    void recordPoint(int x0, int y0, int x1, int y1, int x2, int y2,
                     float X, float Y, float Z);

    void computeProjectionMatrix(cv::Mat K, cv::Mat R,
                                 cv::Mat T, cv::Mat& P);
public:
    // Who triangulator listens to
    Matcher* m_matcher;
    LaserController* m_laser;

    // Camera settings
    const int m_rows{180};
    const int m_cols{240};

    // Triangulator settings
    bool m_enable{false};
    bool m_debug{false};

    // Stereo mode
    enum StereoPair { Cameras=0, CamLeftLaser=1, CamRightLaser=2 };
    std::string stereoPairNames[3] = {"Cameras", "CamLeftLaser", "CamRightLaser"};
    StereoPair m_mode{Cameras};

    // Recording settings
    bool m_record{false};
    bool m_record_pointwise{false};
    bool m_record_next_point{false};
    std::ofstream m_recorder;
    const std::string m_eventRecordFile = "../experiments/triangulation_scene.txt";

    // Calibration paths
    std::string m_path_calib_cam = "../calibration/calibCameras.yaml";
    std::string m_path_calib_laser = "../calibration/calibLaser.yaml";

    // Calibration datastructures (camera0,camera1,laser)
    std::array<cv::Mat, 3> m_K;
    std::array<cv::Mat, 3> m_D;
    std::array<cv::Mat, 3> m_R;
    std::array<cv::Mat, 3> m_T;
    std::array<cv::Mat, 3> m_F;
    std::array<cv::Mat, 3> m_Q;

    // Two matrices for each stereo pairs (cam0-cam1, cam0-laser, cam1-laser)
    std::array<std::array<cv::Mat,2>, 3> m_P;
    std::array<std::array<cv::Mat,2>, 3> m_Rect;
    cv::Mat m_Pfix;

private:
    // Thread this object runs in (including triangulator listeners)
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

    // List of triangulator listeners
    std::list<TriangulatorListener*> m_triangulatorListeners;
};

#endif // EBV_TRIANGULATOR_H


