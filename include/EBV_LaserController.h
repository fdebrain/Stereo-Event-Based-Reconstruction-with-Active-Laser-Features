#ifndef EBV_LASERCONTROLLER_H
#define EBV_LASERCONTROLLER_H

#include <EBV_Triangulator.h>
#include <cmath>
#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>

static const char* modesList[] = {"circle", "swipe"};

class MagneticMirrorLaser;

class LaserController : public TriangulatorListener
{
public:
    LaserController(Triangulator* triangulator);
    ~LaserController();

    void receivedNewDepth(const unsigned int &u,
                          const unsigned int &v,
                          const double &X,
                          const double &Y,
                          const double &Z) override;

    void start();
    void draw();
    void stop();

    // Getters and setters
    int getCenterX() const { return m_cx; }
    void setCenterX(const int cx) { m_cx= cx; }

    int getCenterY() const { return m_cy; }
    void setCenterY(const int cy) { m_cy = cy; }

    int getRadius() const { return m_r; }
    void setRadius(const int r) { m_r= r; }

    double getStep() const { return m_step; }
    void setStep(const double step) { m_step = step; }

    int getFreq() const { return m_freq; }
    void setFreq(const int freq);

public:
    std::thread m_thread;
    std::string m_mode;

    // For laser calibration (WHY NOT MAKE LASER CONTROLLER A LISTENER TO TRIANGULATOR?)
    int m_x;
    int m_y;
    //int m_current;

    bool m_calibrateLaser;
    const std::string m_eventRecordFile = "../calibration/laserCalibPoints.txt";
    std::ofstream m_recorder;

private:
    MagneticMirrorLaser* m_laser;
    Triangulator* m_triangulator;
    unsigned int m_max_x;
    unsigned int m_max_y;
    int m_freq;
    double m_step;
    int m_cx;
    int m_cy;
    int m_r;
    int m_vx;
    int m_vy;

};

#endif // EBV_LASERCONTROLLER_H
