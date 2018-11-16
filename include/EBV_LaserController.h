#ifndef EBV_LASERCONTROLLER_H
#define EBV_LASERCONTROLLER_H

#include <cmath>
#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>

static const char* modesList[] = {"circle", "swipe"};

class MagneticMirrorLaser;

class LaserController
{
public:
    LaserController();
    ~LaserController();

    void start();
    void draw();
    void stop();

    // Getters and setters
    int getX() const { return m_x; }
    void setX(const int x) { m_x= x; }

    int getY() const { return m_y; }
    void setY(const int y) { m_y= y; }

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

    bool getCalibrationMode() const { return m_calibrateLaser; }
    void setCalibrationMode(bool mode);

public:
    std::thread m_thread;
    std::string m_mode;
    std::mutex m_mutex;

    int m_x;
    int m_y;
    bool m_calibrateLaser;

private:
    MagneticMirrorLaser* m_laser;
    int m_min_x;
    int m_min_y;
    int m_max_x;
    int m_max_y;
    int m_freq;
    double m_step;
    int m_cx;
    int m_cy;
    int m_r;
    int m_vx;
    int m_vy;

};

#endif // EBV_LASERCONTROLLER_H
