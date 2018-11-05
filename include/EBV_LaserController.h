#ifndef EBV_LASERCONTROLLER_H
#define EBV_LASERCONTROLLER_H

#include <thread>
#include <mutex>

class MagneticMirrorLaser;

class LaserController
{
public:
    LaserController();
    ~LaserController();

    void start();
    void draw();
    void stop();

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

private:
    MagneticMirrorLaser* m_laser;

    int m_freq;
    double m_step;
    int m_cx;
    int m_cy;
    int m_r;
};

#endif // EBV_LASERCONTROLLER_H
