#ifndef EBV_LASERCONTROLLER_H
#define EBV_LASERCONTROLLER_H

#include <cmath>
#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>

class MagneticMirrorLaser;

class LaserController
{
public:
    LaserController();
    ~LaserController();

    void start();
    void startSwipe();
    void draw();
    void stop();

    // Getters and setters
    int getX() const { return m_x; }
    void setX(const int x) { m_x = x; this->setPos(m_x,m_y);}

    int getY() const { return m_y; }
    void setY(const int y) { m_y = y; this->setPos(m_x,m_y);}

    int getVx() const { return m_vx; }
    void setVx(const int vx) { m_vx = vx; this->setVel(m_vx,m_vy);}

    int getVy() const { return m_vy; }
    void setVy(const int vy) { m_vy = vy; this->setVel(m_vx,m_vy);}

    int getFreq() const { return m_freq; }
    void setFreq(const int freq);

    bool getCalibrationMode() const { return m_calibrateLaser; }
    void setCalibrationMode(bool mode);

    void setPos(const int x, const int y);
    void setVel(const int vx, const int vy);

public:
    std::thread m_thread;
    std::string m_mode;
    std::mutex m_mutex;

    int m_x;
    int m_y;
    bool m_calibrateLaser;
    bool m_laser_on;
    bool m_laser_swipe;

private:
    MagneticMirrorLaser* m_laser;
    int m_min_x;
    int m_min_y;
    int m_max_x;
    int m_max_y;
    int m_freq;
    double m_step;
    int m_vx;
    int m_vy;
};

#endif // EBV_LASERCONTROLLER_H
