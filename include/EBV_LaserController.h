#ifndef EBV_LASERCONTROLLER_H
#define EBV_LASERCONTROLLER_H

#include <EBV_DAVIS240C.h>

#include <list>
#include <cmath>
#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

class MagneticMirrorLaser;

struct LaserEvent : public DAVIS240CEvent
{
  LaserEvent()
      : DAVIS240CEvent (0,0,false,0)
  {}

  LaserEvent(int x, int y, int t)
      : DAVIS240CEvent (x,y,false,t)
  {}
};

class LaserListener
{
public:
    LaserListener(void) {}
    virtual void receivedNewLaserEvent(const LaserEvent& event) = 0;
};

class LaserController
{
public:
    LaserController(int freq);
    ~LaserController();

    // Getters and setters
    int getMinX() const { return m_min_x; }
    void setMinX(const int min_x) { m_min_x = min_x; }

    int getMaxX() const { return m_max_x; }
    void setMaxX(const int max_x) { m_max_x = max_x; }

    int getMinY() const { return m_min_y; }
    void setMinY(const int min_y) { m_min_y = min_y; }

    int getMaxY() const { return m_max_y; }
    void setMaxY(const int max_y) { m_max_y = max_y; }

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

    int getStep() const { return m_step; }
    void setStep(const int step) { m_step = step; }

    float getRatio() const { return m_ratio; }
    void setRatio(const float ratio) { m_ratio= ratio; }


    //bool getCalibrationMode() const { return m_calibrateLaser; }
    //void setCalibrationMode(bool mode);

    void setPos(const int x, const int y);
    void setVel(const int vx, const int vy);

    // Life cycle
    void start();
    void stop();
    void toogleState();
    void toogleSweep();
    void runThread();
    void sweep();

    // Laser event listening
    void registerLaserListener(LaserListener* listener);
    void deregisterLaserListener(LaserListener* listener);
    void warnCommand(const LaserEvent& event);

public:
    std::thread m_thread;
    std::string m_mode;
    std::mutex m_mutex;
    std::chrono::high_resolution_clock::time_point m_chrono{};

    // Laser state
    int m_x;
    int m_y;
    bool m_laser_on;
    bool m_swipe_on;
    bool m_received_new_state;

    int m_min_x;
    int m_min_y;
    int m_max_x;
    int m_max_y;

private:
    MagneticMirrorLaser* m_laser;
    int m_freq;
    int m_step;
    float m_ratio;
    int m_vx;
    int m_vy;
    int m_swipe_vx;
    int m_swipe_vy;

    // Wait when no processing has to be done
    std::condition_variable m_condWait;
    std::mutex m_condWaitMutex;

    std::list<LaserListener*> m_laserListeners;
};

#endif // EBV_LASERCONTROLLER_H
