#ifndef EBV_LASERCONTROLLER_H
#define EBV_LASERCONTROLLER_H

#include <list>
#include <cmath>
#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

class MagneticMirrorLaser;

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

    int const& getX() const { return m_x; }
    void setX(const int x) { m_x = x; this->setPos(m_x,m_y);}

    int const& getY() const { return m_y; }
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

    int getSleepTime() const { return m_sleep; }
    void setSleepTime(const int sleep_time);

    void setPos(const int x, const int y);
    void setVel(const int vx, const int vy);

    // Life cycle
    void init();
    void start();
    void stop();
    void runThread();
    void toogleState();
    void toogleSweep();
    void sweep();

public:
    // Laser state
    int m_x;
    int m_y;
    long m_t;
    bool m_laser_on{false};
    bool m_swipe_on{false};
    bool m_received_new_state{false};

    // Laser position boundaries
    int m_min_x{950}; // 500
    int m_min_y{400};   // 0
    int m_max_x{3300}; // 4000
    int m_max_y{3300}; // 4000

    std::chrono::high_resolution_clock::time_point m_t_start;
    std::thread m_thread;

private:
    MagneticMirrorLaser* m_laser;
    int m_freq{0};
    int m_step{50};
    float m_ratio{2};
    int m_vx{0};
    int m_vy{0};
    int m_direction_x{1};
    int m_direction_y{1};
    int m_sleep{2};

    // Wait when no processing has to be done
    std::condition_variable m_condWait;
    std::mutex m_condWaitMutex;
};

#endif // EBV_LASERCONTROLLER_H
