#include <EBV_LaserController.h>
#include <EBV_MagneticMirrorLaser.h>

LaserController::LaserController(int freq)
    : m_laser{new MagneticMirrorLaser},
      m_freq(freq) // Fix the frequency offset
{ 
    // Connect laser
    m_laser->init("/dev/ttyUSB0");
    m_x = m_max_x/2;
    m_y = m_max_y/2;
    m_laser->pos(m_x,m_y);
    m_t_start = std::chrono::system_clock::now();

    // Initialize thread
    m_thread = std::thread(&LaserController::runThread,this);
}

LaserController::~LaserController()
{
    printf("Call laser destructor ! \n\r");
    m_laser->~MagneticMirrorLaser();
    delete m_laser;
}

// SETTERS
void LaserController::setFreq(const int freq)
{
    m_freq = freq;
    std::chrono::microseconds dt{static_cast<int>(1e6f/static_cast<float>(2*m_freq))};
    m_laser->blink(dt);
}

void LaserController::setPos(const int x, const int y)
{
    auto now = std::chrono::system_clock::now();
    m_t = std::chrono::duration_cast<std::chrono::microseconds>(now - m_t_start).count();
    m_x = x;
    m_y = y;
    //printf("Laser t: %ld. \n\r",m_t);

    // Boundaries check
    if (m_x<m_min_x) { m_x = m_min_x; }
    if (m_x>m_max_x) { m_x = m_max_x; }
    if (m_y<m_min_y) { m_y = m_min_y; }
    if (m_y>m_max_y) { m_y = m_max_y; }

    // Move to target
    m_laser->pos(m_x,m_y);

    // Wait for laser to reach target
    std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep));
}

void LaserController::setVel(const int vx, const int vy)
{
    m_vx = vx;
    m_vy = vy;
    m_laser->vel(m_vx,m_vy);
}

void LaserController::init()
{
    m_t_start = std::chrono::system_clock::now();
}

void LaserController::start()
{
    printf("Start laser. \n\r");
    m_laser_on = true;
    m_laser->toggle(1);
    this->setFreq(m_freq);
}

void LaserController::stop()
{
    printf("Stop laser. \n\r");
    m_laser_on = false;
    m_swipe_on = false;
    m_laser->toggle(0);
    m_laser->vel(0,0);
}

void LaserController::runThread()
{
    while(true)
    {
        if (m_swipe_on) { this->sweep(); }
        else
        {
            // IDLE if not received new command
            std::unique_lock<std::mutex> condLock(m_condWaitMutex);
            m_condWait.wait(condLock);
            condLock.unlock();
        }
    }
}

void LaserController::toogleState()
{
    m_received_new_state = true;
    m_laser_on = !m_laser_on;

    if (m_laser_on)
    {
        printf("Laser on. \n\r");
        this->start();
    }
    else
    {
        printf("Laser off. \n\r");
        this->stop();
    }

    // Notify new command to the thread
    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

void LaserController::toogleSweep()
{
    m_received_new_state = true;
    m_swipe_on = !m_swipe_on;

    if (m_swipe_on)
    {
        printf("Laser sweep mode enabled. \n\r");
        this->start();
    }
    else
    {
        printf("Laser sweep mode disabled. \n\r");
        this->stop();
    }

    // Notify new command to the thread
    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

void LaserController::sweep()
{
    // Sweep mode 1
    m_y += m_step;
    if (m_y>=m_max_y) { m_y = m_min_y; m_x += m_ratio*m_step; }
    if (m_x>m_max_x) { m_x = m_min_x; }
    this->setPos(m_x,m_y);

    // Sweep mode 2
//    m_y += m_direction_y*m_step;
//    if (m_y>m_max_y) { m_direction_y = -1; m_x += m_direction_x*m_ratio*m_step; }
//    else if (m_y<m_min_y) { m_direction_y = 1; m_x += m_direction_x*m_ratio*m_step; }
//    if (m_x>m_max_x) { m_direction_x = -1; }
//    else if (m_x<m_min_x) { m_direction_x = 1; }
//    this->setPos(m_x,m_y);
}
