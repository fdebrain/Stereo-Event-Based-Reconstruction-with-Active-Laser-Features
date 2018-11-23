#include <EBV_LaserController.h>
#include <EBV_MagneticMirrorLaser.h>

LaserController::LaserController(int freq)
    : m_laser{new MagneticMirrorLaser},
      m_freq(freq), // Fix the offset
      m_step(100),
      m_min_x(500),
      m_min_y(0),
      m_max_x(3500),
      m_max_y(4000),
      m_calibrateLaser(false),
      m_laser_on(false),
      m_swipe_on(false),
      m_received_new_state(false),
      m_vx(0),
      m_vy(0),
      m_swipe_vx(15e3), //(15e3),
      m_swipe_vy(200e3) //(400ee),
{ 
    // Initialize laser
    m_laser->init("/dev/ttyUSB0");
    m_x = m_max_x/2;
    m_y = m_max_y/2;
    m_laser->pos(m_x,m_y);

    // Initialize thread
    m_thread = std::thread(&LaserController::runThread,this);
}

LaserController::~LaserController()
{
    printf("Call laser destructor ! \n\r");
    m_laser->~MagneticMirrorLaser();
    delete m_laser;
}

void LaserController::setFreq(const int freq)
{
    m_freq = freq;
    m_laser->blink(static_cast<uint>(1e6/static_cast<double>(2*m_freq)));
}

void LaserController::setPos(const int x, const int y)
{
    m_x = x;
    m_y = y;

    // Boundaries check
    if (m_x<m_min_x) { m_x = m_min_x; }
    if (m_x>m_max_x) { m_x = m_max_x; }
    if (m_y<m_min_y) { m_y = m_min_y; }
    if (m_y>m_max_y) { m_y = m_max_y; }

    // Move to target
    m_laser->pos(m_x,m_y);

    // Wait for laser to reach target
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void LaserController::setVel(const int vx, const int vy)
{
    m_vx = vx;
    m_vy = vy;
    m_laser->vel(m_vx,m_vy);
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

void LaserController::toogleState()
{
    m_received_new_state = true;
    m_laser_on = !m_laser_on;

    if (m_laser_on) { this->start(); }
    else { this->stop(); }

    // Notify new command to the thread
    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

void LaserController::toogleSwipe()
{
    m_received_new_state = true;
    m_swipe_on = !m_swipe_on;

    if (m_swipe_on)
    {
        printf("Swipe mode laser. \n\r");
        this->start();
    }
    else { this->stop(); }

    // Notify new command to the thread
    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

void LaserController::runThread()
{
    while(true)
    {
        if (m_swipe_on) { this->swipe(); }
        else
        {
            // IDLE if not received new command
            std::unique_lock<std::mutex> condLock(m_condWaitMutex);
            m_condWait.wait(condLock);
            condLock.unlock();
        }
    }
}

void LaserController::swipe()
{
    const auto now = std::chrono::high_resolution_clock::now();
    m_chrono = now;

    //this->setVel(m_swipe_vx,m_swipe_vy);

    m_y += m_step;
    if (m_y>=m_max_y) { m_y = m_min_y; m_x += 2*m_step; }
    if (m_x>m_max_x) { m_x = m_min_x; }
    this->setPos(m_x,m_y);
    std::this_thread::sleep_for (std::chrono::milliseconds(3));

//    double t = 0.0;
//    uint x=0, y=0;

//    while(true)
//    {
//        if (m_mode=="circle")
//        {
//            t += m_step;
//            x = static_cast<uint>(m_cx + m_r*cos(2.*3.14*t));
//            y = static_cast<uint>(m_cy + m_r*sin(2.*3.14*t));
//            m_laser->pos(x,y);
//            if (t>1.0) { t = 0.0; }

//            std::this_thread::sleep_for (std::chrono::milliseconds(1));
//        }
//        else
//        {
//            y += 10;
//            m_laser->pos(x,y);
//            if (y>=m_max_y) { y = 0; x += 100; }
//            if (x>m_max_x) { x = 0; }
//            std::this_thread::sleep_for (std::chrono::microseconds(500));
//        }
//    }

    // Normal swipe
//    while(true)
//    {
//        m_y += m_step;
//        this->setPos(m_x,m_y);
//        if (m_y>=m_max_y) { m_y = m_min_y; m_x += 2*m_step; }
//        if (m_x>m_max_x) { m_x = m_min_x; }
//        std::this_thread::sleep_for (std::chrono::milliseconds(10));
//    }

    // Draw rectangle
//    while(true)
//    {
//        while(m_y<m_max_y)
//        {
//            m_y += m_step;
//            m_laser->pos(m_x,m_y);
//            std::this_thread::sleep_for (std::chrono::milliseconds(10));
//        }

//        while(m_x<m_max_x)
//        {
//            m_x += m_step;
//            m_laser->pos(m_x,m_y);
//            std::this_thread::sleep_for (std::chrono::milliseconds(10));
//        }

//        while(m_y>m_min_y)
//        {
//            m_y -= m_step;
//            m_laser->pos(m_x,m_y);
//            std::this_thread::sleep_for (std::chrono::milliseconds(10));
//        }

//        while(m_x>m_min_x)
//        {
//            m_x -= m_step;
//            m_laser->pos(m_x,m_y);
//            std::this_thread::sleep_for (std::chrono::milliseconds(10));
//        }
//    }
}

//=== LASER LISTENING ===//
void LaserController::registerLaserListener(LaserListener* listener)
{
    m_laserListeners.push_back(listener);
}

void LaserController::warnCommand(const int x, const int y)
{
    for(auto it = m_laserListeners.begin(); it!=m_laserListeners.end(); it++)
    {
        (*it)->receivedNewCommand(x,y);
    }
}

void LaserController::deregisterLaserListener(LaserListener* listener)
{
    m_laserListeners.remove(listener);
}
