#include <EBV_LaserController.h>
#include <EBV_MagneticMirrorLaser.h>

LaserController::LaserController()//Triangulator* triangulator)
    : m_laser{new MagneticMirrorLaser},
      m_freq(500), // Fix the offset
      m_step(50),
      m_vx(1.5e4),
      m_vy(4e5),
      m_min_x(1000),
      m_min_y(500),
      m_max_x(3000),
      m_max_y(3000),
      m_learningRate(0.1),
      m_calibrateLaser(false),
      m_laser_on(false)
{ 
    // Initialize laser
    m_laser->init("/dev/ttyUSB0");
    m_x = m_min_x;
    m_y = m_min_y;
    m_laser->pos(m_x,m_y);
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

void LaserController::start()
{
    m_laser_on = true;
    m_laser->toggle(1);
    m_laser->blink(static_cast<uint>(1e6/static_cast<double>(2*m_freq)));
    this->draw();
    //m_thread = std::thread(&LaserController::draw,this);
}

void LaserController::stop()
{
    m_laser_on = false;
    m_laser->toggle(0);
    m_laser->vel(0,0);
}

void LaserController::setPos(const int x, const int y)
{
    m_x = x;
    m_y = y;
    m_laser->pos(x,y);
}

void LaserController::draw()
{
    /*
    double t = 0.0;
    uint x=0, y=0;

    while(true)
    {
        if (m_mode=="circle")
        {
            t += m_step;
            x = static_cast<uint>(m_cx + m_r*cos(2.*3.14*t));
            y = static_cast<uint>(m_cy + m_r*sin(2.*3.14*t));
            m_laser->pos(x,y);
            if (t>1.0) { t = 0.0; }

            std::this_thread::sleep_for (std::chrono::milliseconds(1));
        }
        else
        {
            y += 10;
            m_laser->pos(x,y);
            if (y>=m_max_y) { y = 0; x += 100; }
            if (x>m_max_x) { x = 0; }
            std::this_thread::sleep_for (std::chrono::microseconds(500));
        }
    }
    */
    /*
    while(true)
    {
        while(m_y<m_max_y)
        {
            m_y += m_step;
            m_laser->pos(m_x,m_y);
            std::this_thread::sleep_for (std::chrono::milliseconds(10));
        }

        while(m_x<m_max_x)
        {
            m_x += m_step;
            m_laser->pos(m_x,m_y);
            std::this_thread::sleep_for (std::chrono::milliseconds(10));
        }

        while(m_y>m_min_y)
        {
            m_y -= m_step;
            m_laser->pos(m_x,m_y);
            std::this_thread::sleep_for (std::chrono::milliseconds(10));
        }

        while(m_x>m_min_x)
        {
            m_x -= m_step;
            m_laser->pos(m_x,m_y);
            std::this_thread::sleep_for (std::chrono::milliseconds(10));
        }
    }
    */
    m_laser->vel(m_vx,m_vy);
}
