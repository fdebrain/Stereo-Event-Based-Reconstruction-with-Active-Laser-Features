#include <EBV_LaserController.h>
#include <EBV_MagneticMirrorLaser.h>
#include <cmath>
#include <chrono>

LaserController::LaserController()
    : m_laser(new MagneticMirrorLaser()),
      m_freq(600),
      m_step(1/100.),
      m_cx(2048),
      m_cy(2048),
      m_r(1000)
{ 
    m_laser->init("/dev/ttyUSB0");
    m_laser->toggle(0);
}

LaserController::~LaserController()
{
    m_laser->close();
    delete m_laser;
}

void LaserController::setFreq(const int freq)
{
    m_freq = freq;
    m_laser->blink(static_cast<unsigned int>(1e6/static_cast<double>(2*m_freq)));
}

void LaserController::start()
{
    m_laser->blink(static_cast<unsigned int>(1e6/static_cast<double>(2*m_freq)));
    m_laser->pos(m_cx,m_cy);
    m_thread = std::thread(&LaserController::draw,this);
}

void LaserController::stop()
{
    m_laser->toggle(0);
}

void LaserController::draw()
{
    double t = 0.0;
    unsigned int x, y;

    while(true)
    {
        t += m_step;
        x = static_cast<unsigned int>(m_cx + m_r*cos(2.*3.14*t));
        y = static_cast<unsigned int>(m_cy + m_r*sin(2.*3.14*t));
        m_laser->pos(x,y);
        if (t>1.0) { t = 0.0; }

        std::this_thread::sleep_for (std::chrono::milliseconds(1));
    }
}
