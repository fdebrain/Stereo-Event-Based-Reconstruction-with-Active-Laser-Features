#include <EBV_LaserController.h>
#include <EBV_MagneticMirrorLaser.h>

LaserController::LaserController(Triangulator* triangulator)
    : m_laser(new MagneticMirrorLaser()),
      m_triangulator(triangulator),
      m_freq(500), // Fix the offset
      m_step(1/100.),
      m_cx(2048),
      m_cy(2048),
      m_r(1000),
      m_vx(1.5e4),
      m_vy(4e5),
      m_mode(modesList[1]),
      m_max_x(4000),
      m_max_y(4000),
      m_calibrateLaser(true),
      m_x(0),
      m_y(0)
{ 
    // Initialize laser
    m_laser->init("/dev/ttyUSB0");
    m_laser->toggle(0);

    // Calibration mode
    if (m_calibrateLaser)
    {
        m_triangulator->registerTriangulatorListener(this);
        m_recorder.open(m_eventRecordFile);
    }
}

LaserController::~LaserController()
{
    if (m_calibrateLaser)
    {
        m_triangulator->deregisterTriangulatorListener(this);
        m_recorder.close();
    }

    m_laser->close();
    delete m_laser;
}

void LaserController::receivedNewDepth(const unsigned int &u,
                                  const unsigned int &v,
                                  const double &X,
                                  const double &Y,
                                  const double &Z)
{
    // Record event position/timestamp in each camera + laser position/timestamp
    if (m_calibrateLaser)
    {
        m_recorder << u << '\t'
                   << v << '\t'
                   << m_x << '\t'
                   << m_y << '\t'
                   << X << '\t'
                   << Y << '\t'
                   << Z << '\n';
    }
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
    /*
    double t = 0.0;
    unsigned int x=0, y=0;

    while(true)
    {
        if (m_mode=="circle")
        {
            t += m_step;
            x = static_cast<unsigned int>(m_cx + m_r*cos(2.*3.14*t));
            y = static_cast<unsigned int>(m_cy + m_r*sin(2.*3.14*t));
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

    if(m_calibrateLaser)
    {
        while(true)
        {

            m_x = m_max_x/4;
            m_y = m_max_y/4;
            m_calibrateLaser = true;
            m_laser->pos(m_x,m_y);
            std::this_thread::sleep_for (std::chrono::milliseconds(5));
            m_calibrateLaser = false;
            std::this_thread::sleep_for (std::chrono::milliseconds(500));

            m_x = m_max_x/4;
            m_y = 3*m_max_y/4;
            m_calibrateLaser = true;
            m_laser->pos(m_x,m_y);
            std::this_thread::sleep_for (std::chrono::milliseconds(5));
            m_calibrateLaser = false;
            std::this_thread::sleep_for (std::chrono::milliseconds(500));

            m_x = 3*m_max_x/4;
            m_y = 3*m_max_y/4;
            m_calibrateLaser = true;
            m_laser->pos(m_x,m_y);
            std::this_thread::sleep_for (std::chrono::milliseconds(5));
            m_calibrateLaser = false;
            std::this_thread::sleep_for (std::chrono::milliseconds(500));

            m_x = 3*m_max_x/4;
            m_y = m_max_y/4;
            m_calibrateLaser = true;
            m_laser->pos(m_x,m_y);
            std::this_thread::sleep_for (std::chrono::milliseconds(5));
            m_calibrateLaser = false;
            std::this_thread::sleep_for (std::chrono::milliseconds(500));

            m_x = m_max_x/2;
            m_y = m_max_y/2;
            m_calibrateLaser = true;
            m_laser->pos(m_x,m_y);
            std::this_thread::sleep_for (std::chrono::milliseconds(5));
            m_calibrateLaser = false;
            std::this_thread::sleep_for (std::chrono::milliseconds(500));
        }
    }
    else
    {
        // Method 2: Velocity control
        m_laser->vel(m_vx,m_vy);
    }
}
