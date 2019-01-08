#include <EBV_Benchmarking.h>

LaserTest::LaserTest(LaserController* laser) : m_laser(laser)
{
}

LaserTest::~LaserTest()
{
}

void LaserTest::computeAngularVelocity()
{
    std::cout << "Reset laser position. \n\r";
    m_laser->setPos(1600,900);

    int count = 0;

    Timer timer;
    while(m_laser->getY()<3620 && count<1000)
    {
        m_laser->sweep();
        count++;
    }

    std::cout << "Finished sweep. \n\r";
}

