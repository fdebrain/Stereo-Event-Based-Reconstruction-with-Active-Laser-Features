#include <EBV_Triangulator.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

Triangulator::Triangulator(int rows, int cols,
                           Matcher*  matcher):
    m_rows(rows), m_cols(cols)
{
    this->setMatcher(matcher);
}

Triangulator::~Triangulator() {}

void Triangulator::registerTriangulatorListener(TriangulatorListener* listener)
{
    m_triangulatorListeners.push_back(listener);
}

void Triangulator::deregisterTriangulatorListener(TriangulatorListener* listener)
{
    m_triangulatorListeners.remove(listener);
}

void Triangulator::warnDepth()
{
    std::list<TriangulatorListener*>::iterator it;
    for(it = m_triangulatorListeners.begin(); it!=m_triangulatorListeners.end(); it++)
    {
        (*it)->receivedNewDepth();
    }
}
