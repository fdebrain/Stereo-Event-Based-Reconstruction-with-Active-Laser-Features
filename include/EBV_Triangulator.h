#ifndef EBV_TRIANGULATOR_H
#define EBV_TRIANGULATOR_H

#include <EBV_Matcher.h>

class TriangulatorListener
{
public:
    TriangulatorListener(void) {}
    virtual void receivedNewDepth() = 0;
};

class Triangulator : public MatcherListener
{
public:
    Triangulator(int rows, int cols, Matcher* matcher = nullptr);
    ~Triangulator();

    void setMatcher(Matcher* matcher){ m_matcher=matcher; }

    void receivedNewMatch(DAVIS240CEvent& event1, DAVIS240CEvent& event2){}

    void registerTriangulatorListener(TriangulatorListener* listener);
    void deregisterTriangulatorListener(TriangulatorListener* listener);
    void warnDepth();

private:
    int m_rows;
    int m_cols;
    Matcher* m_matcher;
    std::list<TriangulatorListener*> m_triangulatorListeners;
};


#endif // EBV_TRIANGULATOR_H


