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
    void receivedNewMatch(DAVIS240CEvent& event1, DAVIS240CEvent& event2);
    void run();
    void process();

    void registerTriangulatorListener(TriangulatorListener* listener);
    void deregisterTriangulatorListener(TriangulatorListener* listener);
    void warnDepth();

private:
    int m_rows;
    int m_cols;

    // Thread this object runs in
    std::thread m_thread;

    // List of incoming filtered events for each camera (FIFO)
    std::list<DAVIS240CEvent> m_evtQueue0;
    std::list<DAVIS240CEvent> m_evtQueue1;

    // Mutex to access the queue
    std::mutex m_queueAccessMutex;

    // Wait when no processing has to be done
    std::condition_variable m_condWait;
    std::mutex m_condWaitMutex;

    // Who triangulator is listening to
    Matcher* m_matcher;

    // List of triangulator listeners
    std::list<TriangulatorListener*> m_triangulatorListeners;
};

#endif // EBV_TRIANGULATOR_H


