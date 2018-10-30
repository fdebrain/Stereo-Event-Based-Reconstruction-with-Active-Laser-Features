#ifndef EBV_MATCHER_H
#define EBV_MATCHER_H

#include <EBV_Filter.h>

#include <list>
#include <thread>
#include <mutex>
#include <condition_variable>


//class Triangulator;

class MatcherListener
{
public:
    MatcherListener(void) {}
    virtual void receivedNewMatch(const DAVIS240CEvent& e1,
                                  const DAVIS240CEvent& e2) = 0;
};


class Matcher: public FilterListener
{
public:
    Matcher(const unsigned int rows,
            const unsigned int cols,
            Filter* filter0 = nullptr,
            Filter* filter1 = nullptr);
    ~Matcher();

    void receivedNewFilterEvent(DAVIS240CEvent& event,
                                const unsigned int id) override;
    void runThread();
    void process(DAVIS240CEvent& e0, DAVIS240CEvent& e1);

    void registerMatcherListener(MatcherListener* listener);
    void deregisterMatcherListener(MatcherListener* listener);
    void warnMatch(const DAVIS240CEvent& event0,
                   const DAVIS240CEvent& event1);

public:
    // Thread this object runs in
    std::thread m_thread;

private:
    const unsigned int m_rows;
    const unsigned int m_cols;
    int m_eps;

    // Flushing old events
    int m_currTime;
    int m_maxTimeToKeep;

    // List of incoming filtered events for each camera (FIFO)
    std::list<DAVIS240CEvent> m_evtQueue0;
    std::list<DAVIS240CEvent> m_evtQueue1;

    // Mutex to access the queue
    std::mutex m_queueAccessMutex;

    // Wait when no processing has to be done
    std::condition_variable m_condWait;
    std::mutex m_condWaitMutex;


    // Alternative: We access m_events by reference (setFilter here, getEvents in Filter)
    Filter* m_filter0;
    Filter* m_filter1;

    // List of matcher listeners
    std::list<MatcherListener*> m_matcherListeners;
};

#endif // EBV_MATCHER_H
