#ifndef EBV_MATCHER_H
#define EBV_MATCHER_H

#include <EBV_Filter.h>

#include <list>
#include <thread>
#include <mutex>
#include <condition_variable>

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
    Matcher(Filter* filter0 = nullptr,
            Filter* filter1 = nullptr);
    ~Matcher();

    void receivedNewFilterEvent(DAVIS240CEvent& event,
                                const uint id) override;
    void runThread();
    void process(DAVIS240CEvent& e0, DAVIS240CEvent& e1);

    void registerMatcherListener(MatcherListener* listener);
    void deregisterMatcherListener(MatcherListener* listener);
    void warnMatch(const DAVIS240CEvent& event0,
                   const DAVIS240CEvent& event1);

    int getEps() const { return m_eps; }
    void setEps(const int eps) { m_eps = eps; }

    int getMaxBuffer() const { return m_maxTimeToKeep; }
    void setMaxBuffer(const int maxBuffer) { m_maxTimeToKeep = maxBuffer; }

public:
    // Thread this object runs in
    std::thread m_thread;

private:
    const int m_rows;
    const int m_cols;
    int m_eps;

    // Flushing old events
    std::array<int,2> m_lastEvent_t;
    int m_maxTimeToKeep;

    // List of incoming filtered events for each camera (FIFO)
    std::array<std::list<DAVIS240CEvent>,2> m_evtQueue;

    // Mutex to access the queue
    std::array<std::mutex,2> m_queueAccessMutex;

    // Wait when no processing has to be done
    std::condition_variable m_condWait;
    std::mutex m_condWaitMutex;

    // Who matcher listens to
    std::array<Filter*,2> m_filter;

    // List of matcher listeners
    std::list<MatcherListener*> m_matcherListeners;
};

#endif // EBV_MATCHER_H
