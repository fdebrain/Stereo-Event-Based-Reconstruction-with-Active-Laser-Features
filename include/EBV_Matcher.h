#ifndef EBV_MATCHER_H
#define EBV_MATCHER_H

#include <EBV_Filter.h>
#include <EBV_LaserController.h>

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
//    virtual void receivedNewMatch(const DAVIS240CEvent& e1,
//                                  const LaserEvent& e2) = 0;
};


class Matcher: public FilterListener
               //public LaserListener
{
public:
    Matcher(Filter* filter0 = nullptr,
            Filter* filter1 = nullptr,
            LaserController* laser = nullptr);
    ~Matcher();

    // Getters and setters
    int getEps() const { return m_eps; }
    void setEps(const int eps) { m_eps = eps; }

    int getMaxBuffer() const { return m_maxTimeToKeep; }
    void setMaxBuffer(const int maxBuffer) { m_maxTimeToKeep = maxBuffer; }

    void receivedNewFilterEvent(DAVIS240CEvent& event,
                                const int id) override;
//    void receivedNewLaserEvent(const LaserEvent& event) override;
    void runThread();
    void process(DAVIS240CEvent& e0, DAVIS240CEvent& e1);
    void process(DAVIS240CEvent& e0, LaserEvent& e1);

    void registerMatcherListener(MatcherListener* listener);
    void deregisterMatcherListener(MatcherListener* listener);
    void warnMatch(const DAVIS240CEvent& event0,
                   const DAVIS240CEvent& event1);
//    void warnMatch(const DAVIS240CEvent& event0,
//                   const LaserEvent& event1);

public:
    // Thread this object runs in
    std::thread m_thread;

private:
    const int m_rows{180};
    const int m_cols{240};
    int m_eps{10000};

    // Flushing old events
    int m_currTime0;
    int m_currTime1;
    int m_currTime2;
    int m_maxTimeToKeep{10000};

    // List of incoming filtered events for each camera (FIFO)
    std::list<DAVIS240CEvent> m_evtQueue0;
    std::list<DAVIS240CEvent> m_evtQueue1;
    //std::list<LaserEvent> m_evtQueue2;

    // Mutex to access the queue
    std::mutex m_queueAccessMutex0;
    std::mutex m_queueAccessMutex1;
    //std::mutex m_queueAccessMutex2;

    // Wait when no processing has to be done
    std::condition_variable m_condWait;
    std::mutex m_condWaitMutex;

    // Who matcher listens to
    Filter* m_filter0;
    Filter* m_filter1;
    LaserController* m_laser;

    // List of matcher listeners
    std::list<MatcherListener*> m_matcherListeners;
};

#endif // EBV_MATCHER_H
