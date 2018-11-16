#include <EBV_Matcher.h>

Matcher::Matcher(Filter* filter0,
                 Filter* filter1)
    : m_rows(180), m_cols(240),
      m_filter{filter0,filter1}
{
    // Matching parameters
    m_eps = 1e4;
    m_maxTimeToKeep = 1e4; // Keep 10ms buffer in each filtered events queue

    // Initialize filters listeners
    m_filter[0]->registerFilterListener(this);
    m_filter[1]->registerFilterListener(this);

    // Initialize thread
    m_thread = std::thread(&Matcher::runThread,this);
}

Matcher::~Matcher()
{
    m_filter[0]->deregisterFilterListener(this);
    m_filter[1]->deregisterFilterListener(this);
}

// The function the thread executes and waits on
void Matcher::runThread()
{
    std::array<bool,2> hasQueueEvent;
    DAVIS240CEvent event0;
    DAVIS240CEvent event1;

    while(true)
    {
        m_queueAccessMutex[0].lock();
            hasQueueEvent[0] = !m_evtQueue[0].empty();
        m_queueAccessMutex[0].unlock();

        m_queueAccessMutex[1].lock();
            hasQueueEvent[1] = !m_evtQueue[1].empty();
        m_queueAccessMutex[1].unlock();

        if(hasQueueEvent[0] && hasQueueEvent[1])
        {
            m_queueAccessMutex[0].lock();
                int s0 = m_evtQueue[0].size();
                event0 = m_evtQueue[0].front();
                m_evtQueue[0].pop_front();
            m_queueAccessMutex[0].unlock();

            m_queueAccessMutex[1].lock();
                int s1 = m_evtQueue[1].size();
                event1 = m_evtQueue[1].front();
                m_evtQueue[1].pop_front();
            m_queueAccessMutex[1].unlock();

            // DEBUG - QUEUE SIZE SHOULD BE SIMILAR
            //printf("Queue size: %d - %d.\n\r", s[0], s[1]);
            // END DEBUG

            // DEBUG - DELTA TIME SHOULD BE SMALL
            //int t0 = event0.m_timestamp;
            //int t1 = event1.m_timestamp;
            //printf("Delta-time: (%d).\n\r",t0-t1);
            // END DEBUG

            process(event0,event1);
        }
        else
        {
            std::unique_lock<std::mutex> condLock(m_condWaitMutex);
            m_condWait.wait(condLock);
            condLock.unlock();
        }
    }
}

// Called by the thread of previous agent (here Filter)
void Matcher::receivedNewFilterEvent(DAVIS240CEvent &event,
                                     const uint id)
{
    // DEBUG - CHECK IF EVENTS SYNCHRONIZED
    //printf("Camera %d - Timestamp %d. \n\r",id,event.m_timestamp);
    // END DEBUG

    // Remove old events
    m_queueAccessMutex[id].lock();
        m_lastEvent_t[id] = event.m_timestamp;
        std::list<DAVIS240CEvent>::iterator it = m_evtQueue[id].begin();
        while(    ((m_lastEvent_t[id]-it->m_timestamp) > m_maxTimeToKeep)
               && (it!=m_evtQueue[id].end())
             )
        {
            m_evtQueue[id].erase(it++);
        }
        m_evtQueue[id].push_back(event);
    m_queueAccessMutex[id].unlock();

    // Notify thread
    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

void Matcher::process(DAVIS240CEvent& e0, DAVIS240CEvent& e1)
{
    const int t0 = e0.m_timestamp;
    const int t1 = e1.m_timestamp;

    // Match if similar timestamps
    if (std::abs(t0 - t1)<m_eps)
    {
        warnMatch(e0,e1);
    }
}

void Matcher::registerMatcherListener(MatcherListener* listener)
{
    m_matcherListeners.push_back(listener);
}

void Matcher::deregisterMatcherListener(MatcherListener* listener)
{
    m_matcherListeners.remove(listener);
}

void Matcher::warnMatch(const DAVIS240CEvent& event0,
                        const DAVIS240CEvent& event1)
{
    for(auto it = m_matcherListeners.begin(); it!=m_matcherListeners.end(); it++)
    {
        (*it)->receivedNewMatch(event0,event1);
    }
}
