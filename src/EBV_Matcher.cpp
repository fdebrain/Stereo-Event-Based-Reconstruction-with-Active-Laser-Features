#include <EBV_Matcher.h>

Matcher::Matcher(const unsigned int rows,
                 const unsigned int cols,
                 Filter* filter0,
                 Filter* filter1)
    : m_rows(rows), m_cols(cols),
      m_filter0(filter0), m_filter1(filter1)
{
    // Matching parameters
    m_eps = 2e3;
    m_maxTimeToKeep = 1e4; // Keep 10ms buffer in each filtered events queue

    // Initialize filters listeners
    m_filter0->registerFilterListener(this);
    m_filter1->registerFilterListener(this);

    // Initialize thread
    m_thread = std::thread(&Matcher::runThread,this);
}

Matcher::~Matcher()
{
    m_filter0->deregisterFilterListener(this);
    m_filter1->deregisterFilterListener(this);
}

// The function the thread executes and waits on
void Matcher::runThread()
{
    bool hasQueueEvent0;
    bool hasQueueEvent1;
    DAVIS240CEvent event0;
    DAVIS240CEvent event1;

    while(true)
    {
        m_queueAccessMutex0.lock();
            hasQueueEvent0  = !m_evtQueue0.empty();
        m_queueAccessMutex0.unlock();

        m_queueAccessMutex1.lock();
            hasQueueEvent1  = !m_evtQueue1.empty();
        m_queueAccessMutex1.unlock();

        // Process only if incoming filtered events in both cameras
        //CHANGE
        if(hasQueueEvent0 && hasQueueEvent1)
        {
            m_queueAccessMutex0.lock();
                //int s0 = m_evtQueue0.size();
                event0 = m_evtQueue0.front();
                m_evtQueue0.pop_front();
            m_queueAccessMutex0.unlock();

            m_queueAccessMutex1.lock();
                //int s1 = m_evtQueue1.size();
                event1 = m_evtQueue1.front();
                m_evtQueue1.pop_front();
            m_queueAccessMutex1.unlock();

            // DEBUG - QUEUE SIZE SHOULD BE SIMILAR
            //printf("Queue size: %d - %d.\n\r", s0, s1);
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
                                     const unsigned int id)
{
    // DEBUG - CHECK IF EVENTS SYNCHRONIZED
    //printf("Camera %d - Timestamp %d. \n\r",id,event.m_timestamp);
    // END DEBUG

    switch (id)
    {
        case 0:
        {
            m_queueAccessMutex0.lock();
                m_evtQueue0.push_back(event);

                // Remove old events
                m_currTime0 = event.m_timestamp;
                std::list<DAVIS240CEvent>::iterator it = m_evtQueue0.begin(); // DANGER SHARED RESOURCE
                while(    ((m_currTime0-it->m_timestamp) > m_maxTimeToKeep)
                       && (it!=m_evtQueue0.end())
                     )
                {
                    m_evtQueue0.erase(it++);
                }
            m_queueAccessMutex0.unlock();
            break;
        }

        case 1:
        {
            m_queueAccessMutex1.lock();
                m_evtQueue1.push_back(event);

                // Remove old events
                m_currTime1 = event.m_timestamp;
                std::list<DAVIS240CEvent>::iterator it = m_evtQueue1.begin(); // DANGER SHARED RESOURCE
                while(    ((m_currTime1-it->m_timestamp) > m_maxTimeToKeep)
                       && (it!=m_evtQueue1.end())
                     )
                {
                    m_evtQueue1.erase(it++);
                }
            m_queueAccessMutex1.unlock();
            break;
        }
    }

    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

void Matcher::process(DAVIS240CEvent& e0, DAVIS240CEvent& e1)
{
    const int t0 = e0.m_timestamp;
    const int t1 = e1.m_timestamp;
    //int dt = std::abs(t0 - t1);
    //printf("%d. \n\r",dt);

    // Positive match if similar timestamps
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
    std::list<MatcherListener*>::iterator it;
    for(it = m_matcherListeners.begin(); it!=m_matcherListeners.end(); it++)
    {
        (*it)->receivedNewMatch(event0,event1);
    }
}
