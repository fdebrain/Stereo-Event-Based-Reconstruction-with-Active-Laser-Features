#include <EBV_Matcher.h>

Matcher::Matcher(int rows, int cols,
                 Filter* filter0,
                 Filter* filter1)
    : m_rows(rows), m_cols(cols),
      m_filter0(filter0), m_filter1(filter1)
{
    // Matching parameters
    m_eps = 5e4;
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
        m_queueAccessMutex.lock();
            hasQueueEvent0  = !m_evtQueue0.empty();
            hasQueueEvent1  = !m_evtQueue1.empty();
        m_queueAccessMutex.unlock();

        // Process only if incoming filtered events in both cameras
        //CHANGE
        if(hasQueueEvent0 && hasQueueEvent1)
        {
            m_queueAccessMutex.lock();
                event0 = m_evtQueue0.front();
                event1 = m_evtQueue1.front();

                // DEBUG - QUEUE SIZE SHOULD BE SIMILAR
                //printf("Queue size: %d - %d.\n\r",
                //       m_evtQueue0.size(),
                //       m_evtQueue1.size());
                // END DEBUG
            m_queueAccessMutex.unlock();

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
void Matcher::receivedNewFilterEvent(DAVIS240CEvent& event, int id)
{
    // DEBUG - CHECK IF EVENTS SYNCHRONIZED
    //printf("Camera %d - Timestamp %d. \n\r",id,event.m_timestamp);
    // END DEBUG

    m_currTime = event.m_timestamp;

    if (id==0)
    {
        m_queueAccessMutex.lock();
            m_evtQueue0.push_back(event);

            std::list<DAVIS240CEvent>::iterator it = m_evtQueue0.begin(); // DANGER SHARED RESOURCE
            while(    ((m_currTime-it->m_timestamp) > m_maxTimeToKeep)
                   && (it!=m_evtQueue0.end())
                 )
            {
                m_evtQueue0.erase(it++);
            }

        m_queueAccessMutex.unlock();
    }
    else
    {
        m_queueAccessMutex.lock();
            m_evtQueue1.push_back(event);

            std::list<DAVIS240CEvent>::iterator it = m_evtQueue1.begin(); // DANGER SHARED RESOURCE
            while(    ((m_currTime-it->m_timestamp) > m_maxTimeToKeep)
                   && (it!=m_evtQueue1.end())
                 )
            {
                m_evtQueue1.erase(it++);
            }

        m_queueAccessMutex.unlock();
    }

    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

// Processing to be done for each event
void Matcher::process(DAVIS240CEvent e0, DAVIS240CEvent e1)
{
    // Positive match if similar timestamps
    if (std::abs(e0.m_timestamp - e1.m_timestamp)<m_eps)
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

void Matcher::warnMatch(DAVIS240CEvent& event0,
                        DAVIS240CEvent& event1)
{
    std::list<MatcherListener*>::iterator it;
    for(it = m_matcherListeners.begin(); it!=m_matcherListeners.end(); it++)
    {
        (*it)->receivedNewMatch(event0,event1);
    }
}
