#include <EBV_Matcher.h>

Matcher::Matcher(Filter* filter0, Filter* filter1)
    : m_filter0(filter0), m_filter1(filter1)
{
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
    bool ask_for_next0 = true;
    bool ask_for_next1 = true;
    DAVIS240CEvent event0{};
    DAVIS240CEvent event1{};

    while(true)
    {
        m_queueAccessMutex0.lock();
            hasQueueEvent0  = !m_evtQueue0.empty();
        m_queueAccessMutex0.unlock();

        m_queueAccessMutex1.lock();
            hasQueueEvent1  = !m_evtQueue1.empty();
        m_queueAccessMutex1.unlock();

        if (hasQueueEvent0 && hasQueueEvent1)
        {
            if(ask_for_next0)
            {
                m_queueAccessMutex0.lock();
                event0 = m_evtQueue0.front();
                m_evtQueue0.pop_front();
                m_queueAccessMutex0.unlock();
                ask_for_next0 = false;
            }

            if (ask_for_next1)
            {
                m_queueAccessMutex1.lock();
                event1 = m_evtQueue1.front();
                m_evtQueue1.pop_front();
                m_queueAccessMutex1.unlock();
                ask_for_next1 = false;
            }

            // Synchronize incoming filtered events
            if (event0.m_timestamp > event1.m_timestamp + m_eps)
            {
                //printf("Event1 too old: %d. \n\r", event0.m_timestamp - event1.m_timestamp);
                ask_for_next1 = true;
                continue;
            }
            else if (event1.m_timestamp > event0.m_timestamp + m_eps)
            {
                //printf("Event0 too old: %d. \n\r", event0.m_timestamp - event1.m_timestamp);
                ask_for_next0 = true;
                continue;
            }

            if (ask_for_next0==false && ask_for_next1==false)
            {
                this->process(event0,event1);
                ask_for_next0 = true;
                ask_for_next1 = true;
            }
        }
        else
        {
            // IDLE if no new received events
            std::unique_lock<std::mutex> condLock(m_condWaitMutex);
            m_condWait.wait(condLock);
            condLock.unlock();
        }
    }
}

// Called by the thread of previous agent (here Filter)
void Matcher::receivedNewFilterEvent(DAVIS240CEvent &event, const int id)
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

    // Match if similar timestamps
    if (std::abs(t0 - t1)<m_eps) { warnMatch(e0,e1); }
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
