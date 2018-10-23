#include <EBV_Matcher.h>

void Matcher::setFilter(Filter* filter, int id)
{
    switch (id)
    {
    case 0:
        m_filter0 = filter;
        break;
    case 1:
        m_filter1 = filter;
        break;
    }
}

Matcher::Matcher(int rows, int cols,
                 Filter* filter0,
                 Filter* filter1)
    : m_rows(rows), m_cols(cols)
{
    // Initialize thread
    m_thread = std::thread(&Matcher::run,this);

    // Matching parameters
    m_eps = 1e2;

    // Initialize filter
    this->setFilter(filter0,0);
    this->setFilter(filter1,1);
}

Matcher::~Matcher() {}

// The function the thread executes and waits on
void Matcher::run()
{
    bool hasQueueEvent0;
    bool hasQueueEvent1;
    DAVIS240CEvent event0;
    DAVIS240CEvent event1;
    while(true)
    {
        m_queueAccessMutex.lock();
            hasQueueEvent0  =!m_evtQueue0.empty();
            hasQueueEvent1  =!m_evtQueue1.empty();
        m_queueAccessMutex.unlock();

        // Process only if incoming filtered events in both cameras
        if(hasQueueEvent0 && hasQueueEvent1)
        {
            m_queueAccessMutex.lock();
                event0 = m_evtQueue0.front();
                m_evtQueue0.pop_front();
                event1 = m_evtQueue1.front();
                m_evtQueue1.pop_front();
            m_queueAccessMutex.unlock();

            process();
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
    switch (id)
    {
    case 1:
        m_queueAccessMutex.lock();
            m_evtQueue0.push_back(event);
        m_queueAccessMutex.unlock();
        break;
    case 2:
        m_queueAccessMutex.lock();
            m_evtQueue1.push_back(event);
        m_queueAccessMutex.unlock();
        break;
    }

    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

// Processing to be done for each event
void Matcher::process()
{
    DAVIS240CEvent e0 = m_filter0->getCoGEvent();
    DAVIS240CEvent e1 = m_filter1->getCoGEvent();

    // Positive match if similar timestamps
    if (std::abs(e0.m_timestamp - e1.m_timestamp)<m_eps)
    {
        warnMatch(e0,e1);
        printf("WarnMatch");
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
