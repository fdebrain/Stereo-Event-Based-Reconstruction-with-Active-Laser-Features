#include <EBV_Filter.h>
#include <EBV_Matcher.h>
#include <EBV_DFF_Visualizer.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

Filter::Filter(const unsigned int rows,
               const unsigned int cols,
               DAVIS240C* davis)
    : m_rows(rows),
      m_cols(cols),
      m_davis(davis),
      m_maxTimeToKeep(1e4), //When to flush old events = 10ms
      m_frequency(600),     //Hz (n°15=204 / n°17=167 / n°5=543, laser=600)
      m_targetPeriod(1e6/m_frequency),
      m_eps(10),            // In percent of period T
      m_epsPeriod((m_eps*m_targetPeriod)/100.f),
      m_neighborSize(4),    //3; //2;
      m_threshSupportsA(3), //5; //3;
      m_threshSupportsB(3), //10;  //3;
      m_threshAntiSupports(20),//5; //2;
      m_xc(0.0f),
      m_yc(0.0f),
      m_eta(0.01f)
{
    // Listen to Davis
    m_events.resize(m_rows*m_cols);
    m_davis->registerEventListener(this);

    // Initialize thread
    //m_thread = std::thread(&Filter::runThread,this);
    //printf("Starting filter in thread %d. \n\r",m_thread.get_id());
}

Filter::~Filter()
{
    m_davis->deregisterEventListener(this);
}

void Filter::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                       const unsigned int id)
{
    // QUESTION: Since event is passed by reference and method is also called in visu,
    // should we care about thread-safety ? e is accessed by multiple thread
    const unsigned int x = e.m_x;
    const unsigned int y = e.m_y;
    const unsigned int p = e.m_pol;
    m_currTime = e.m_timestamp;

    // Neighbor bounding box
    const int xMin = std::max(0,int(x)-m_neighborSize);
    const int yMin = std::max(0,int(y)-m_neighborSize);
    const int xMax = std::min(int(m_rows-1),int(x)+m_neighborSize);
    const int yMax = std::min(int(m_cols-1),int(y)+m_neighborSize);


    // QUESTION: When I try p>0 events, the master is always lagging behing the slave (about 1s), but many more events pass through the filter
    if (p<=0)
    {
        int nbSupportsA = 0;
        int nbSupportsB = 0;
        int nbAntiSupports = 0;
        for (int row=xMin; row<=xMax; row++)
        {
            for (int col=yMin; col<=yMax; col++)
            {
                // QUESTION: Do we really need to lock a mutex here ? Seems expansive and already works well without
                //m_evtMutex.lock();
                // Get list of events at neighbor pixel // DANGER SHARED RESOURCE
                std::list<DAVIS240CEvent>* neighborList = &(m_events[row*m_cols+col]);

                // Iterate through events in neighbor pixel (recent first)
                std::list<DAVIS240CEvent>::reverse_iterator it;
                for(it = neighborList->rbegin(); it!=neighborList->rend(); it++)
                {
                    // Only consider events of negative polarity
                    if (it->m_pol>0){ continue; }

                    const int t = it->m_timestamp;

                    int dt = (m_currTime - t);
                    //printf("Delta: %d. \n\r",dt);

                    // Check if support event of type A: neighbor events with p=0 and t in [m_currTime-eps;m_currTime+eps]
                    if (dt < m_epsPeriod){ nbSupportsA++; }

                    // Check support events of type B: neighbor events with p=0 and t in [m_currTime+dt-eps;m_currTime+dt+eps]
                    else if (std::abs(dt-m_targetPeriod)<m_epsPeriod)
                    {
                        nbSupportsB++;
                    }

                    // Check anti-support event: neighbor events with p=0
                    else if (   (dt > m_epsPeriod)
                             && (dt < m_targetPeriod - m_epsPeriod))
                    {
                        nbAntiSupports++;
                    }

                    // Avoid unnecessary computation
                    if (nbAntiSupports>m_threshAntiSupports){ break; }
                }
                //m_evtMutex.unlock();
            }
        }

        // Update tracker & send filtered event
        if (   nbSupportsA>m_threshSupportsA
            && nbSupportsB>m_threshSupportsB
            && nbAntiSupports<m_threshAntiSupports
           )
        {
            // Center of mass tracker
            //m_evtMutex.lock();
                m_xc = m_eta*m_xc + (1.f-m_eta)*x;
                m_yc = m_eta*m_yc + (1.f-m_eta)*y;

                // We send CoG coordinates
                e.m_x = static_cast<unsigned int>(m_xc);
                e.m_y = static_cast<unsigned int>(m_yc);
            //m_evtMutex.unlock();

            //printf("Camera %d - Timestamp %d. \n\r",m_davis->m_id,e.m_timestamp);
            this->warnFilteredEvent(e);
        }

        // ==== DEBUG ====
        //printf("SupportsA: %d - SupportsB: %d - Antisupports: %d. \n\r",
        //       nbSupportsA, nbSupportsB, nbAntiSupports);
        // ==== END DEBUG ====
    }

    // Queue new event
    //m_evtMutex.lock();
        std::list<DAVIS240CEvent>* eventsList = &(m_events[x*m_cols+y]);
        eventsList->push_back(e);

        // Remove old events from list O(1) assuming constant generation rate.
        std::list<DAVIS240CEvent>::iterator it = eventsList->begin();
        const int t = it->m_timestamp;
        while(((m_currTime-t) > m_maxTimeToKeep) && (it!=eventsList->end()))
        {
            eventsList->erase(it++);
        }
    //m_evtMutex.unlock();
}

void Filter::registerFilterListener(FilterListener* listener)
{
    m_filteredEventListeners.push_back(listener);
}

void Filter::deregisterFilterListener(FilterListener* listener)
{
    m_filteredEventListeners.remove(listener);
}

void Filter::warnFilteredEvent(DAVIS240CEvent& filtEvent)
{
    std::list<FilterListener*>::iterator it;
    for(it = m_filteredEventListeners.begin(); it!=m_filteredEventListeners.end(); it++)
    {
        (*it)->receivedNewFilterEvent(filtEvent,m_davis->m_id);
    }
}

/*
void Filter::runThread()
{
    bool hasQueueEvent;
    DAVIS240CEvent event;

    while(true)
    {
        m_queueAccessMutex.lock();
            hasQueueEvent = !m_evtQueue.empty();
        m_queueAccessMutex.unlock();

        if(hasQueueEvent)
        {
            m_queueAccessMutex.lock();
                event = m_evtQueue.front();
                m_evtQueue.pop_front();
            m_queueAccessMutex.unlock();

            process(event);
        }
        else
        {
            std::unique_lock<std::mutex> condLock(m_condWaitMutex);
            m_condWait.wait(condLock);
            condLock.unlock();
        }
    }
}

void Filter::receivedNewDAVIS240CEvent(DAVIS240CEvent& event,
                                       const unsigned int id)
{
    m_queueAccessMutex.lock();
        m_evtQueue.push_back(event);
    m_queueAccessMutex.unlock();

    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

void Filter::process(DAVIS240CEvent& event)
{
    const unsigned int x = event.m_x;
    const unsigned int y = event.m_y;
    const unsigned int p = event.m_pol;
    m_currTime = event.m_timestamp;

    // Neighbor bounding box
    const int xMin = std::max(0,int(x)-m_neighborSize);
    const int yMin = std::max(0,int(y)-m_neighborSize);
    const int xMax = std::min(int(m_rows-1),int(x)+m_neighborSize);
    const int yMax = std::min(int(m_cols-1),int(y)+m_neighborSize);

    if (p<=0)
    {
        int nbSupportsA = 0;
        int nbSupportsB = 0;
        int nbAntiSupports = 0;
        for (int row=xMin; row<=xMax; row++)
        {
            for (int col=yMin; col<=yMax; col++)
            {
                m_queueAccessMutex.lock();
                // Get list of events at neighbor pixel // DANGER SHARED RESOURCE
                std::list<DAVIS240CEvent>* neighborList = &(m_events[row*m_cols+col]);

                // Iterate through events in neighbor pixel (recent first)
                std::list<DAVIS240CEvent>::reverse_iterator it;
                for(it = neighborList->rbegin(); it!=neighborList->rend(); it++)
                {
                    // Only consider events of negative polarity

                    const int pol = it->m_pol;
                    if (pol>0){ continue; }

                    const int t = it->m_timestamp;
                    int dt = (m_currTime - t);
                    //printf("Delta: %d. \n\r",dt);

                    // Check if support event of type A: neighbor events with p=0 and t in [m_currTime-eps;m_currTime+eps]
                    if (dt < m_epsPeriod){ nbSupportsA++; }

                    // Check support events of type B: neighbor events with p=0 and t in [m_currTime+dt-eps;m_currTime+dt+eps]
                    else if (std::abs(dt-m_targetPeriod)<m_epsPeriod)
                    {
                        nbSupportsB++;
                    }

                    // Check anti-support event: neighbor events with p=0
                    else if (   (dt > m_epsPeriod)
                             && (dt < m_targetPeriod - m_epsPeriod))
                    {
                        nbAntiSupports++;
                    }

                    // Avoid unnecessary computation
                    if (nbAntiSupports>m_threshAntiSupports){ break; }
                }
                m_queueAccessMutex.unlock();
            }
        }

        // Update tracker & send filtered event
        if (   nbSupportsA>m_threshSupportsA
            && nbSupportsB>m_threshSupportsB
            && nbAntiSupports<m_threshAntiSupports
           )
        {
            // Center of mass tracker
            m_queueAccessMutex.lock();
                m_xc = m_eta*m_xc + (1.f-m_eta)*x;
                m_yc = m_eta*m_yc + (1.f-m_eta)*y;

                // We send CoG coordinates
                event.m_x = static_cast<unsigned int>(m_xc);
                event.m_y = static_cast<unsigned int>(m_yc);
            m_queueAccessMutex.unlock();
            //printf("Camera %d - Timestamp %d. \n\r",m_davis->m_id,e.m_timestamp);
            this->warnFilteredEvent(event);
        }

        // ==== DEBUG ====
        //printf("SupportsA: %d - SupportsB: %d - Antisupports: %d. \n\r",
        //       nbSupportsA, nbSupportsB, nbAntiSupports);
        // ==== END DEBUG ====
    }

    // Queue new event
    m_queueAccessMutex.lock();
    std::list<DAVIS240CEvent>* eventsList = &(m_events[x*m_cols+y]);
    eventsList->push_back(event);

    // Remove old events from list O(1) assuming constant generation rate.
    std::list<DAVIS240CEvent>::iterator it = eventsList->begin(); // DANGER SHARED RESOURCE
        const int t = it->m_timestamp;
        while(    ((m_currTime-t) > m_maxTimeToKeep)
               && (it!=eventsList->end())
             )
        {
            eventsList->erase(it++);
        }
    m_queueAccessMutex.unlock();
}
*/

