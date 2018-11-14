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
      m_frequency(530),     //Hz (n°15=204 / n°17=167 / n°5=543, laser=600)
      m_targetPeriod(1e6/m_frequency),
      m_eps(10),            // In percent of period T
      m_epsPeriod((m_eps*m_targetPeriod)/100.f),
      m_neighborSize(3),    //3; //2;
      m_threshSupportsA(2), //5; //3;
      m_threshSupportsB(2), //10;  //3;
      m_threshAntiSupports(20),//5; //2;
      m_maxTimeToKeep(2*m_targetPeriod), //When to flush old events = 10ms
      m_xc(0.0f),
      m_yc(0.0f),
      m_eta(0.01f)
{
    // Listen to Davis
    m_events.resize(m_rows*m_cols);
    m_davis->registerEventListener(this);
}

Filter::~Filter()
{
    m_davis->deregisterEventListener(this);
}

// Associated to DAVIS event thread
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
                // Get list of events timestamps at neighbor pixel
                std::list<int>* neighborList = &(m_events[row*m_cols+col]);

                // Iterate through events in neighbor pixel (recent first -> end to begin)
                std::list<DAVIS240CEvent>::reverse_iterator it;
                for(auto it = neighborList->rbegin(); it!=neighborList->rend(); it++)
                {
                    // Remove old events from the list
                    if ((m_currTime-*it) > m_maxTimeToKeep )
                    {
                        neighborList->erase(neighborList->begin(),it.base());
                        break;
                    }

                    int dt = (m_currTime - *it);
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
                    if (nbAntiSupports>m_threshAntiSupports)
                    {
                        m_events[x*m_cols+y].push_back(e.m_timestamp);
                        return;
                    }
                }
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
                //m_xc = m_eta*m_xc + (1.f-m_eta)*x;
                //m_yc = m_eta*m_yc + (1.f-m_eta)*y;

                // We send CoG coordinates
                //e.m_x = static_cast<unsigned int>(m_xc);
                //e.m_y = static_cast<unsigned int>(m_yc);
            //m_evtMutex.unlock();

            //printf("Camera %d - Timestamp %d. \n\r",m_davis->m_id,e.m_timestamp);
            this->warnFilteredEvent(e);
        }

        // ==== DEBUG ====
        //printf("SupportsA: %d - SupportsB: %d - Antisupports: %d. \n\r",
        //       nbSupportsA, nbSupportsB, nbAntiSupports);
        // ==== END DEBUG ====

        // Queue new event (only if negative)
        m_events[x*m_cols+y].push_back(e.m_timestamp);
    }
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
