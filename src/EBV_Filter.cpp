#include <EBV_Filter.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

Filter::Filter(int rows, int cols,
               DAVIS240C* davis) :
      m_rows(rows),
      m_cols(cols),
      m_davis(davis)
{
    // Initialize thread
    //m_thread = std::thread(&Filter::run,this);

    // Listen to Davis
    m_davis->registerEventListener(this);
    m_events.resize(m_rows*m_cols);

    // Parameters for flushing old events
    m_maxTimeToKeep = 1e4; //us (=>10ms)

    // Parameters for events filtering
    m_frequency = 600; //Hz (n°15=204 / n°17=167 / n°5=543, laser=600)
    m_eps = 10; // In percent of period T
    m_neighborSize = 3;//3; //2;
    m_threshSupportsA = 3;//5; //3;
    m_threshSupportsB = 3;//10;  //3;
    m_threshAntiSupports = 2;//5; //2;

    // Center of mass tracker initialization
    m_xc = 0;
    m_yc = 0;
    m_eta = 0.1;
}

Filter::~Filter()
{
    m_davis->deregisterEventListener(this);
}

void Filter::receivedNewDAVIS240CEvent(DAVIS240CEvent& e, int id)
{
    int x = e.m_x;
    int y = e.m_y;
    int p = e.m_pol;

    int t = e.m_timestamp;
    m_currTime = t;

    const int targetPeriod = 1e6/m_frequency;
    const int epsPeriod = m_eps*targetPeriod/100.;

    // Renaming for conveniency // DANGER SHARED RESOURCE
    std::list<DAVIS240CEvent>* eventsList = &(m_events[x*m_cols+y]);

    // Neighbor bounding box
    const int xMin = std::max(0,x-m_neighborSize);
    const int yMin = std::max(0,y-m_neighborSize);
    const int xMax = std::min(m_rows-1,x+m_neighborSize);
    const int yMax = std::min(m_cols-1,y+m_neighborSize);

    if (p<=0)
    {
        int nbSupportsA = 0;
        int nbSupportsB = 0;
        int nbAntiSupports = 0;
        for (int row=xMin; row<=xMax; row++)
        {
            for (int col=yMin; col<=yMax; col++)
            {
                // Get list of events at neighbor pixel // DANGER SHARED RESOURCE
                std::list<DAVIS240CEvent>* neighborList = &(m_events[row*m_cols+col]);

                // Iterate through events in neighbor pixel (recent first)
                std::list<DAVIS240CEvent>::reverse_iterator it;
                for(it = neighborList->rbegin(); it!=neighborList->rend(); it++)
                {
                    // Only consider events of negative polarity

                    if (it->m_pol>0){ continue; } // DANGER SHARED RESOURCE

                    int dt = (m_currTime - static_cast<int>(it->m_timestamp)); // DANGER SHARED RESOURCE
                    //printf("Delta: %d. \n\r",dt);

                    // Check if support event of type A: neighbor events with p=0 and t in [m_currTime-eps;m_currTime+eps]
                    if (dt < epsPeriod){ nbSupportsA++; }

                    // Check support events of type B: neighbor events with p=0 and t in [m_currTime+dt-eps;m_currTime+dt+eps]
                    else if (std::abs(dt-targetPeriod)<epsPeriod)
                    {
                        nbSupportsB++;
                    }

                    // Check anti-support event: neighbor events with p=0
                    else if (   (dt > epsPeriod)
                             && (dt < targetPeriod - epsPeriod))
                    {
                        nbAntiSupports++;
                    }

                    // Avoid unnecessary computation
                    if (nbAntiSupports>m_threshAntiSupports){ break; }
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
            m_xc = m_eta*m_xc + (1.-m_eta)*x;
            m_yc = m_eta*m_yc + (1.-m_eta)*y;

            // We send CoG coordinates
            e.m_x = m_xc;
            e.m_y = m_yc;

            //printf("Camera %d - Timestamp %d. \n\r",m_davis->m_id,e.m_timestamp);

            this->warnFilteredEvent(e);
        }

        // ==== DEBUG ====
        //printf("SupportsA: %d - SupportsB: %d - Antisupports: %d. \n\r",
        //       nbSupportsA, nbSupportsB, nbAntiSupports);
        // ==== END DEBUG ====

    }

    // Queue new event
    eventsList->push_back(e);

    // Remove old events from list
    // Hence to keep the list of a fixed time, we look at the beginning
    // of the list and start removing elements until we meet an element
    // which is new enough (since the elements are ordered, we can break
    // the loop which guarantees, O(1) removal instead of O(N) assuming
    // constant generation rate.
    std::list<DAVIS240CEvent>::iterator it = eventsList->begin(); // DANGER SHARED RESOURCE
    while(    ((m_currTime-it->m_timestamp) > m_maxTimeToKeep)
           && (it!=eventsList->end())
         )
    {
        eventsList->erase(it++);

        // DEBUG ===
        /*
        count++;
        if(x==0 && y==0)
        {
            std::cout << "size: " << list->size()
                      << "/deleted: " << count
                      << std::endl;
        }
        */ // === END DEBUG
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

void Filter::warnFilteredEvent(DAVIS240CEvent& filterEvent)
{
    std::list<FilterListener*>::iterator it;
    for(it = m_filteredEventListeners.begin(); it!=m_filteredEventListeners.end(); it++)
    {
        (*it)->receivedNewFilterEvent(filterEvent,m_davis->m_id);
    }
}
