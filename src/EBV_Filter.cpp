#include <EBV_Filter.h>

//float dtMean = 0;
//float eta = 0.99;

Filter::Filter(int rows, int cols)
    : m_rows(rows),
      m_cols(cols)
{
    m_events.resize(m_rows*m_cols);

    // Parameters for flushing old events
    m_maxTimeToKeep = 1e5; //us (=>100ms)

    // Parameters for events filtering
    m_targetPeriod = 1680; //us
    m_eps = 0.01*m_targetPeriod; //us
    m_neighborSize = 3;
    m_threshSupports = 50;
    m_threshAntiSupports = 50;
}

Filter::~Filter()
{}

void Filter::receivedNewDAVIS240CEvent(DAVIS240CEvent& e, int id)
{
    int x = e.m_x;
    int y = e.m_y;
    int t = e.m_timestamp;
    int p = e.m_pol;

    m_currTime = t;

    // Renaming for conveniency
    std::list<DAVIS240CEvent>* eventsList = &(m_events[x*m_cols+y]);

    // Neighbor bounding box
    const int xMin = std::max(0,x-m_neighborSize);
    const int yMin = std::max(0,y-m_neighborSize);
    const int xMax = std::min(m_rows,x+m_neighborSize);
    const int yMax = std::min(m_cols,y+m_neighborSize);

    // Condition 0: Only consider incoming events with p=0
    if (p<=0)
    {
        // Condition 1: Enough nbr events (supports A) of same polarity and timestamp (approximately) in neighbor pixels
        int nbSupportsA = 0;
        for (int row=xMin; row<xMax; row++)
        {
            for (int col=yMin; col<yMax; col++)
            {
                // Get list of events at neighbor pixel
                std::list<DAVIS240CEvent>* neighborList = &(m_events[row*m_cols+col]);

                // Iterate through events in neighbor pixel (recent first)
                std::list<DAVIS240CEvent>::reverse_iterator it;
                for(it = neighborList->rbegin(); it!=neighborList->rend(); it++)
                {
                    // Check if support event of type A: neighbor events with p=0 and t in [m_currTime-eps;m_currTime+eps]
                    if (   (it->m_pol<=0)
                        && (it->m_timestamp > (m_currTime - m_eps))
                        && (it->m_timestamp < (m_currTime + m_eps))
                       )
                    {
                        nbSupportsA++;
                        /* DEBUG ==
                        if (nbSupports>m_threshSupports)
                        {
                            printf(" Found laser at (%d,%d) with %d supports.\n\r",col,row,nbSupports);
                        }
                        */ // == END DEBUG
                    }
                    // Otherwise: useless to check following event in the list (ordered in timestamps)
                    //else {break;}
                }
            }
        } // end if condition 1

        // Condition2: Condition 1 + enough nbr events (supports B) of same polarity and timestamp is one period after
        int nbSupportsB = 0;
        if (nbSupportsA>m_threshSupports)
        {
            for (int row=xMin; row<xMax; row++)
            {
                for (int col=yMin; col<yMax; col++)
                {
                    // Get list of events at neighbor pixel
                    std::list<DAVIS240CEvent>* neighborList = &(m_events[row*m_cols+col]);

                    // Iterate through events in neighbor pixel (recent first)
                    std::list<DAVIS240CEvent>::reverse_iterator it;
                    for(it = neighborList->rbegin(); it!=neighborList->rend(); it++)
                    {
                        // Check support events of type B: neighbor events with p=0 and t in [m_currTime+dt-eps;m_currTime+dt+eps]
                        if (   (it->m_pol<=0)
                            && (it->m_timestamp > (m_currTime + m_targetPeriod - m_eps))
                            && (it->m_timestamp < (m_currTime + m_targetPeriod + m_eps))
                           )
                        {
                            nbSupportsB++;
                        }
                    }
                }
            }
        } // end if condition 2

        // Condition3: Condition 2 + few noisy events (anti-supports)
        int nbAntiSupports = 0;
        if (nbSupportsB>m_threshSupports)
        {
            for (int row=xMin; row<xMax; row++)
            {
                for (int col=yMin; col<yMax; col++)
                {
                    // Get list of events at neighbor pixel
                    std::list<DAVIS240CEvent>* neighborList = &(m_events[row*m_cols+col]);

                    // Iterate through events in neighbor pixel (recent first)
                    std::list<DAVIS240CEvent>::reverse_iterator it;
                    for(it = neighborList->rbegin(); it!=neighborList->rend(); it++)
                    {
                        // Check anti-support event: neighbor events with p=0
                        // and t in [m_currTime+eps;m_currTime+dt/2-eps] or [m_currTime+dt/2+eps;m_currTime+dt-eps]
                        if (it->m_pol<=0)
                        {
                            if (   (it->m_timestamp > (m_currTime + m_eps))
                                && (it->m_timestamp < (m_currTime + m_targetPeriod/2 - m_eps))
                               )
                            {
                                nbAntiSupports++;
                            }

                            if (   (it->m_timestamp > (m_currTime + m_targetPeriod/2 + m_eps))
                                && (it->m_timestamp < (m_currTime + m_targetPeriod - m_eps))
                               )
                           {
                                nbAntiSupports++;
                           }
                        }
                    }
                }
            }
        } // end if condition 3

        // If incoming event fulfil all 4 conditions it's likely to be stemming from the laser
        if (nbAntiSupports<m_threshAntiSupports)
        {
            printf(" Found laser at (%d,%d) with %d anti-supports.\n\r",x,y,nbAntiSupports);
            this->warnFilteredEvent(e,id);
        }
    } // end if condition 0


    // Get mean period between events at (x,y)
    /*
    if (x==m_laserPosX && y==m_laserPosY && p<=0)
    {
        // Iterate through events in pixel (recent first)
        std::list<DAVIS240CEvent>::reverse_iterator it;
        for(it = eventsList->rbegin(); it!=eventsList->rend(); it++)
        {
            // Find first event with pol=0
            if(it->m_pol <= 0)
            {
                dtMean = eta*dtMean + (1-eta)*(t-it->m_timestamp);
                printf("Last: %u | Current: %u | dtMean: %f.\n\r",
                       it->m_timestamp,t,dtMean);
                break;
            }
        }
    }
    */

    // Queue new event
    eventsList->push_back(e);

    // Remove old events from list
    // Hence to keep the list of a fixed time, we look at the beginning
    // of the list and start removing elements until we meet an element
    // which is new enough (since the elements are ordered, we can break
    // the loop which guarantees, O(1) removal instead of O(N) assuming
    // constant generation rate.
    std::list<DAVIS240CEvent>::iterator it = eventsList->begin();
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

//====
// Register a listener to receive the new events
void Filter::registerFilterListener(FilterListener* listener)
{
    m_filteredEventListeners.push_back(listener);
}

void Filter::deregisterFilterListener(FilterListener* listener)
{
    m_filteredEventListeners.remove(listener);
}

// Send new event to all listeners
void Filter::warnFilteredEvent(DAVIS240CEvent& filterEvent, int id)
{
    std::list<FilterListener*>::iterator it;
    for(it = m_filteredEventListeners.begin(); it!=m_filteredEventListeners.end(); it++)
    {
        (*it)->receivedNewFilterEvent(filterEvent,id);
    }
}
//====
