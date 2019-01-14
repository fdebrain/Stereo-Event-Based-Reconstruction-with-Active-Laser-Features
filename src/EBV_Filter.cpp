#include <EBV_Filter.h>
#include <EBV_Matcher.h>
#include <EBV_DFF_Visualizer.h>
#include <EBV_Benchmarking.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

Filter::Filter(int freq, DAVIS240C* davis)
    : m_davis(davis),
      m_id(davis->m_id),
      m_frequency(freq),
      m_targetPeriod(1e6f/(float)m_frequency),
      m_epsPeriod((m_eps*m_targetPeriod)/100.f),
      m_max_t(10*m_targetPeriod) //When to flush old events = 10ms
{
}

Filter::~Filter() {}

void Filter::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                       const int id)
{
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

// BASE FILTER
BaseFilter::BaseFilter(int freq, DAVIS240C* davis)
    : Filter(freq,davis)
{
    m_events.resize(m_rows*m_cols);
    m_davis->registerEventListener(this);
}

// Associated to DAVIS event thread
void BaseFilter::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                           const int id)
{
    const int x = e.m_x;
    const int y = e.m_y;
    const bool p = e.m_pol;
    m_current_t= e.m_timestamp;

    // Neighbor bounding box
    const int xMin = std::max(0,x-m_neighborSize);
    const int yMin = std::max(0,y-m_neighborSize);
    const int xMax = std::min(m_rows-1,x+m_neighborSize);
    const int yMax = std::min(m_cols-1,y+m_neighborSize);

    // DEBUG: When trying p>0 events, the master is always lagging behing the slave (about 1s), but many more events pass through the filter
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
                    if ((m_current_t-*it) > m_max_t )
                    {
                        neighborList->erase(neighborList->begin(),it.base());
                        break;
                    }

                    int dt = (m_current_t- *it);
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
            m_xc = m_eta*m_xc + (1.f-m_eta)*x;
            m_yc = m_eta*m_yc + (1.f-m_eta)*y;

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


// ADAPTIVE FILTER
AdaptiveFilter::AdaptiveFilter(int freq,DAVIS240C* davis)
      : Filter(freq,davis)
{
    // Initialize datastructures
    m_last_event.resize(m_rows*m_cols,{0,0});
    m_last_transitions.resize(m_rows*m_cols,{0,0});
    m_last_filtered_events.resize(0); // Optimization: estimate max size

    // Listen to Davis
    m_davis->registerEventListener(this);

    // Recording events frequency
    if (m_record) { m_recorder.open(m_eventRecordFile); }
}

AdaptiveFilter::~AdaptiveFilter()
{
   if (m_record) { m_recorder.close(); }
}

// Associated to DAVIS event thread
void AdaptiveFilter::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                               const int id)
{
    // Make a local copy
    const bool p = e.m_pol;
    const int x = e.m_x;
    const int y = e.m_y;
    const int t = e.m_timestamp;
    const int laser_x = e.m_laser_x;
    const int laser_y = e.m_laser_y;
    m_current_t = t;

    // Check if different to last event's polarity
    if (m_last_event[x*m_cols+y].second > 0)
    {
        const bool last_p = m_last_event[x*m_cols+y].first;

        // Encode transition type (0=falling, 1=rising)
        if (last_p != p)
        {
            bool type = (p > last_p?1:0);

            // Send transition event
            DAVIS240CEvent e{x,y,p,t,laser_x,laser_y};
            receivedNewTransition(e,type);
        }
    }

    // Update last events matrix
    m_last_event[x*m_cols+y] = std::make_pair(p,t);
}

void AdaptiveFilter::receivedNewTransition(DAVIS240CEvent& e,
                                           const bool type)
{
    // Make a local copy
    const bool p = e.m_pol;
    const int x = e.m_x;
    const int y = e.m_y;
    const int t = e.m_timestamp;
    const int laser_x = e.m_laser_x;
    const int laser_y = e.m_laser_y;

    // Compare to last transition event, if one
    if (m_last_transitions[x*m_cols+y][type] > 0)
    {
        // Compute delta between to transitions of same type
        const int last_t = m_last_transitions[x*m_cols+y][type];
        const int dt = t - last_t;

        // Send hyper transition
        DAVIS240CEvent e{x,y,p,t,laser_x,laser_y};
        receivedNewHyperTransition(e,dt);
    }

    // Update last transition matrix
    m_last_transitions[x*m_cols+y][type] = t;
}


void AdaptiveFilter::receivedNewHyperTransition(DAVIS240CEvent& e,
                                                const int dt)
{
    // Make a local copy
    const bool p = e.m_pol;
    const int x = e.m_x;
    const int y = e.m_y;
    const int t = e.m_timestamp;
    const int laser_x = e.m_laser_x;
    const int laser_y = e.m_laser_y;
    const int freq = int(1e6f/float(dt));

    if (m_record) { m_recorder << freq << '\n'; }

    //=== DEBUG ===//
//    if (freq>200)
//    {
//        printf("Freq: %d. \n\r", freq);

//    }
    //=== END DEBUG ===//

    if (std::abs(m_frequency-freq)<m_sigma)
    {
        // Center of mass tracker
        m_xc = m_eta*m_xc + (1.f-m_eta)*x;
        m_yc = m_eta*m_yc + (1.f-m_eta)*y;

        // Send filtered event
        //DAVIS240CEvent e{m_xc,m_yc,p,t,laser_x,laser_y};
        DAVIS240CEvent e{x,y,p,t,laser_x,laser_y};
        warnFilteredEvent(e);

        // Save last filtered event in list
        m_last_filtered_events.emplace_back(x,y,p,t,laser_x,laser_y);


        // Remove old filtered events
        auto it = m_last_filtered_events.begin();
        while(    ((m_current_t-it->m_timestamp) > m_max_t)
               && (it!=m_last_filtered_events.end())
        )
        {
            m_last_filtered_events.erase(it++);
        }
    }
}

// ADAPTIVE FILTER BIS
AdaptiveFilterNeighbor::AdaptiveFilterNeighbor(int freq,DAVIS240C* davis)
      : AdaptiveFilter(freq,davis)
{
    // Initialize datastructures
    m_last_event.resize(m_rows*m_cols,{0,0});
    m_last_transitions.resize(m_rows*m_cols,{0,0});
    m_last_hypertransitions.resize(m_rows*m_cols,-1);
    m_last_filtered_events.resize(0); // Optimization: estimate max size

    // Listen to Davis
    m_davis->registerEventListener(this);

    // Recording events frequency
    if (m_record) { m_recorder.open(m_eventRecordFile); }
}

AdaptiveFilterNeighbor::~AdaptiveFilterNeighbor()
{
   if (m_record) { m_recorder.close(); }
}

// Associated to DAVIS event thread
void AdaptiveFilterNeighbor::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                               const int id)
{
    // Make a local copy
    const bool p = e.m_pol;
    const int x = e.m_x;
    const int y = e.m_y;
    const int t = e.m_timestamp;
    const int laser_x = e.m_laser_x;
    const int laser_y = e.m_laser_y;
    m_current_t = t;

    // Check if different to last event's polarity
    if (m_last_event[x*m_cols+y].second > 0)
    {
        const bool last_p = m_last_event[x*m_cols+y].first;

        // Encode transition type (0=falling, 1=rising)
        if (last_p != p)
        {
            bool type = (p > last_p?1:0);

            // Send transition event
            DAVIS240CEvent e{x,y,p,t,laser_x,laser_y};
            receivedNewTransition(e,type);
        }
    }

    // Update last events matrix
    m_last_event[x*m_cols+y] = std::make_pair(p,t);
}

void AdaptiveFilterNeighbor::receivedNewTransition(DAVIS240CEvent& e,
                                           const bool type)
{
    // Make a local copy
    const bool p = e.m_pol;
    const int x = e.m_x;
    const int y = e.m_y;
    const int t = e.m_timestamp;
    const int laser_x = e.m_laser_x;
    const int laser_y = e.m_laser_y;

    // Compare to last transition event, if one
    if (m_last_transitions[x*m_cols+y][type] > 0)
    {
        // Compute delta between to transitions of same type
        const int last_t = m_last_transitions[x*m_cols+y][type];
        const int dt = t - last_t;

        // Send hyper transition event
        DAVIS240CEvent e{x,y,p,t,laser_x,laser_y};
        receivedNewHyperTransition(e,dt);
    }

    // Update last transition matrix
    m_last_transitions[x*m_cols+y][type] = t;
}


void AdaptiveFilterNeighbor::receivedNewHyperTransition(DAVIS240CEvent& e,
                                                const int dt)
{
    // Make a local copy
    const bool p = e.m_pol;
    const int x = e.m_x;
    const int y = e.m_y;
    const int t = e.m_timestamp;
    const int laser_x = e.m_laser_x;
    const int laser_y = e.m_laser_y;

    const int freq = int(1e6f/float(dt));
    if (m_record) { m_recorder << freq << '\n'; }

    // Update last hypertransition matrix
    m_last_hypertransitions[x*m_cols+y] = freq;

    if (std::abs(m_frequency-freq)<m_sigma)
    {
        // Send filtered event
        DAVIS240CEvent e{x,y,p,t,laser_x,laser_y};
        warnFilteredEvent(e);

        // Center of mass tracker
        m_xc = m_eta*m_xc + (1.f-m_eta)*x;
        m_yc = m_eta*m_yc + (1.f-m_eta)*y;
    }
    else
    {
        // Chekc neighborhood frequency consistency (frequency blob detection)
        int nb_supports = 0;
        const int x_min = std::max(0,x-1);
        const int y_min = std::max(0,y-1);
        const int x_max = std::min(m_rows-1,x+1);
        const int y_max = std::min(m_cols-1,y+1);
        for (int row=x_min; row<=x_max; row++)
        {
            for (int col=y_min; col<=y_max; col++)
            {
                if( std::abs(m_frequency-m_last_hypertransitions[row*m_cols+col])<m_sigma)
                {
                    nb_supports++;
                }
            }
        }

        // Trigger new filtered event if enough support events
        if (nb_supports>m_thresh_supports)
        {
            // Send filtered event
            DAVIS240CEvent e{x,y,p,t,laser_x,laser_y};
            warnFilteredEvent(e);

            // Center of mass tracker
            m_xc = m_eta*m_xc + (1.f-m_eta)*x;
            m_yc = m_eta*m_yc + (1.f-m_eta)*y;
        }
    }
}
