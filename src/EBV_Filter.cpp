#include <EBV_Filter.h>
#include <EBV_Matcher.h>
#include <EBV_DFF_Visualizer.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

Filter::Filter(DAVIS240C* davis)
    : m_davis(davis),
      m_rows(180),
      m_cols(240),
      m_frequency(630),     //Hz (n°15=204 / n°17=167 / n°5=543, laser=600)
      m_targetPeriod(1e6/m_frequency),
      m_eps(10), // In percent of period T
      m_epsPeriod((m_eps*m_targetPeriod)/100.f),
      m_max_t(2*m_targetPeriod), //When to flush old events = 10ms
      m_xc(0.0f),
      m_yc(0.0f),
      m_eta(0.01f)
{
}

Filter::~Filter()
{
}

void Filter::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                       const uint id)
{
    printf("Filter Event! \n\r");
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
BaseFilter::BaseFilter(DAVIS240C* davis)
    : Filter(davis),
      m_neighborSize(3),    //3; //2;
      m_threshSupportsA(2), //5; //3;
      m_threshSupportsB(2), //10;  //3;
      m_threshAntiSupports(20)//5; //2;
{
    // Initialize datastructures
    m_events.resize(m_rows*m_cols);

    // Listen to Davis
    m_davis->registerEventListener(this);
}

// Associated to DAVIS event thread
void BaseFilter::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                       const uint id)
{
    // QUESTION: Since event is passed by reference and method is also called in visu,
    // should we care about thread-safety ? e is accessed by multiple thread
    const int x = e.m_x;
    const int y = e.m_y;
    const bool p = e.m_pol;
    m_current_t= e.m_timestamp;

    // Neighbor bounding box
    const int xMin = std::max(0,x-m_neighborSize);
    const int yMin = std::max(0,y-m_neighborSize);
    const int xMax = std::min(m_rows-1,x+m_neighborSize);
    const int yMax = std::min(m_cols-1,y+m_neighborSize);

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

            // We send CoG coordinates
            //e.m_x = static_cast<uint>(m_xc);
            //e.m_y = static_cast<uint>(m_yc);

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


// ADAPTIVE FILTER
AdaptiveFilter::AdaptiveFilter(DAVIS240C* davis)
      : Filter(davis),
        m_sigma(60)
{
    // Initialize datastructures
    m_last_event.resize(m_rows*m_cols);
    m_last_transitions.resize(m_rows*m_cols);
    m_last_filtered_events.resize(0); // Optimization: estimate max size

    // Listen to Davis
    m_davis->registerEventListener(this);
}

// Associated to DAVIS event thread
void AdaptiveFilter::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                               const uint id)
{
    // Make a local copy
    const bool p = e.m_pol;
    const int x = e.m_x;
    const int y = e.m_y;
    const int t = e.m_timestamp;
    m_current_t = t;

    // Compare to last event
    if (m_last_event[x*m_cols+y].second)
    {
        // Make a copy of last event's polarity, if one
        const bool last_p = m_last_event[x*m_cols+y].first;
        bool type{};

        // Encode transition type (0=falling, 1=rising)
        if (last_p!=p)
        {
            type = (p>last_p?1:0);

            // Send event if transition (QUESTION: MAKE A COPY OR BY REF)
            DAVIS240CEvent e(x,y,p,t);
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

    // Compare to last transition event, if one
    if (m_last_transitions[x*m_cols+y][type])
    {
        // Compute delta between to transitions of same type
        const int last_t = m_last_transitions[x*m_cols+y][type];
        const int dt = t - last_t;

        // Send hyper transition
        DAVIS240CEvent e(x,y,p,t);
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

    // Compare with reference frequency
    const int freq = int(1e6f/float(dt));

    if (std::abs(m_frequency-freq)<m_sigma)
    {
        // Compute probability (gaussian)
        const double prob = std::exp( -(freq-m_frequency)*(freq-m_frequency)
                                     /(2.*m_sigma*m_sigma));
        //printf("Freq: %d - Prob: %f. \n\r", freq, prob);

        if (prob>0.1f)
        {
            // Send filtered event
            DAVIS240CEvent e(x,y,p,t);
            warnFilteredEvent(e);

            // Center of mass tracker
            m_xc = m_eta*m_xc + (1.f-m_eta)*x;
            m_yc = m_eta*m_yc + (1.f-m_eta)*y;

            // Save last filtered event in list
            //m_last_filtered_events.push_back(e);
            m_last_filtered_events.emplace_back(x,y,p,t);

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
}
