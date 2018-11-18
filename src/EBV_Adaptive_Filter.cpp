#include <EBV_Adaptive_Filter.h>
#include <EBV_Matcher.h>
#include <EBV_DFF_Visualizer.h>

#include <cmath>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

Filter::Filter(DAVIS240C* davis)
    : m_davis(davis),
      m_rows(180),
      m_cols(240),
      m_frequency(530),
      m_eps(0.1*m_frequency),
      m_sigma(30),
      m_max_t(10e3) // 10ms
{
    m_last_event.resize(m_rows*m_cols);
    m_last_transitions.resize(m_rows*m_cols);
    m_last_filtered_events.resize(0);

    // Listen to Davis
    m_davis->registerEventListener(this);
}

Filter::~Filter()
{
    m_davis->deregisterEventListener(this);
}

// Associated to DAVIS event thread
void Filter::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
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
        }

        // Send transition (QUESTION: MAKE A COPY OR BY REF)
        DAVIS240CEvent e(x,y,p,t);
        receivedNewTransition(e,type);
    }

    // Update last events matrix
    m_last_event[x*m_cols+y] = std::make_pair(p,t);
}

void Filter::receivedNewTransition(DAVIS240CEvent& e,
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


void Filter::receivedNewHyperTransition(DAVIS240CEvent& e,
                                        const int dt)
{
    // Make a local copy
    const bool p = e.m_pol;
    const int x = e.m_x;
    const int y = e.m_y;
    const int t = e.m_timestamp;

    // Compare with reference frequency
    const int freq = int(1e6f/float(dt));
    if (std::abs(m_frequency-freq)<m_eps)
    {
        // Compute probability (gaussian)
        const float prob = std::exp( (freq-m_frequency)*(freq-m_frequency)
                                     /(2*m_sigma*m_sigma))
                                     /(m_sigma*std::sqrt(2*m_pi));

        if (prob>0.5f)
        {
            // Send filtered event
            DAVIS240CEvent e(x,y,p,t);
            warnFilteredEvent(e);

            // Save last filtered event in list
            m_last_filtered_events.push_back(e);

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

void Filter::registerFilterListener(FilterListener* listener)
{
    m_filter_listeners.push_back(listener);
}

void Filter::deregisterFilterListener(FilterListener* listener)
{
    m_filter_listeners.remove(listener);
}

void Filter::warnFilteredEvent(DAVIS240CEvent& filtEvent)
{
    std::list<FilterListener*>::iterator it;
    for(it = m_filter_listeners.begin(); it!=m_filter_listeners.end(); it++)
    {
        (*it)->receivedNewFilterEvent(filtEvent,m_davis->m_id);
    }
}
