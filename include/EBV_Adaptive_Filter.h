#ifndef EBV_ADAPTIVE_FILTER_H
#define EBV_ADAPTIVE_FILTER_H

#include <EBV_DAVIS240C.h>

#include <string>
#include <list>
#include <vector>

class FilterListener
{
public:
    FilterListener(void) {}
    virtual void receivedNewFilterEvent(DAVIS240CEvent& filterEvent,
                                        const uint id) = 0;
};

class Filter : public DAVIS240CEventListener
{
public:
    Filter(DAVIS240C* davis = nullptr);
    ~Filter(); 

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const unsigned int id) override;
    void receivedNewTransition(DAVIS240CEvent& e, const bool transition);
    void receivedNewHyperTransition(DAVIS240CEvent& e, const int dt);

    void registerFilterListener(FilterListener* listener);
    void deregisterFilterListener(FilterListener* listener);
    void warnFilteredEvent(DAVIS240CEvent& event);

private:
    // Who filter listens to
    DAVIS240C* m_davis;

    const int m_rows;
    const int m_cols;
    int m_frequency;
    int m_eps;
    int m_sigma;
    const float m_pi{3.14159f};
    int m_current_t;
    int m_max_t;

    // Data structure matrix of list of events
    std::vector<std::pair<bool,int>> m_last_event;
    std::vector<std::array<int,2>> m_last_transitions;
    std::vector<DAVIS240CEvent> m_last_filtered_events;

    // List of filter listeners
    std::list<FilterListener*> m_filter_listeners;
};

#endif // EBV_ADAPTIVE_FILTER_H
