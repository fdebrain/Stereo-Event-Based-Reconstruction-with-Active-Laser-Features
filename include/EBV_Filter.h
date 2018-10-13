#ifndef EBV_FILTER_H
#define EBV_FILTER_H

#include <EBV_DAVIS240C.h>

#include <vector>
#include <mutex>
#include <list>

//=====
class FilterListener
{
    public:
        FilterListener(void) {}
        virtual void receivedNewFilterEvent(DAVIS240CEvent& filterEvent,int id) = 0;
};
//=====


class Filter : public DAVIS240CListener
{
public:
    Filter(int rows, int cols);
    ~Filter();

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e, int id);
    void receivedNewDAVIS240CFrame(DAVIS240CFrame& f, int id) {}
    //====
    void registerFilterListener(FilterListener* listener);
    void deregisterFilterListener(FilterListener* listener);
    void warnFilteredEvent(DAVIS240CEvent& event, int id);
    //====

private:
    // Datastructure matrix of list of events
    std::vector<std::list<DAVIS240CEvent>> m_events;
    int m_rows;
    int m_cols;

    // Parameters for flushing old events
    int m_currTime;
    int m_maxTimeToKeep;

    // Parameters for events filtering
    int m_targetPeriod;
    int m_eps;
    int m_neighborSize;
    int m_threshSupports;
    int m_threshAntiSupports;

    // List of filter listeners
    std::list<FilterListener*> m_filteredEventListeners;
};

#endif // EBV_FILTER_H
