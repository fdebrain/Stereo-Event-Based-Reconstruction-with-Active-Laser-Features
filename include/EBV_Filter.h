#ifndef EBV_FILTER_H
#define EBV_FILTER_H

#include <EBV_DAVIS240C.h>

#include <string>
#include <list>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>

class FilterListener
{
public:
    FilterListener(void) {}
    virtual void receivedNewFilterEvent(DAVIS240CEvent& filterEvent,int id) = 0;
};

class Filter : public DAVIS240CListener
{
public:
    Filter(int rows, int cols,
           DAVIS240C* davis = nullptr);
    ~Filter();

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e, int id);
    void receivedNewDAVIS240CFrame(DAVIS240CFrame& f, int id) {}

    void registerFilterListener(FilterListener* listener);
    void deregisterFilterListener(FilterListener* listener);
    void warnFilteredEvent(DAVIS240CEvent& event);

    // Setters and Getters
    float getX() const {return m_xc;}
    float getY() const {return m_yc;}

    int getFreq() const {return m_frequency;}
    void setFreq(int freq) {m_frequency=freq;}

    int getEps() const {return m_eps;}
    void setEps(int eps) {m_eps=eps;}

    int getNeighborSize() const {return m_neighborSize;}
    void setNeighborSize(int size) {m_neighborSize=size;}

    int getThreshA() const {return m_threshSupportsA;}
    void setThreshA(int nbSupports) {m_threshSupportsA=nbSupports;}

    int getThreshB() const {return m_threshSupportsB;}
    void setThreshB(int nbSupports) {m_threshSupportsB=nbSupports;}

    int getThreshAnti() const {return m_threshAntiSupports;}
    void setThreshAnti(int nbAntiSupports) {m_threshAntiSupports=nbAntiSupports;}

    float getEta() const {return m_eta;}
    void setEta(float eta) {m_eta=eta;}

//====
public:
    std::thread m_thread;
//====

private:
    // Id of the filter
    DAVIS240C* m_davis;

    // Datastructure matrix of list of events
    int m_rows;
    int m_cols;
    std::vector<std::list<DAVIS240CEvent>> m_events;

    // Parameters for flushing old events
    int m_currTime;
    int m_maxTimeToKeep;

    // Parameters for events filtering
    int m_frequency;
    int m_eps;
    int m_neighborSize;
    int m_threshSupportsA;
    int m_threshSupportsB;
    int m_threshAntiSupports;

    // Center of mass tracker
    float m_xc, m_yc;
    float m_eta;

    // List of filter listeners
    std::list<FilterListener*> m_filteredEventListeners;

    // List of incoming raw events
    std::list<DAVIS240CEvent> m_evtQueue;

    // Mutex to access the queue
    std::mutex m_queueAccessMutex;

    // Wait when no processing has to be done
    std::condition_variable m_condWait;
    std::mutex m_condWaitMutex;
};

#endif // EBV_FILTER_H
