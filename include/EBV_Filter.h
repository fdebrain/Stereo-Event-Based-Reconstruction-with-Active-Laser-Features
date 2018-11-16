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
    virtual void receivedNewFilterEvent(DAVIS240CEvent& filterEvent,
                                        const unsigned int id) = 0;
};

class Filter : public DAVIS240CEventListener
{
public:
    Filter(DAVIS240C* davis = nullptr);
    ~Filter();

    void runThread();
    void process(DAVIS240CEvent& event);

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const unsigned int id) override;

    void registerFilterListener(FilterListener* listener);
    void deregisterFilterListener(FilterListener* listener);
    void warnFilteredEvent(DAVIS240CEvent& event);

    // Setters and Getters
    inline float getX() const {return m_xc;}
    inline float getY() const {return m_yc;}

    int getFreq() const {return m_frequency;}
    void setFreq(int freq) {m_frequency=freq;
                            m_targetPeriod = 1e6/m_frequency;}

    int getEps() const {return m_eps;}
    void setEps(int eps) {m_eps=eps;
                          m_epsPeriod = m_eps*m_targetPeriod/100.;}

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

private:
    // Datastructure matrix of list of events
    const int m_rows;
    const int m_cols;
    std::vector<std::list<int>> m_events;

    // Who filter listens to
    DAVIS240C* m_davis;

    // Parameters for events filtering
    int m_frequency;
    int m_targetPeriod;
    int m_eps;
    int m_epsPeriod;
    int m_neighborSize;
    int m_threshSupportsA;
    int m_threshSupportsB;
    int m_threshAntiSupports;

    // Parameters for flushing old events
    int m_currTime;
    int m_maxTimeToKeep;

    // Center of mass tracker
    float m_xc, m_yc;
    float m_eta;

    // List of incoming raw events
    std::list<DAVIS240CEvent> m_evtQueue;
    std::mutex                m_evtMutex;

    // List of filter listeners
    std::list<FilterListener*> m_filteredEventListeners;

    // Mutex to access the queue
    std::mutex m_queueAccessMutex;

    // Wait when no processing has to be done
    std::condition_variable m_condWait;
    std::mutex m_condWaitMutex;
};

#endif // EBV_FILTER_H
