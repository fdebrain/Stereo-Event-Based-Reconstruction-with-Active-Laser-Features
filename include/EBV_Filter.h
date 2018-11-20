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
                                        const uint id) = 0;
};

class Filter : public DAVIS240CEventListener
{
public:
    explicit Filter(DAVIS240C* davis = nullptr);
    ~Filter();

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const unsigned int id);

    void registerFilterListener(FilterListener* listener);
    void deregisterFilterListener(FilterListener* listener);
    void warnFilteredEvent(DAVIS240CEvent& event);

    // Setters and Getters
    int getX() const {return m_xc;}
    int getY() const {return m_yc;}

    int getFreq() const {return m_frequency;}
    void setFreq(const int freq) {m_frequency=freq;
                                  m_targetPeriod = 1e6f/(float)m_frequency;}

    int getEps() const {return m_eps;}
    void setEps(const int eps) {m_eps=eps;
                                m_epsPeriod = m_eps*m_targetPeriod/100.f;}

    float getEta() const {return m_eta;}
    void setEta(const float eta) {m_eta=eta;}

    // Who filter listens to
    DAVIS240C* m_davis;

    // Datastructure matrix of list of events
    const int m_rows;
    const int m_cols;

    // Parameters for events filtering
    int m_frequency;
    int m_targetPeriod;
    int m_eps;
    int m_epsPeriod;

    // Parameters for flushing old events
    int m_current_t;
    int m_max_t;

    // Center of mass tracker
    int m_xc, m_yc;
    float m_eta;

    // List of filter listeners
    std::list<FilterListener*> m_filteredEventListeners;
};

class BaseFilter : public Filter
{
public:
    explicit BaseFilter(DAVIS240C* davis);
    ~BaseFilter(){}

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const unsigned int id) override;

    int getNeighborSize() const {return m_neighborSize;}
    void setNeighborSize(int size) {m_neighborSize=size;}

    int getThreshA() const {return m_threshSupportsA;}
    void setThreshA(int nbSupports) {m_threshSupportsA=nbSupports;}

    int getThreshB() const {return m_threshSupportsB;}
    void setThreshB(int nbSupports) {m_threshSupportsB=nbSupports;}

    int getThreshAnti() const {return m_threshAntiSupports;}
    void setThreshAnti(int nbAntiSupports) {m_threshAntiSupports=nbAntiSupports;}

    std::vector<std::list<int>> m_events;

    // Parameters for events filtering
    int m_neighborSize;
    int m_threshSupportsA;
    int m_threshSupportsB;
    int m_threshAntiSupports;
};

class AdaptiveFilter : public Filter
{
public:
    AdaptiveFilter(DAVIS240C* davis);
    ~AdaptiveFilter(){}

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const unsigned int id) override;
    void receivedNewTransition(DAVIS240CEvent& e, const bool transition);
    void receivedNewHyperTransition(DAVIS240CEvent& e, const int dt);

    int getSigma() const {return m_sigma;}
    void setSigma(int sigma) {m_sigma=sigma;}

    int getMaxT() const {return m_max_t;}
    void setMaxT(int max_t) {m_max_t=max_t;}

    int m_sigma;
    int m_neighbor_radius;
    const float m_pi{3.14159f};

    // Data structure matrix of list of events
    std::vector<std::pair<bool,int>> m_last_event;
    std::vector<std::array<int,2>> m_last_transitions;
    std::vector<DAVIS240CEvent> m_last_filtered_events;
};

#endif // EBV_FILTER_H
