#ifndef EBV_FILTER_H
#define EBV_FILTER_H

#include <EBV_DAVIS240C.h>

#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>

constexpr float m_pi{3.14159f};

class FilterListener
{
public:
    FilterListener(void) {}
    virtual void receivedNewFilterEvent(DAVIS240CEvent& filterEvent,
                                        const int id) = 0;
};

class Filter : public DAVIS240CEventListener
{
public:
    explicit Filter(int freq, DAVIS240C* davis = nullptr);
    ~Filter();

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const int id);

    void registerFilterListener(FilterListener* listener);
    void deregisterFilterListener(FilterListener* listener);
    void warnFilteredEvent(DAVIS240CEvent& event);

    // Setters and Getters
    int getX()     const {return m_xc;}
    int getY()     const {return m_yc;}
    int getFreq()  const {return m_frequency;}
    int getEps()   const {return m_eps;}
    float getEta() const {return m_eta;}
    void setFreq(const int freq) {m_frequency=freq;
                                  m_targetPeriod = 1e6f/(float)m_frequency;}
    void setEps(const int eps) {m_eps=eps;
                                m_epsPeriod = m_eps*m_targetPeriod/100.f;}
    void setEta(const float eta) {m_eta=eta;}

public:
    // Who filter is listening to
    DAVIS240C* m_davis;

    // Camera settings
    const int m_rows{180};
    const int m_cols{240};
    const int  m_id{-1};

    // Filter settings
    int m_frequency;
    int m_targetPeriod;
    int m_eps{10};
    int m_epsPeriod;

    // Recording settings
    bool m_record{false};
    const std::string m_eventRecordFile = "../experiments/frequencies_500_sweep"
                                          + std::to_string(m_id) + ".txt";
    std::ofstream m_recorder;

    // Parameters for flushing old events
    int m_current_t;
    int m_max_t;

    // Center of mass tracker
    int m_xc{0};
    int m_yc{0};
    float m_eta{0.01f};

    // List of filter listeners
    std::list<FilterListener*> m_filteredEventListeners;
};

class BaseFilter : public Filter
{
public:
    explicit BaseFilter(int freq, DAVIS240C* davis);
    ~BaseFilter(){}

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const int id) override;

    int getNeighborSize() const {return m_neighborSize;}
    void setNeighborSize(int size) {m_neighborSize=size;}

    int getThreshA() const {return m_threshSupportsA;}
    void setThreshA(int nbSupports) {m_threshSupportsA=nbSupports;}

    int getThreshB() const {return m_threshSupportsB;}
    void setThreshB(int nbSupports) {m_threshSupportsB=nbSupports;}

    int getThreshAnti() const {return m_threshAntiSupports;}
    void setThreshAnti(int nbAntiSupports) {m_threshAntiSupports=nbAntiSupports;}

    // Events of negative polarity are stored in matrices of lists
    std::vector<std::list<int>> m_events;

    // Parameters for events filtering
    int m_neighborSize{3};
    int m_threshSupportsA{2};
    int m_threshSupportsB{2};
    int m_threshAntiSupports{20};
};

class AdaptiveFilter : public Filter
{
public:
    AdaptiveFilter(int freq, DAVIS240C* davis);
    ~AdaptiveFilter();

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const int id) override;
    void receivedNewTransition(DAVIS240CEvent& e, const bool transition);
    void receivedNewHyperTransition(DAVIS240CEvent& e, const int dt);

    int getSigma() const { return m_sigma; }
    void setSigma(int sigma) { m_sigma=sigma; }

    int getMaxT() const {return m_max_t;}
    void setMaxT(int max_t) { m_max_t=max_t; }

    // Data structure matrix of list of events
    std::vector<std::pair<bool,int>> m_last_event;
    std::vector<std::array<int,2>> m_last_transitions;
    std::vector<DAVIS240CEvent> m_last_filtered_events;

    int m_sigma{30};
};

// AdaptiveFilter with support neighborhood thresholding
class AdaptiveFilterNeighbor : public AdaptiveFilter
{
public:
    AdaptiveFilterNeighbor(int freq, DAVIS240C* davis);
    ~AdaptiveFilterNeighbor();

    void receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                   const int id) override;
    void receivedNewTransition(DAVIS240CEvent& e, const bool transition);
    void receivedNewHyperTransition(DAVIS240CEvent& e, const int dt);

    std::vector<int> m_last_hypertransitions;
    int m_thresh_supports{4};

private:
    const std::string m_eventRecordFile = "../experiments/frequencies_500_sweep"
                                          + std::to_string(m_id) + ".txt";
    std::ofstream m_recorder;
};

#endif // EBV_FILTER_H
