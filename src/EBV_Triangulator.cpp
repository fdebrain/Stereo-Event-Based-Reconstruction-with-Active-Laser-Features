#include <EBV_Triangulator.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

void Triangulator::importCalibration(std::string path)
{
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::READ);

    cv::FileNode calibs = fs["camera_calib"];
    calibs[0]["camera_matrix"] >> m_K0;
    std::cout << m_K0 << std::endl;

    calibs[1]["camera_matrix"] >> m_K1;
    std::cout << m_K1 << std::endl;

    fs["R"] >> m_R;
    std::cout << m_R << std::endl;

    fs["T"] >> m_T;
    std::cout << m_T << std::endl;

    fs["F"] >> m_F;
    std::cout << m_F << std::endl;
}


Triangulator::Triangulator(int rows, int cols,
                           Matcher*  matcher):
    m_rows(rows), m_cols(cols),
    m_matcher(matcher)
{
    // Initialize thread
    m_thread = std::thread(&Triangulator::run,this);

    // Listen to matcher
    m_matcher->registerMatcherListener(this);

    // Initialize camera matrices
    this->importCalibration(m_pathCalib);

}

Triangulator::~Triangulator()
{
    m_matcher->deregisterMatcherListener(this);
}

void Triangulator::run()
{
    bool hasQueueEvent0;
    bool hasQueueEvent1;
    DAVIS240CEvent event0;
    DAVIS240CEvent event1;
    while(true)
    {
        m_queueAccessMutex.lock();
            hasQueueEvent0  =!m_evtQueue0.empty();
            hasQueueEvent1  =!m_evtQueue1.empty();
        m_queueAccessMutex.unlock();

        // Process only if incoming filtered events in both cameras
        if(hasQueueEvent0 && hasQueueEvent1)
        {
            m_queueAccessMutex.lock();
                event0 = m_evtQueue0.front();
                m_evtQueue0.pop_front();
                event1 = m_evtQueue1.front();
                m_evtQueue1.pop_front();
            m_queueAccessMutex.unlock();

            process(event0,event1);
        }
        else
        {
            std::unique_lock<std::mutex> condLock(m_condWaitMutex);
            m_condWait.wait(condLock);
            condLock.unlock();
        }
    }
}

void Triangulator::receivedNewMatch(DAVIS240CEvent& event0, DAVIS240CEvent& event1)
{
    printf("Received new match ! \n\r");
    m_queueAccessMutex.lock();
        m_evtQueue0.push_back(event0);
        m_evtQueue1.push_back(event1);
    m_queueAccessMutex.unlock();

    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

void Triangulator::process(DAVIS240CEvent& event0, DAVIS240CEvent& event1)
{
    cv::Point2d point0;
    point0.x = event0.m_x;
    point0.y = event0.m_y;
    cv::Point2d point1;
    point1.x = event1.m_x;
    point1.y = event1.m_y;

    cv::Mat point3D(4,1,CV_64FC4);

    //cv::triangulatePoints(m_P0,m_P1,point0,point1,point3D);
}

void Triangulator::registerTriangulatorListener(TriangulatorListener* listener)
{
    m_triangulatorListeners.push_back(listener);
}

void Triangulator::deregisterTriangulatorListener(TriangulatorListener* listener)
{
    m_triangulatorListeners.remove(listener);
}

void Triangulator::warnDepth()
{
    std::list<TriangulatorListener*>::iterator it;
    for(it = m_triangulatorListeners.begin(); it!=m_triangulatorListeners.end(); it++)
    {
        (*it)->receivedNewDepth();
    }
}
