#include <EBV_Triangulator.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

StereoRectificationData::StereoRectificationData() {
    for (auto &r : R) {
        r.resize(0);
    }
    for (auto &p : P) {
        p.resize(0);
    }
    Q.resize(0);
}

bool StereoRectificationData::is_valid() const {
    return R[0].rows == 3 && R[0].cols == 3 && R[1].rows == 3 &&
           R[1].cols == 3 && P[0].rows == 3 && P[0].cols == 4 &&
           P[1].rows == 3 && P[1].cols == 4 && Q.rows == 4 && Q.cols == 4;
}

void Triangulator::importCalibration(std::string path)
{
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::READ);

    cv::FileNode calibs = fs["camera_calib"];
    calibs[0]["camera_matrix"] >> m_K0;
    std::cout << "K0: " << m_K0 << std::endl;

    calibs[0]["dist_coeffs"] >> m_D0;
    std::cout << "D0: " << m_D0 << std::endl;

    calibs[1]["camera_matrix"] >> m_K1;
    std::cout << "K1: " << m_K1 << std::endl;

    calibs[1]["dist_coeffs"] >> m_D1;
    std::cout << "D1: " << m_D1 << std::endl;

    fs["R"] >> m_R;
    std::cout << "R: " << m_R << std::endl;

    fs["T"] >> m_T;
    std::cout << "T: " << m_T << std::endl;

    fs["F"] >> m_F;
    std::cout << "F: " << m_F << std::endl;

    // Compute projection matrices + stereo-rectification rotations
    cv::stereoRectify(m_K0,m_D0,
                      m_K1,m_D1,
                      cv::Size(m_rows,m_cols),
                      m_R,m_T,
                      m_rect.R[0], m_rect.R[1],
                      m_rect.P[0], m_rect.P[1],
                      m_rect.Q);
}

Triangulator::Triangulator(const unsigned int rows,
                           const unsigned int cols,
                           Matcher*  matcher)
    : m_rows(rows),
      m_cols(cols),
      m_matcher(matcher)
{
    // Initialize thread
    m_thread = std::thread(&Triangulator::run,this);

    // Listen to matcher
    m_matcher->registerMatcherListener(this);

    // Get camera settings
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
        m_queueAccessMutex0.lock();
            hasQueueEvent0  =!m_evtQueue0.empty();
        m_queueAccessMutex0.unlock();

        m_queueAccessMutex1.lock();
            hasQueueEvent1  =!m_evtQueue1.empty();
        m_queueAccessMutex1.unlock();

        // Process only if incoming filtered events in both cameras
        if(hasQueueEvent0 && hasQueueEvent1)
        {
            m_queueAccessMutex0.lock();
                event0 = m_evtQueue0.front();
                m_evtQueue0.pop_front();
            m_queueAccessMutex0.unlock();

            m_queueAccessMutex1.lock();
                event1 = m_evtQueue1.front();
                m_evtQueue1.pop_front();
            m_queueAccessMutex1.unlock();

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

void Triangulator::receivedNewMatch(const DAVIS240CEvent& event0,
                                    const DAVIS240CEvent& event1)
{
    m_queueAccessMutex0.lock();
        m_evtQueue0.push_back(event0);
    m_queueAccessMutex0.unlock();

    m_queueAccessMutex1.lock();
        m_evtQueue1.push_back(event1);
    m_queueAccessMutex1.lock();

    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

void Triangulator::process(const DAVIS240CEvent& event0, const DAVIS240CEvent& event1)
{
    // DEBUG - CHECK EVENTS POSITION
    m_queueAccessMutex0.lock();
        printf("Event0: (%d,%d). \n\r",event0.m_y,event0.m_x);
    m_queueAccessMutex0.unlock();
    m_queueAccessMutex1.lock();
        printf("Event1: (%d,%d). \n\r",event1.m_y,event1.m_x);
    m_queueAccessMutex1.unlock();


    // END DEBUG

    std::vector<cv::Point2d> coords0, coords1;
    std::vector<cv::Point2d> undistCoords0, undistCoords1;
    std::vector<cv::Point2d> undistCoordsCorrected0, undistCoordsCorrected1;
    cv::Vec4d point3D;

    m_queueAccessMutex0.lock();
        const unsigned int x0 = event0.m_x;
        const unsigned int y0 = event0.m_y;
        const unsigned int x1 = event1.m_x;
        const unsigned int y1 = event1.m_y;
    m_queueAccessMutex0.unlock();

    coords0.push_back(cv::Point2d(x0,y0));
    coords1.push_back(cv::Point2d(x1,y1));
    //printf("Event0: (%f,%f). \n\r",coords0.back().x,coords0.back().y);
    //printf("Event1: (%f,%f). \n\r",coords1.back().x,coords1.back().y);

    // Undistort 2D points
    cv::undistortPoints(coords0, undistCoords0,
                        m_K0, m_D0,
                        m_rect.R[0],
                        m_rect.P[0]);
    cv::undistortPoints(coords1, undistCoords1,
                        m_K1, m_D1,
                        m_rect.R[1],
                        m_rect.P[1]);
    //printf("Undistorted Event0: (%f,%f). \n\r",undistCoords0.back().x,undistCoords0.back().y);
    //printf("Undistorted Event1: (%f,%f). \n\r",undistCoords1.back().x,undistCoords1.back().y);
    // Correct matches
    cv::correctMatches(m_F,undistCoords0,undistCoords1,
                       undistCoordsCorrected0,
                       undistCoordsCorrected1);

    // Triangulate
    cv::triangulatePoints(m_rect.P[0], m_rect.P[1],
                          undistCoordsCorrected0,
                          undistCoordsCorrected1,
                          point3D);

    //printf("Point at: (%3.f,%3.f,%4.f).\n\r",
    //        point3D[0]/point3D[3],
    //        point3D[1]/point3D[3],
    //        point3D[2]/point3D[3]);

    //printf("Point at: (%3.f,%3.f,%4.f).\n\r",point3D.at<double>(0)/point3D.at<double>(3),
    //                                   point3D.at<double>(1)/point3D.at<double>(3),
    //                                   point3D.at<double>(2)/point3D.at<double>(3));

    double depth = point3D[2]/point3D[3];
    printf("Depth: %f.\n\r",depth);

    warnDepth(x0,y0,depth);
}

void Triangulator::registerTriangulatorListener(TriangulatorListener* listener)
{
    m_triangulatorListeners.push_back(listener);
}

void Triangulator::deregisterTriangulatorListener(TriangulatorListener* listener)
{
    m_triangulatorListeners.remove(listener);
}

void Triangulator::warnDepth(const unsigned int u,
                             const unsigned int v,
                             const double z)
{
    std::list<TriangulatorListener*>::iterator it;
    for(it = m_triangulatorListeners.begin(); it!=m_triangulatorListeners.end(); it++)
    {
        (*it)->receivedNewDepth(u,v,z);
    }
}
