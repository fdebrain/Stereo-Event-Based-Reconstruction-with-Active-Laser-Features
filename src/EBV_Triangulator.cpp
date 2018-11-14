#include <EBV_Triangulator.h>

void Triangulator::calibrateLaser()
{
    m_recorder.open(m_eventRecordFile);
}

void Triangulator::importCalibration()
{
    // Import camera calibration
    cv::FileStorage fs;
    fs.open(m_pathCalib, cv::FileStorage::READ);
    fs["camera_matrix0"] >> m_K[0];
    fs["dist_coeffs0"] >> m_D[0];
    fs["camera_matrix1"] >> m_K[1];
    fs["dist_coeffs1"] >> m_D[1];
    fs["R"] >> m_R[0];
    fs["T"] >> m_T[0];
    fs["F"] >> m_F[0];
    std::cout << "K0: " << m_K[0] << "\n\r";
    std::cout << "D0: " << m_D[0] << "\n\r";
    std::cout << "K1: " << m_K[1] << "\n\r";
    std::cout << "D1: " << m_D[1] << "\n\r";
    std::cout << "R: " << m_R[0] << "\n\r";
    std::cout << "T: " << m_T[0] << "\n\r";
    std::cout << "F: " << m_F[0] << "\n\r";
    fs.release();

    // Import laser calibration
    fs.open(m_pathCalibLaser, cv::FileStorage::READ);
    fs["camera_matrix_laser"] >> m_K[2];
    fs["dist_coeffs_laser"] >> m_D[2];
    fs["R0"] >> m_R[1];
    fs["T0"] >> m_T[1];
    fs["F0"] >> m_F[1];
    fs["R1"] >> m_R[2];
    fs["T1"] >> m_T[2];
    fs["F1"] >> m_F[2];
    std::cout << "KLaser: " << m_K[2] << "\n\r";
    std::cout << "DLaser: " << m_D[2] << "\n\r";
    std::cout << "R0: " << m_R[1] << "\n\r";
    std::cout << "T0: " << m_T[1] << "\n\r";
    std::cout << "F0: " << m_F[1] << "\n\r";
    std::cout << "R1: " << m_R[2] << "\n\r";
    std::cout << "T1: " << m_T[2] << "\n\r";
    std::cout << "F1: " << m_F[2] << "\n\r";
    fs.release();

    // Compute projection matrices + stereo-rectification rotations
    cv::stereoRectify(m_K[0],m_D[0],
                      m_K[2],m_D[2],
                      cv::Size(m_cols,m_rows),
                      m_R[1],m_T[1],
                      m_Rect[0], m_Rect[2],
                      m_P[0], m_P[2],
                      m_Q[1]);

    cv::stereoRectify(m_K[0],m_D[0],
                      m_K[1],m_D[1],
                      cv::Size(m_cols,m_rows),
                      m_R[0],m_T[0],
                      m_Rect[0], m_Rect[1],
                      m_P[0], m_P[1],
                      m_Q[0], cv::CALIB_ZERO_DISPARITY);
}

Triangulator::Triangulator(const unsigned int rows,
                           const unsigned int cols,
                           Matcher* matcher,
                           LaserController* laser)
    : m_rows(rows),
      m_cols(cols),
      m_matcher(matcher),
      m_laser(laser)
{
    // Initialize datastructures
    for (auto &k : m_K) { k = cv::Mat(3,3,CV_32FC1); }
    for (auto &d : m_D) { d = cv::Mat(1,5,CV_32FC1); }
    for (auto &p : m_P) { p = cv::Mat(3,4,CV_32FC1); }
    for (auto &r : m_R) { r = cv::Mat(3,3,CV_32FC1); }
    for (auto &t : m_T) { t = cv::Mat(3,1,CV_32FC1); }
    for (auto &f : m_F) { f = cv::Mat(1,5,CV_32FC1); }
    for (auto &rect : m_Rect) { rect.resize(0); }
    for (auto &q : m_Q) { q.resize(0); }

    // Listen to matcher
    m_matcher->registerMatcherListener(this);

    // Laser calibration
    if (m_laser->m_calibrateLaser) { calibrateLaser(); }

    // Calibration
    this->importCalibration();
    m_thread = std::thread(&Triangulator::run,this);
}

Triangulator::~Triangulator()
{
    m_matcher->deregisterMatcherListener(this);
    if (m_laser->m_calibrateLaser) { m_recorder.close(); }
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
            hasQueueEvent0 =! m_evtQueue0.empty();
        m_queueAccessMutex0.unlock();

        m_queueAccessMutex1.lock();
            hasQueueEvent1 =! m_evtQueue1.empty();
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
    m_queueAccessMutex1.unlock();

    std::unique_lock<std::mutex> condLock(m_condWaitMutex);
    m_condWait.notify_one();
}

void Triangulator::process(const DAVIS240CEvent& event0, const DAVIS240CEvent& event1)
{
    // DEBUG - CHECK EVENTS POSITION
    //m_queueAccessMutex0.lock();
    //    printf("Event0: (%d,%d). \n\r",event0.m_x,event0.m_y);
    //m_queueAccessMutex0.unlock();
    //m_queueAccessMutex1.lock();
    //    printf("Event1: (%d,%d). \n\r",event1.m_y,event1.m_x);
    //m_queueAccessMutex1.unlock();
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
        int xLaser = m_laser->getX();
        int yLaser = m_laser->getY();
    m_queueAccessMutex0.unlock();

    //printf("xLaser: %d - yLaser: %d - xEvent: %d - yEvent: %d. \n\r",
    //       xLaser,yLaser, x0, y0);

    ///*
    // Camera0-Camera1 stereo
    coords0.push_back(cv::Point2d(y0,x0)); // Warning: x = column, y = row
    coords1.push_back(cv::Point2d(y1,x1));

    // Undistort 2D points
    cv::undistortPoints(coords0, undistCoords0,
                        m_K[0], m_D[0],
                        m_Rect[0],
                        m_P[0]);
    cv::undistortPoints(coords1, undistCoords1,
                        m_K[1], m_D[1],
                        m_Rect[1],
                        m_P[1]);

    // Correct matches
    cv::correctMatches(m_F[0],undistCoords0,undistCoords1,
                       undistCoordsCorrected0,
                       undistCoordsCorrected1);

    // Triangulate
    cv::triangulatePoints(m_P[0], m_P[1],
                          undistCoordsCorrected0,
                          undistCoordsCorrected1,
                          point3D);
    //*/

    /*
    // Camera0-Laser stereo
    coords0.push_back(cv::Point2d(y0,x0)); // Warning: Point2d(column,row)
    coords1.push_back(cv::Point2d(yLaser,xLaser));

    // Undistort 2D points
    cv::undistortPoints(coords0, undistCoords0,
                        m_K0, m_D0,
                        m_rect.R[0],
                        m_rect.P[0]);

    //undistCoords1 = coords1;
    cv::undistortPoints(coords1, undistCoords1,
                        m_KLaser, m_DLaser,
                        m_rect.R[2],
                        m_rect.P[2]);

    // Correct matches
    //cv::correctMatches(m_F0,undistCoords0,undistCoords1,
    //                  undistCoordsCorrected0,
    //                  undistCoordsCorrected1);

    // Triangulate
    cv::triangulatePoints(m_rect.P[0], m_rect.P[2],
                          undistCoords0,
                          undistCoords1,
                          //undistCoordsCorrected0,
                          //undistCoordsCorrected1,
                          point3D);
    */

    double X = point3D[0]/point3D[3];
    double Y = point3D[1]/point3D[3];
    double Z = point3D[2]/point3D[3];

    // DEBUG - CHECK 3D POINT POSITION
    //printf("Point at: (%2.1f,%2.1f,%2.1f).\n\r",X,Y,Z);
    // END DEBUG

    // Save laser calibration file
    if (m_laser->m_calibrateLaser)
    {
        m_recorder << x0 << '\t'
                   << y0 << '\t'
                   << x1 << '\t'
                   << y1 << '\t'
                   << xLaser << '\t'
                   << yLaser << '\t'
                   << X << '\t'
                   << Y << '\t'
                   << Z << '\n';
    }

    warnDepth(x0,y0,X,Y,Z);
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
                             const double X,
                             const double Y,
                             const double Z)
{
    std::list<TriangulatorListener*>::iterator it;
    for(it = m_triangulatorListeners.begin(); it!=m_triangulatorListeners.end(); it++)
    {
        (*it)->receivedNewDepth(u,v,X,Y,Z);
    }
}
