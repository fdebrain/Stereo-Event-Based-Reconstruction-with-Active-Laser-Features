#include <EBV_Triangulator.h>

StereoRectificationData::StereoRectificationData()
{
    for (auto &r : R) { r.resize(0); }
    for (auto &p : P) { p.resize(0); }
    Q.resize(0);
}

bool StereoRectificationData::is_valid() const
{
    return R[0].rows == 3 && R[0].cols == 3 && R[1].rows == 3 &&
           R[1].cols == 3 && P[0].rows == 3 && P[0].cols == 4 &&
           P[1].rows == 3 && P[1].cols == 4 && Q.rows == 4 && Q.cols == 4;
}

void Triangulator::importCalibration()
{
    // Import camera calibration
    cv::FileStorage fs;
    fs.open(m_pathCalib, cv::FileStorage::READ);
    fs["camera_matrix0"] >> m_K0;
    fs["dist_coeffs0"] >> m_D0;
    fs["camera_matrix1"] >> m_K1;
    fs["dist_coeffs1"] >> m_D1;
    fs["R"] >> m_R;
    fs["T"] >> m_T;
    fs["F"] >> m_F;
    std::cout << "K0: " << m_K0 << "\n\r";
    std::cout << "D0: " << m_D0 << "\n\r";
    std::cout << "K1: " << m_K1 << "\n\r";
    std::cout << "D1: " << m_D1 << "\n\r";
    std::cout << "R: " << m_R << "\n\r";
    std::cout << "T: " << m_T << "\n\r";
    std::cout << "F: " << m_F << "\n\r";
    fs.release();

    // Import laser calibration
    fs.open(m_pathCalibLaser, cv::FileStorage::READ);
    fs["camera_matrix_laser"] >> m_KLaser;
    fs["dist_coeffs_laser"] >> m_DLaser;
    fs["R0"] >> m_R0;
    fs["T0"] >> m_T0;
    fs["F0"] >> m_F0;
    fs["R1"] >> m_R1;
    fs["T1"] >> m_T1;
    fs["F1"] >> m_F1;
    std::cout << "KLaser: " << m_KLaser << "\n\r";
    std::cout << "DLaser: " << m_DLaser << "\n\r";
    std::cout << "R0: " << m_R0 << "\n\r";
    std::cout << "T0: " << m_T0 << "\n\r";
    std::cout << "F0: " << m_F0 << "\n\r";
    std::cout << "R1: " << m_R1 << "\n\r";
    std::cout << "T1: " << m_T1 << "\n\r";
    std::cout << "F1: " << m_F1 << "\n\r";
    fs.release();

    // Compute projection matrices + stereo-rectification rotations
    cv::stereoRectify(m_K0,m_D0,
                      m_KLaser,m_DLaser,
                      cv::Size(m_rows,m_cols),
                      m_R0,m_T0,
                      m_rect.R[0], m_rect.R[2],
                      m_rect.P[0], m_rect.P[2],
                      m_rect.Q);

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
                           DAVIS240C* davis0,
                           DAVIS240C* davis1,
                           Matcher* matcher,
                           LaserController* laser)
    : m_rows(rows),
      m_cols(cols),
      m_davis0(davis0),
      m_davis1(davis1),
      m_matcher(matcher),
      m_laser(laser),
      m_calibrateCamera(false)
{
    // Listen to matcher
    m_matcher->registerMatcherListener(this);

    // Laser calibration
    if (m_laser->m_calibrateLaser)
    {
        m_recorder.open(m_eventRecordFile);
        //m_laser->setCalibrationMode(true); // Doesn't work -> to debug
    }

    // Camera calibration
    if (m_calibrateCamera)
    {
        m_video0 = cv::VideoWriter(m_frameRecordFile0,
                CV_FOURCC('M', 'J', 'P', 'G'),
                10, cv::Size(240,180),true);
        m_video1 = cv::VideoWriter(m_frameRecordFile1,
                CV_FOURCC('M', 'J', 'P', 'G'),
                10, cv::Size(240,180),true);
        m_davis0->registerFrameListener(this);
        m_davis1->registerFrameListener(this);
    }
    else
    {
        this->importCalibration();
        m_thread = std::thread(&Triangulator::run,this);
    }
}

Triangulator::~Triangulator()
{
    m_matcher->deregisterMatcherListener(this);

    if (m_calibrateCamera)
    {
        m_video0.release();
        m_video1.release();
        m_davis0->deregisterFrameListener(this);
        m_davis1->deregisterFrameListener(this);
    }

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

// Runs in DAVIS240 thread
void Triangulator::receivedNewDAVIS240CFrame(DAVIS240CFrame& f,
                                           const unsigned int id)
{
    switch(id)
    {
        case 0:
        {
            //m_frameMutex0.lock();
                cv::Mat m_grayFrame0 = cv::Mat(m_rows,m_cols,CV_8UC1,f.m_frame.data());
                if (m_calibrateCamera)
                {
                    cv::Mat frame0;
                    cv::cvtColor(m_grayFrame0,frame0,cv::COLOR_GRAY2BGR);
                    m_video0.write(frame0);
                }
            //m_frameMutex0.unlock();
            break;
        }

        case 1:
        {
            //m_frameMutex1.lock();
                cv::Mat m_grayFrame1 = cv::Mat(m_rows,m_cols,CV_8UC1,f.m_frame.data());
                if (m_calibrateCamera)
                {
                    cv::Mat frame1;
                    cv::cvtColor(m_grayFrame1,frame1,cv::COLOR_GRAY2BGR);
                    m_video1.write(frame1);
                }
            //m_frameMutex1.unlock();
            break;
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

    // Remapping laser points
    xLaser = static_cast<int>(180.f*(float(xLaser)-1000.f)/(2000.f));
    yLaser = static_cast<int>(240.f*(float(yLaser)-500.f)/(2500.f));
    //printf("xLaser: %d - yLaser: %d. \n\r",xLaser,yLaser);

    // Camera0-Camera1 stereo
    /*
    coords0.push_back(cv::Point2d(y0,x0)); // Warning: x = column, y = row
    coords1.push_back(cv::Point2d(y1,x1));

    // Undistort 2D points
    cv::undistortPoints(coords0, undistCoords0,
                        m_K0, m_D0,
                        m_rect.R[0],
                        m_rect.P[0]);
    cv::undistortPoints(coords1, undistCoords1,
                        m_K1, m_D1,
                        m_rect.R[1],
                        m_rect.P[1]);

    // Correct matches
    cv::correctMatches(m_F,undistCoords0,undistCoords1,
                       undistCoordsCorrected0,
                       undistCoordsCorrected1);

    // Triangulate
    cv::triangulatePoints(m_rect.P[0], m_rect.P[1],
                          undistCoordsCorrected0,
                          undistCoordsCorrected1,
                          point3D);
    */

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
    cv::correctMatches(m_F0,undistCoords0,undistCoords1,
                      undistCoordsCorrected0,
                      undistCoordsCorrected1);

    // Triangulate
    cv::triangulatePoints(m_rect.P[0], m_rect.P[2],
                          undistCoords0,
                          undistCoords1,
                          point3D);

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
