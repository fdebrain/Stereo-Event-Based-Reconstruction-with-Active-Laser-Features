#include <EBV_Triangulator.h>
#include <EBV_Benchmarking.h>

void Triangulator::switchMode()
{
    m_mode = static_cast<Triangulator::StereoPair>((m_mode+1)%3);
    std::cout << "Switch triangulation mode to: " << stereoPairNames[m_mode] << "\n\r";
}

void Triangulator::resetCalibration()
{
    for (auto &k : m_K) { k = cv::Mat(3,3,CV_32FC1); }
    for (auto &d : m_D) { d = cv::Mat(1,5,CV_32FC1); }
    for (auto &r : m_R) { r = cv::Mat(3,3,CV_32FC1); }
    for (auto &t : m_T) { t = cv::Mat(3,1,CV_32FC1); }
    for (auto &f : m_F) { f = cv::Mat(3,3,CV_32FC1); }
    for (auto &q : m_Q) { q = cv::Mat(4,4,CV_32FC1); }
    for (auto &p : m_P) { p[0] = cv::Mat(3,4,CV_32FC1); p[1] = cv::Mat(3,4,CV_32FC1); }
    for (auto &rect : m_Rect) { rect[0] = cv::Mat(3,3,CV_32FC1); rect[1] = cv::Mat(3,3,CV_32FC1); }
}

void Triangulator::importCalibration()
{
    resetCalibration();
    cv::FileStorage fs;

    // Import camera calibration
    fs.open(m_path_calib_cam, cv::FileStorage::READ);
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

    // Initialize projection matrices for cam0-cam1 stereo pair
    if (isValid(0) && isValid(1))
    {
        m_enable = true;
        cv::stereoRectify(m_K[0],m_D[0],
                          m_K[1],m_D[1],
                          cv::Size(m_cols,m_rows),
                          m_R[0],m_T[0],
                          m_Rect[0][0], m_Rect[0][1],
                          m_P[0][0], m_P[0][1],
                          m_Q[0], cv::CALIB_ZERO_DISPARITY);
    }
    else
    {
        m_enable = false;
        printf("Please press C to calibrate the cameras.\n\r");
        return;
    }

    // Import laser calibration
    fs.open(m_path_calib_laser, cv::FileStorage::READ);
    fs["camera_matrix_laser"] >> m_K[2];
    fs["dist_coeffs_laser"] >> m_D[2];
    fs["R1"] >> m_R[1];
    fs["T1"] >> m_T[1];
    fs["F1"] >> m_F[1];
    fs["R2"] >> m_R[2];
    fs["T2"] >> m_T[2];
    fs["F2"] >> m_F[2];
    std::cout << "KLaser: " << m_K[2] << "\n\r";
    std::cout << "DLaser: " << m_D[2] << "\n\r";
    std::cout << "R1: " << m_R[1] << "\n\r";
    std::cout << "T1: " << m_T[1] << "\n\r";
    std::cout << "F1: " << m_F[1] << "\n\r";
    std::cout << "R2: " << m_R[2] << "\n\r";
    std::cout << "T2: " << m_T[2] << "\n\r";
    std::cout << "F2: " << m_F[2] << "\n\r";
    fs.release();

    // Initialize projection matrices for cam0-laser & cam1-laser stereo pairs
    if (isValid(0) && isValid(1) && isValid(2))
    {
        m_enable = true;

        // Camera0-laser
        cv::Mat tmpR;
        cv::Mat tmpT;
        cv::transpose(m_R[1],tmpR);
        tmpT = -tmpR*m_T[1];

        cv::stereoRectify(m_K[0],m_D[0],
                          m_K[2],m_D[2],
                          cv::Size(m_cols,m_rows),
                          m_R[1],m_T[1],
                          m_Rect[1][0], m_Rect[1][1],
                          m_P[1][0], m_P[1][1],
                          m_Q[1], cv::CALIB_ZERO_DISPARITY);
        //Camera1-laser
        cv::stereoRectify(m_K[1],m_D[1],
                          m_K[2],m_D[2],
                          cv::Size(m_cols,m_rows),
                          m_R[2],m_T[2],
                          m_Rect[2][0], m_Rect[2][1],
                          m_P[2][0], m_P[2][1],
                          m_Q[2], cv::CALIB_ZERO_DISPARITY);
    }
    else
    {
        m_enable = false;
        printf("Please press L to calibrate the laser.\n\r");
        return;
    }
}

bool Triangulator::isValid(const int id) const
{
    return m_K[id].rows==3 && m_K[id].cols==3 &&
           m_D[id].rows==1 && m_D[id].cols==5;
}

Triangulator::Triangulator(Matcher* matcher,
                           LaserController* laser)
    : m_matcher(matcher),
      m_laser(laser)
{
    // Initialize datastructures
    resetCalibration();

    // Listen to matcher
    m_matcher->registerMatcherListener(this);

    // Recording triangulation
    if (m_record){ m_recorder.open(m_eventRecordFile); }

    // Calibration
    this->importCalibration();

    if (m_enable) { m_thread = std::thread(&Triangulator::run,this); }
}

Triangulator::~Triangulator()
{
    m_matcher->deregisterMatcherListener(this);
    if (m_record){ m_recorder.close(); }
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

    //Timer timer; // 0.1ms on average

    std::vector<cv::Point2d> coords0, coords1;
    std::vector<cv::Point2d> undistCoords0, undistCoords1;
    std::vector<cv::Point2d> undistCoordsCorrected0, undistCoordsCorrected1;
    cv::Vec4d point3D;
    cv::Vec4d point3D_bis;

    m_queueAccessMutex0.lock();
    const int x0 = event0.m_x;
    const int y0 = event0.m_y;
    const int x1 = event1.m_x;
    const int y1 = event1.m_y;
    int laser_x = 180*(m_laser->getX()-500)/(4000 - 500);
    int laser_y = 240*(m_laser->getY())/(4000);
    m_queueAccessMutex0.unlock();

    //printf("xLaser: %d - yLaser: %d - xEvent0: %d - yEvent0: %d - xEvent1: %d - yEvent1: %d. \n\r",
    //       xLaser, yLaser, x0, y0, x1, y1);

    // Camera0-Camera1 stereo
    if (m_mode==Cameras)
    {
        coords0.emplace_back(y0,x0);
        coords1.emplace_back(y1,x1);

        cv::undistortPoints(coords0, undistCoords0,
                            m_K[0], m_D[0],
                            m_Rect[m_mode][0],
                            m_P[m_mode][0]);
        cv::undistortPoints(coords1, undistCoords1,
                            m_K[1], m_D[1],
                            m_Rect[m_mode][1],
                            m_P[m_mode][1]);
    }
    // Camera0-Laser stereo
    else if (m_mode==CamLeftLaser)
    {
        coords0.emplace_back(y0,x0);
        coords1.emplace_back(laser_y,laser_x);

        cv::undistortPoints(coords0, undistCoords0,
                            m_K[0], m_D[0],
                            m_Rect[m_mode][0],
                            m_P[m_mode][0]);

        cv::undistortPoints(coords1, undistCoords1,
                            m_K[2], m_D[2],
                            m_Rect[m_mode][1],
                            m_P[m_mode][1]);
    }
    // Camera1-Laser stereo
    else
    {
        coords0.emplace_back(y1,x1);
        coords1.emplace_back(laser_y,laser_x);

        cv::undistortPoints(coords0, undistCoords0,
                            m_K[1], m_D[1],
                            m_Rect[m_mode][0],
                            m_P[m_mode][0]);

        cv::undistortPoints(coords1, undistCoords1,
                            m_K[2], m_D[2],
                            m_Rect[m_mode][1],
                            m_P[m_mode][1]);
//        cv::Mat res = m_P[0]*cv::Mat(point3D);
//        std::cout << res << "\n\r";
//        point3D_bis = cv::Vec4d(res.at<double>(0,0),res.at<double>(0,1),res.at<double>(0,2),res.at<double>(0,3));
    }

    // Refine matches coordinates (using epipolar constraint)
    cv::correctMatches(m_F[m_mode],undistCoords0,undistCoords1,
                       undistCoordsCorrected0,
                       undistCoordsCorrected1);

    cv::triangulatePoints(m_P[m_mode][0], m_P[m_mode][1],
                          undistCoordsCorrected0,
                          undistCoordsCorrected1,
                          point3D);

    // Convert to cm
    double X = 100*point3D[0]/point3D[3];
    double Y = 100*point3D[1]/point3D[3];
    double Z = 100*point3D[2]/point3D[3];

//    double X2 = 100*point3D_bis[0]/point3D_bis[3];
//    double Y2 = 100*point3D_bis[1]/point3D_bis[3];
//    double Z2 = 100*point3D_bis[2]/point3D_bis[3];

    // DEBUG - CHECK 3D POINT POSITION
    if (m_debug)
    {
        printf("Point at: (%2.1f,%2.1f,%2.1f). \n\r ",X,Y,Z);
        //printf("Point at: (%2.1f,%2.1f,%2.1f) - (%2.1f,%2.1f,%2.1f).\n\r",X,Y,Z,X2,Y2,Z2);
    }
    // END DEBUG

    if (m_record)
    {
        if (m_record_pointwise==false)
        {
            recordPoint(x0,y0,x1,y1,laser_x,laser_y,X,Y,Z);
        }
        else
        {
            if (m_record_next_point==true)
            {
                recordPoint(x0,y0,x1,y1,laser_x,laser_y,X,Y,Z);
                m_record_next_point = false;
            }
        }
    }

    warnDepth(x0,y0,X,Y,Z);
}

void Triangulator::recordPoint(int x0, int y0, int x1, int y1, int x2, int y2,
                               float X, float Y, float Z)
{
    m_recorder << x0 << '\t'
               << y0 << '\t'
               << x1 << '\t'
               << y1 << '\t'
               << x2 << '\t'
               << y2 << '\t'
               << X << '\t'
               << Y << '\t'
               << Z << '\n';
}

void Triangulator::registerTriangulatorListener(TriangulatorListener* listener)
{
    m_triangulatorListeners.push_back(listener);
}

void Triangulator::deregisterTriangulatorListener(TriangulatorListener* listener)
{
    m_triangulatorListeners.remove(listener);
}

void Triangulator::warnDepth(const int u,
                             const int v,
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
