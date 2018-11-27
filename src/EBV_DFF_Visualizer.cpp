#include <EBV_DFF_Visualizer.h>

#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//=== TRACKBAR CALLBACKS ===//
//=== Filters ===//
static void callbackTrackbarFilterFreq(int new_freq, void *data)
{
    Filter* filter = static_cast<Filter*>(data);
    filter->setFreq(new_freq);
}

static void callbackTrackbarFilterEps(int new_eps, void *data)
{
    Filter* filter = static_cast<Filter*>(data);
    filter->setEps(new_eps);
}

static void callbackTrackbarFilterSigma(int new_sigma, void *data)
{
    AdaptiveFilter* filter = static_cast<AdaptiveFilter*>(data);
    filter->setSigma(new_sigma);
}

static void callbackTrackbarNeighborSize(int size, void *data)
{
    BaseFilter* filter = static_cast<BaseFilter*>(data);
    filter->setNeighborSize(size);
}

static void callbackTrackbarThreshA(int newThreshA, void *data)
{
    BaseFilter* filter = static_cast<BaseFilter*>(data);
    filter->setThreshA(newThreshA);
}

static void callbackTrackbarThreshB(int newThreshB, void *data)
{
    BaseFilter* filter = static_cast<BaseFilter*>(data);
    filter->setThreshB(newThreshB);
}

static void callbackTrackbarThreshAnti(int newThreshAnti, void *data)
{
    BaseFilter* filter = static_cast<BaseFilter*>(data);
    filter->setThreshAnti(newThreshAnti);
}

static void callbackTrackbarEta(int newEta, void *data)
{
    Filter* filter = static_cast<Filter*>(data);
    filter->setEta(newEta/100.f);
}

//=== Laser ===//
static void callbackTrackbarLaserMinX(int min_x, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setMinX(min_x);
}

static void callbackTrackbarLaserMaxX(int max_x, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setMaxX(max_x);
}

static void callbackTrackbarLaserMinY(int min_y, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setMinY(min_y);
}

static void callbackTrackbarLaserMaxY(int max_y, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setMaxY(max_y);
}

static void callbackTrackbarLaserX(int x, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setX(x);
}

static void callbackTrackbarLaserY(int y, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setY(y);
}

static void callbackTrackbarLaserVx(int vx, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setVx(vx);
}

static void callbackTrackbarLaserVy(int vy, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setVy(vy);
}

static void callbackTrackbarLaserLearningRate(int lrInt, void *data)
{
    StereoCalibrator* calib = static_cast<StereoCalibrator*>(data);
    calib->setLearningRate(lrInt/100.f);
}

static void callbackTrackbarLaserFreq(int freq, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setFreq(freq);
}

static void callbackTrackbarLaserStep(int step, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setStep(step);
}

static void callbackTrackbarLaserRatio(int ratio, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setRatio(ratio/10.f);
}

//=== Matcher ===//
static void callbackTrackbarMatcherEps(int eps, void *data)
{
    Matcher* matcher = static_cast<Matcher*>(data);
    matcher->setEps(eps);
}

static void callbackTrackbarMatcherMaxBuffer(int maxBuffer, void *data)
{
    Matcher* matcher = static_cast<Matcher*>(data);
    matcher->setMaxBuffer(maxBuffer);
}

Visualizer::Visualizer(const uint nbCams,
                       DAVIS240C* davis0, DAVIS240C* davis1,
                       Filter* filter0, Filter* filter1,
                       StereoCalibrator* calibrator,
                       Triangulator* triangulator,
                       LaserController* laser)
    : m_rows(180), m_cols(240), m_nbCams(nbCams),
      m_ageThresh(40e3), m_max_trackbar_val(1e6),
      m_min_depth(20),m_max_depth(40),
      m_davis{davis0,davis1},
      m_laser(laser),
      m_filter{filter0,filter1},
      m_calibrator(calibrator),
      m_triangulator(triangulator)
{
    // Initialize data structure
    for (auto& v : m_polEvts) { v.resize(m_rows*m_cols,0); }
    for (auto& v : m_ageEvts) { v.resize(m_rows*m_cols,0); }
    for (auto& v : m_filtEvts) { v.resize(m_rows*m_cols,0); }
    for (auto& v : m_grayFrame) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC1); }
    m_depthMap.resize(m_rows*m_cols,0);
    m_mask = cv::Mat::zeros(m_rows,m_cols,CV_8UC1);

    for (uint idx=0; idx<2; idx++)
    {
        // Initialize davis
        if (m_davis[idx]!=nullptr)
        {
            // Initialize windows
            m_polWin[idx] = "Events by Polarity - " + std::to_string(idx);
            m_ageWin[idx] = "Events by Age - "  + std::to_string(idx);
            m_frameWin[idx] = "Frame - "  + std::to_string(idx);
            cv::namedWindow(m_polWin[idx],0);
            //cv::namedWindow(m_ageWin[idx],0);
            cv::namedWindow(m_frameWin[idx],0);

            // Age trackbars
            cv::createTrackbar("Threshold",m_polWin[idx],&m_ageThresh,m_max_trackbar_val,nullptr);

            // Listen to davis
            m_davis[idx]->init();
            m_davis[idx]->start();
            m_davis[idx]->registerEventListener(this);
            m_davis[idx]->registerFrameListener(this);
            m_davis[idx]->listen();
        }

        // Initialize filter
        if (m_filter[idx]!=nullptr)
        {
            // Listen to filter
            m_filter[idx]->registerFilterListener(this);

            // Initialize window
            m_filtWin[idx] = "Filtered Events - "  + std::to_string(idx);
            cv::namedWindow(m_filtWin[idx],0);

            // Initialize filter tuning parameters
            m_filter_freq[idx] = m_filter[idx]->getFreq();
            m_filter_eps[idx] = m_filter[idx]->getEps();
            m_filter_etaInt[idx] = static_cast<int>(100*m_filter[idx]->getEta());
            m_filter_sigma[idx] = static_cast<AdaptiveFilter*>(m_filter[idx])->getSigma();
            m_filter_max_t[idx] = static_cast<AdaptiveFilter*>(m_filter[idx])->getMaxT();

//            m_neighborSize[idx] = m_filter[idx]->getNeighborSize();
//            m_threshA[idx] = m_filter[idx]->getThreshA();
//            m_threshB[idx] = m_filter[idx]->getThreshB();
//            m_threshAnti[idx] = m_filter[idx]->getThreshAnti();

            // Filter trackbars
            cv::createTrackbar("Filter frequency",m_filtWin[idx],&m_filter_freq[idx],1500,
                               &callbackTrackbarFilterFreq,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("Epsilon",m_filtWin[idx],&m_filter_eps[idx],20,
                               &callbackTrackbarFilterEps,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("Sigma",m_filtWin[idx],&m_filter_sigma[idx],100,
                               &callbackTrackbarFilterSigma,static_cast<void*>(m_filter[idx]));
//            cv::createTrackbar("Neighbor Size",m_filtWin[idx],&m_neighborSize[idx],100,
//                               &callbackTrackbarNeighborSize,static_cast<void*>(m_filter[idx]));
//            cv::createTrackbar("SupportsA",m_filtWin[idx],&m_threshA[idx],100,
//                               &callbackTrackbarThreshA,static_cast<void*>(m_filter[idx]));
//            cv::createTrackbar("SupportsB",m_filtWin[idx],&m_threshB[idx],100,
//                               &callbackTrackbarThreshB,static_cast<void*>(m_filter[idx]));
//            cv::createTrackbar("Anti-Supports",m_filtWin[idx],&m_threshAnti[idx],100,
//                               &callbackTrackbarThreshAnti,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("Eta",m_filtWin[idx],&m_filter_etaInt[idx],100,
                               &callbackTrackbarEta,static_cast<void*>(m_filter[idx]));
        }
    }

    // Initialize laser
    if (m_laser!=nullptr)
    {
        // Initialize tuning parameters (laser)
        m_laser_pos[0] = m_laser->getX();
        m_laser_pos[1] = m_laser->getY();
        m_laser_vel[0] = m_laser->getVx();
        m_laser_vel[1] = m_laser->getVy();
        m_laser_freq = m_laser->getFreq();
        m_laser_step = m_laser->getStep();
        m_laser_ratio_int = static_cast<int>(10*m_laser->getRatio());
        m_laser_boundaries[0] = m_laser->getMinX();
        m_laser_boundaries[1] = m_laser->getMaxX();
        m_laser_boundaries[2] = m_laser->getMinY();
        m_laser_boundaries[3] = m_laser->getMaxY();

        // Laser trackbars
        cv::createTrackbar("Laser frequency",m_polWin[0],&m_laser_freq,1500,
                           &callbackTrackbarLaserFreq,static_cast<void*>(m_laser));

        cv::createTrackbar("laserMinX",m_polWin[0],&m_laser_boundaries[0],4000,
                           &callbackTrackbarLaserMinX,static_cast<void*>(m_laser));
        cv::createTrackbar("laserMaxX",m_polWin[0],&m_laser_boundaries[1],4000,
                           &callbackTrackbarLaserMaxX,static_cast<void*>(m_laser));
        cv::createTrackbar("laserMinY",m_polWin[0],&m_laser_boundaries[2],4000,
                           &callbackTrackbarLaserMinY,static_cast<void*>(m_laser));
        cv::createTrackbar("laserMaxY",m_polWin[0],&m_laser_boundaries[3],4000,
                           &callbackTrackbarLaserMaxY,static_cast<void*>(m_laser));

        cv::createTrackbar("laserX",m_polWin[0],&m_laser_pos[0],m_laser->m_max_x,
                           &callbackTrackbarLaserX,static_cast<void*>(m_laser));
        cv::createTrackbar("laserY",m_polWin[0],&m_laser_pos[1],m_laser->m_max_y,
                           &callbackTrackbarLaserY,static_cast<void*>(m_laser));
        cv::createTrackbar("laserVx",m_polWin[0],&m_laser_vel[0],1e6,
                           &callbackTrackbarLaserVx,static_cast<void*>(m_laser));
        cv::createTrackbar("laserVy",m_polWin[0],&m_laser_vel[1],1e6,
                           &callbackTrackbarLaserVy,static_cast<void*>(m_laser));
        cv::createTrackbar("Laser step",m_polWin[0],&m_laser_step,300,
                           &callbackTrackbarLaserStep,static_cast<void*>(m_laser));
        cv::createTrackbar("Laser ratio",m_polWin[0],&m_laser_ratio_int,100,
                           &callbackTrackbarLaserRatio,static_cast<void*>(m_laser));
    }

    // Initialize triangulator
    if (m_triangulator!=nullptr)
    {
        // Listen to triangulator
        m_triangulator->registerTriangulatorListener(this);

        // Initialize window
        m_depthWin = "Depth Map";
        cv::namedWindow(m_depthWin,0);

        m_depthInpaintedWin = "Depth Map Inpainted";
        cv::namedWindow(m_depthInpaintedWin,0);

        // Initialize matcher parameters
        m_matcherEps = m_triangulator->m_matcher->getEps();
        m_matcherMaxBuffer = m_triangulator->m_matcher->getMaxBuffer();

        // Matcher trackbars
        cv::createTrackbar("Matcher eps",m_depthWin,&m_matcherEps,1e5,
                           &callbackTrackbarMatcherEps,static_cast<void*>(m_triangulator->m_matcher));
        cv::createTrackbar("Matcher max buffer",m_depthWin,&m_matcherMaxBuffer,1e5,
                           &callbackTrackbarMatcherMaxBuffer,static_cast<void*>(m_triangulator->m_matcher));

        // Depth trackbars
        cv::createTrackbar("minDepth",m_depthWin,&m_min_depth,100,nullptr);
        cv::createTrackbar("maxDepth",m_depthWin,&m_max_depth,100,nullptr);
    }

    // Initialize calibrator
    if (m_calibrator!=nullptr)
    {
        m_lrInt = static_cast<int>(100*m_calibrator->getLearningRate());
        cv::createTrackbar("LearningRate",m_polWin[0],&m_lrInt,100,
                           &callbackTrackbarLaserLearningRate,static_cast<void*>(m_calibrator));
    }
}


Visualizer::~Visualizer()
{
    for (uint idx=0; idx<2; idx++)
    {
        if (m_davis[idx]!=nullptr)
        {
            m_davis[idx]->stopListening();
            m_davis[idx]->deregisterEventListener(this);
            m_davis[idx]->deregisterFrameListener(this);
            m_davis[idx]->stop();
        }

        if (m_filter[idx]!=nullptr)
        {
            m_filter[idx]->deregisterFilterListener(this);
        }
    }

    if (m_triangulator!=nullptr)
    {
        m_triangulator->deregisterTriangulatorListener(this);
    }

    if (m_laser != nullptr)
    {
        m_laser->stop();
    }
}

// Runs in DAVIS240 thread
void Visualizer::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                           const uint id)
{
    const int x = e.m_x;
    const int y = e.m_y;
    const bool p = e.m_pol; // p={0,1}
    const int t = e.m_timestamp;
    m_currenTime[id] = t;
    m_polEvts[id][x*m_cols+y] = p;
    m_ageEvts[id][x*m_cols+y] = t;
}

// Runs in DAVIS240 thread
void Visualizer::receivedNewDAVIS240CFrame(DAVIS240CFrame& f,
                                           const uint id)
{
    auto frame = f.m_frame;
    const int t = f.m_timestamp;
    m_grayFrame[id] = cv::Mat(m_rows,m_cols,CV_8UC1,frame.data());
    m_currenTime[id] = t;
}

// Runs in Filter thread
void Visualizer::receivedNewFilterEvent(DAVIS240CEvent& e,
                                        const uint id)
{
    const int x = e.m_x;
    const int y = e.m_y;
    const int t = e.m_timestamp;
    m_filtEvts[id][x*m_cols+y] = t;
}

void Visualizer::receivedNewDepth(const int &u,
                                  const int &v,
                                  const double &X,
                                  const double &Y,
                                  const double &Z)
{
    // Update depth value
    const float depth = Z;
    const float last_depth = m_depthMap[u*m_cols+v];
    if (last_depth>0)
    {
        m_depthMap[u*m_cols+v] = 0.5*(last_depth + depth);
    }
    else
    {
        m_depthMap[u*m_cols+v] = depth;
    }
}

void Visualizer::run()
{   
    // Wait (in ms)
    char key = ' ';

    // Initialize display matrices
    std::array<cv::Mat,2> polMat;
    std::array<cv::Mat,2> filtMat;
    std::array<cv::Mat,2> ageMatHSV;
    std::array<cv::Mat,2> ageMatRGB;
    cv::Mat depthMatHSV = cv::Mat(m_rows,m_cols,CV_8UC3);
    cv::Mat depthMatRGB = cv::Mat(m_rows,m_cols,CV_8UC3);
    cv::Mat depthMatRGBInpainted = cv::Mat(m_rows,m_cols,CV_8UC3);
    for (auto& v : polMat) { v = cv::Mat(m_rows,m_cols,CV_8UC3); }
    for (auto& v : filtMat) { v = cv::Mat(m_rows,m_cols,CV_8UC3); }
    for (auto& v : ageMatHSV) { v = cv::Mat(m_rows,m_cols,CV_8UC3); }
    for (auto& v : ageMatRGB) { v = cv::Mat(m_rows,m_cols,CV_8UC3); }

    while(key != 'q')
    {
        for(uint i=0; i<m_rows*m_cols; i++)
        {
            for (uint idx=0; idx<2; idx++)
            {
                // Events by polarity - Master
                // QUESTION: Where do we need mutex lock ?
                bool pol = m_polEvts[idx][i];
                int dt = m_currenTime[idx] - m_ageEvts[idx][i];

                if(dt<m_ageThresh)
                {
                    polMat[idx].data[3*i + 0] = static_cast<uchar>(0);              // Blue channel
                    polMat[idx].data[3*i + 1] = static_cast<uchar>(pol>0?255:0);    // Green channel
                    polMat[idx].data[3*i + 2] = static_cast<uchar>(pol==0?255:0);    // Red Channel
                }
                // Events by age
                else { dt = m_ageThresh; }
                ageMatHSV[idx].data[3*i + 0] = static_cast<uchar>(0.75*180*dt/float(m_ageThresh)); //H
                ageMatHSV[idx].data[3*i + 1] = static_cast<uchar>(255); //S
                ageMatHSV[idx].data[3*i + 2] = static_cast<uchar>(dt==m_ageThresh?0:255); //V

                // Events by polarity
                pol = m_polEvts[idx][i];
                dt = m_currenTime[idx] - m_ageEvts[idx][i];

                // Filtered events
                dt = m_currenTime[idx] - m_filtEvts[idx][i];

                if(dt < m_ageThresh)
                {
                    filtMat[idx].data[3*i + 0] = static_cast<uchar>(0);
                    filtMat[idx].data[3*i + 1] = static_cast<uchar>(0);
                    filtMat[idx].data[3*i + 2] = static_cast<uchar>(255);  // why not change color given laser frequency?
                }
                else
                {
                    filtMat[idx].data[3*i + 0] = static_cast<uchar>(0);
                    filtMat[idx].data[3*i + 1] = static_cast<uchar>(0);
                    filtMat[idx].data[3*i + 2] = static_cast<uchar>(0);
                }
            }

            // Depth map
            float z = m_depthMap[i];
            if (z>0)
            {
                if (z<m_min_depth){ z = m_min_depth; }
                else if (z>m_max_depth){ z = m_max_depth; }
                depthMatHSV.data[3*i + 0] = static_cast<uchar>(0.8*180.*(z-m_min_depth)/(m_max_depth-m_min_depth));
                depthMatHSV.data[3*i + 1] = static_cast<uchar>(255);
                depthMatHSV.data[3*i + 2] = static_cast<uchar>(255);
                m_mask.data[i] = static_cast<uint8_t>(0);
            }
            else
            {
                depthMatHSV.data[3*i + 0] = static_cast<uchar>(0.8*180.*(z-m_min_depth)/(m_max_depth-m_min_depth));
                depthMatHSV.data[3*i + 1] = static_cast<uchar>(255);
                depthMatHSV.data[3*i + 2] = static_cast<uchar>(0);
                m_mask.data[i] = static_cast<uint8_t>(255);
            }
        }

        for (unsigned int idx=0; idx<2; idx++)
        {
            if (m_davis[idx] != nullptr)
            {
                // Display events by polarity
                cv::imshow(m_polWin[idx],polMat[idx]);

                // Display events by age (timestamp)
                //cv::cvtColor(ageMatHSV[idx],ageMatRGB[idx],CV_HSV2BGR);
                //cv::imshow(m_ageWin[idx],ageMatRGB[idx]);

                // Check if calibration mode
                if (m_calibrator!=nullptr)
                {
                    if (m_calibrator->m_calibrate_cameras)
                    {
                        //cv::equalizeHist(m_grayFrame[idx],m_grayFrame[idx]);
                        m_calibrator->calibrateCameras(m_grayFrame[idx],idx);
                    }
                }

                // Display frame
                cv::imshow(m_frameWin[idx],m_grayFrame[idx]);
            }

            // Display trackers
            if(m_filter[idx] != nullptr)
            {
                int x = m_filter[idx]->getX();
                int y = m_filter[idx]->getY();
                cv::circle(filtMat[idx],cv::Point2i(y,x),
                           3,cv::Scalar(0,255,0));

                // Display filtered events + trackers
                cv::imshow(m_filtWin[idx],filtMat[idx]);
            }
        }

        if (m_triangulator != nullptr)
        {
            // Display depth map
            cv::cvtColor(depthMatHSV,depthMatRGB,CV_HSV2BGR);
            cv::imshow(m_depthWin,depthMatRGB);
        }

        key = cv::waitKey(10);
        switch (key)
        {
        case 'r':
            printf("Reset depth map.\n\r");
            m_depthMap.clear();
            m_depthMap.resize(m_rows*m_cols,0);
            break;

        // Calibrate cameras
        case 'c':
            if (m_calibrator)
            {
                printf("Starting camera calibration. \n\r");
                m_calibrator->m_calibrate_cameras = true;
            }
            break;

        // Calibrate laser
        case 'l':
            if (m_calibrator && m_laser)
            {
                if(!m_laser->m_laser_on) { m_laser->start(); }
                //m_laser->setPos(m_laser->m_max_x/2,m_laser->m_max_y/2);
                printf("Starting laser calibration. \n\r");
                m_calibrator->m_calibrate_laser = true;
                m_calibrator->calibrateLaser(m_grayFrame[0],
                                             m_grayFrame[1],0);
            }
            break;

        // Toogle laser state (on/off)
        case 'o':
            if (m_laser) { m_laser->toogleState(); }
            break;

        // Toogle laser swipe mode (on/off)
        case 's':
            if (m_laser) { m_laser->toogleSweep(); }
            break;

        case 'i':
            // Inpainting
            printf("Inpainting. \n\r");
            cv::inpaint(depthMatRGB,m_mask,depthMatRGBInpainted,2,cv::INPAINT_TELEA);
            cv::imshow(m_depthInpaintedWin,depthMatRGBInpainted);
            break;

        case 't':
            // Change triangulation mode
            m_triangulator->m_camera_stereo = !m_triangulator->m_camera_stereo;
            m_triangulator->importCalibration();
            printf("Switch triangulation mode. \n\r");
            break;

         case 'd':
            printf("Switch debug mode. \n\r");
            m_triangulator->m_debug = !m_triangulator->m_debug;
            break;
        }

        // Reset the display matrices
        for (auto& v : polMat) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC3); }
        for (auto& v : filtMat) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC3); }
        for (auto& v : ageMatHSV) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC3); }
    }
}
