#include <EBV_DFF_Visualizer.h>

#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//=== RECORDINGS ===//
constexpr bool recordEvents(false);
constexpr char eventRecordFile[] = "../calibration/recordedEvents.txt";

//=== TRACKBAR CALLBACKS ===//
static void callbackTrackbarFreq(int newFreq, void *data)
{
    Filter* filter = static_cast<Filter*>(data);
    filter->setFreq(newFreq);
}

static void callbackTrackbarEps(int newEps, void *data)
{
    Filter* filter = static_cast<Filter*>(data);
    filter->setEps(newEps);
}

static void callbackTrackbarNeighborSize(int size, void *data)
{
    Filter* filter = static_cast<Filter*>(data);
    filter->setNeighborSize(size);
}

static void callbackTrackbarThreshA(int newThreshA, void *data)
{
    Filter* filter = static_cast<Filter*>(data);
    filter->setThreshA(newThreshA);
}

static void callbackTrackbarThreshB(int newThreshB, void *data)
{
    Filter* filter = static_cast<Filter*>(data);
    filter->setThreshB(newThreshB);
}

static void callbackTrackbarThreshAnti(int newThreshAnti, void *data)
{
    Filter* filter = static_cast<Filter*>(data);
    filter->setThreshAnti(newThreshAnti);
}

static void callbackTrackbarEta(int newEta, void *data)
{
    Filter* filter = static_cast<Filter*>(data);
    filter->setEta(newEta/100.f);
}

static void callbackTrackbarLaserCenterX(int cx, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setCenterX(cx);
}

static void callbackTrackbarLaserCenterY(int cy, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setCenterY(cy);
}

static void callbackTrackbarLaserRadius(int r, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setRadius(r);
}

static void callbackTrackbarLaserStep(int stepInt, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setStep(1./stepInt);
}

static void callbackTrackbarLaserFreq(int freq, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setFreq(freq);
}

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
                       Matcher* matcher,
                       Triangulator* triangulator,
                       LaserController* laser)
    : m_rows(180), m_cols(240), m_nbCams(nbCams),
      m_ageThresh(40e3),
      m_max_trackbar_val(1e6),
      m_min_depth(20),
      m_max_depth(40),
      m_davis{davis0,davis1},
      m_filter{filter0,filter1},
      m_laser(laser),
      m_matcher(matcher),
      m_triangulator(triangulator),
      m_calibrator(calibrator)
{
    // Initialize data structure
    for (auto& v : m_polEvts) { v.resize(m_rows*m_cols); }
    for (auto& v : m_ageEvts) { v.resize(m_rows*m_cols); }
    for (auto& v : m_filtEvts) { v.resize(m_rows*m_cols); }
    for (auto& v : m_grayFrame) { v = cv::Mat::zeros(m_rows,m_cols,CV_16UC1); }
    m_depthMap.resize(m_rows*m_cols);

    for (uint idx=0; idx<nbCams; idx++)
    {
        // Initialize davis
        if (m_davis[idx]!=nullptr)
        {
            // Initialize windows
            m_polWin[idx] = "Events by Polarity - " + std::to_string(idx);
            cv::namedWindow(m_polWin[idx],0);

            m_frameWin[idx] = "Frame - " + std::to_string(idx);
            cv::namedWindow(m_frameWin[idx],0);

            m_ageWin[idx] = "Events by Age - " + std::to_string(idx);
            //cv::namedWindow(m_ageWin0,0);

            // Davis trackbars
            cv::createTrackbar("Threshold",m_polWin[idx],
                               &m_ageThresh,
                               m_max_trackbar_val,
                               nullptr);

            // Listen to davis
            m_davis[idx]->init();
            printf("Hello %d",idx);
            m_davis[idx]->start();
            m_davis[idx]->registerEventListener(this);
            m_davis[idx]->registerFrameListener(this);
            m_davis[idx]->listen();
        }

        // Initialize filters
        if (m_filter[idx]!=nullptr)
        {
            // Initialize window
            m_filtWin[idx] = "Filtered Events - " + std::to_string(idx);
            cv::namedWindow(m_filtWin[idx],0);

            // Initialize filter tuning parameters
            m_freq[idx] = m_filter[idx]->getFreq();
            m_eps[idx] = m_filter[idx]->getEps();
            m_neighborSize[idx] = m_filter[idx]->getNeighborSize();
            m_threshA[idx] = m_filter[idx]->getThreshA();
            m_threshB[idx] = m_filter[idx]->getThreshB();
            m_threshAnti[idx] = m_filter[idx]->getThreshAnti();
            m_etaInt[idx] = static_cast<int>(100*m_filter[idx]->getEta());

            // Filter trackbars
            cv::createTrackbar("Filter frequency",m_filtWin[idx],&m_freq[idx],1000,
                               &callbackTrackbarFreq,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("Epsilon",m_filtWin[idx],&m_eps[idx],20,
                               &callbackTrackbarEps,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("Neighbor Size",m_filtWin[idx],&m_neighborSize[idx],100,
                               &callbackTrackbarNeighborSize,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("SupportsA",m_filtWin[idx],&m_threshA[idx],100,
                               &callbackTrackbarThreshA,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("SupportsB",m_filtWin[idx],&m_threshB[idx],100,
                               &callbackTrackbarThreshB,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("Anti-Supports",m_filtWin[idx],&m_threshAnti[idx],100,
                               &callbackTrackbarThreshAnti,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("Eta",m_filtWin[idx],&m_etaInt[idx],100,
                               &callbackTrackbarEta,static_cast<void*>(m_filter[idx]));

            // Listen to filter
            m_filter[idx]->registerFilterListener(this);
        }
    }

    // Initialize triangulator
    if (m_triangulator!=nullptr)
    {
        // Initialize window
        m_depthWin = "Depth Map";
        cv::namedWindow(m_depthWin,0);

        // Initialize matcher parameters
        m_matcherEps = m_matcher->getEps();
        m_matcherMaxBuffer = m_matcher->getMaxBuffer();

        // Matcher trackbars
        cv::createTrackbar("Matcher eps",m_depthWin,&m_matcherEps,1e5,
                           &callbackTrackbarMatcherEps,static_cast<void*>(m_matcher));
        cv::createTrackbar("Matcher max buffer",m_depthWin,&m_matcherMaxBuffer,1e5,
                           &callbackTrackbarMatcherMaxBuffer,static_cast<void*>(m_matcher));

        // Depth trackbars
        cv::createTrackbar("minDepth",m_depthWin,&m_min_depth,100,nullptr);
        cv::createTrackbar("maxDepth",m_depthWin,&m_max_depth,100,nullptr);

        // Listen to triangulator
        m_triangulator->registerTriangulatorListener(this);
    }

    // Initialize laser
    if (m_laser!=nullptr)
    {
        m_laser->start();

        // Initialize tuning parameters (laser)
        m_cx = m_laser->getCenterX();
        m_cy = m_laser->getCenterY();
        m_r = m_laser->getRadius();
        m_stepInt = static_cast<int>(1./m_laser->getStep());
        m_laserFreq = m_laser->getFreq();

        // Laser trackbars
        cv::createTrackbar("c_x",m_polWin[0],&m_cx,4096,
                           &callbackTrackbarLaserCenterX,static_cast<void*>(m_laser));
        cv::createTrackbar("c_y",m_polWin[0],&m_cy,4096,
                           &callbackTrackbarLaserCenterY,static_cast<void*>(m_laser));
        cv::createTrackbar("radius",m_polWin[0],&m_r,2048,
                           &callbackTrackbarLaserRadius,static_cast<void*>(m_laser));
        cv::createTrackbar("step",m_polWin[0],&m_stepInt,1000,
                           &callbackTrackbarLaserStep,static_cast<void*>(m_laser));
        cv::createTrackbar("Laser frequency",m_polWin[0],&m_laserFreq,1000,
                           &callbackTrackbarLaserFreq,static_cast<void*>(m_laser));
    }

    // Saving events in .txt
    if (recordEvents)
    {
        m_recorder.open(eventRecordFile);
    }
}

Visualizer::~Visualizer()
{
    for (uint idx=0; idx<m_nbCams; idx++)
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

    if (recordEvents) { m_recorder.close(); }
}

// Runs in DAVIS240C thread
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

// Runs in DAVIS240C thread
void Visualizer::receivedNewDAVIS240CFrame(DAVIS240CFrame& f,
                                           const uint id)
{
    auto m_grayFrameData = f.m_frame;
    const int t = f.m_timestamp;
    m_grayFrame[id] = cv::Mat(m_rows,m_cols,CV_16UC1,m_grayFrameData.data);
    m_currenTime[id] = t;
}

void Visualizer::receivedNewFilterEvent(DAVIS240CEvent& e,
                                        const uint id)
{
    const int x = e.m_x;
    const int y = e.m_y;
    const int t = e.m_timestamp;

    m_filtEvts[id][x*m_cols+y] = t;
    if (recordEvents)
    {
        m_recorder << id << "\t" << x << "\t"
                   << y << "\t" << t << std::endl;
    }
}

void Visualizer::receivedNewDepth(const int &u,
                                  const int &v,
                                  const double &X,
                                  const double &Y,
                                  const double &Z)
{
    double depth = Z;
    m_depthMap[u*m_cols+v] = depth;
}

void Visualizer::run()
{   
    char key = ' ';

    // Initialize display matrices
    std::array<cv::Mat,2> polMat;
    std::array<cv::Mat,2> filtMat;
    std::array<cv::Mat,2> ageMatHSV;
    std::array<cv::Mat,2> ageMatRGB;
    cv::Mat depthMatHSV = cv::Mat(m_rows,m_cols,CV_8UC3);
    cv::Mat depthMatRGB = cv::Mat(m_rows,m_cols,CV_8UC3);

    for (auto& v : polMat) { v = cv::Mat(m_rows,m_cols,CV_8UC3); }
    for (auto& v : filtMat) { v = cv::Mat(m_rows,m_cols,CV_8UC3); }
    for (auto& v : ageMatHSV) { v = cv::Mat(m_rows,m_cols,CV_8UC3); }
    for (auto& v : ageMatRGB) { v = cv::Mat(m_rows,m_cols,CV_8UC3); }

    while(key != 'q')
    {
        for(size_t i=0; i<m_rows*m_cols; i++)
        {
            for (uint idx=0; idx<m_nbCams; idx++)
            {
                // Events by polarity
                int pol = m_polEvts[idx][i];
                int dt = m_currenTime[idx] - m_ageEvts[idx][i];
                if(dt<m_ageThresh)
                {
                    polMat[idx].data[3*i + 0] = static_cast<uchar>(0);              // Blue channel
                    polMat[idx].data[3*i + 1] = static_cast<uchar>(pol>0?255:0);    // Green channel for positive events
                    polMat[idx].data[3*i + 2] = static_cast<uchar>(pol==0?255:0);    // Red Channel for negative events
                }

                // Events by age
                else { dt = m_ageThresh; }
                ageMatHSV[idx].data[3*i + 0] = static_cast<uchar>(0.75*180*dt/float(m_ageThresh)); //H
                ageMatHSV[idx].data[3*i + 1] = static_cast<uchar>(255); //S
                ageMatHSV[idx].data[3*i + 2] = static_cast<uchar>(dt==m_ageThresh?0:255); //V

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
            double z = m_depthMap[i];
            if (z>0)
            {
                if (z<m_min_depth){ z = m_min_depth; }
                else if (z>m_max_depth){ z = m_max_depth; }
                depthMatHSV.data[3*i + 0] = static_cast<uchar>(0.8*180.*(z-m_min_depth)/(m_max_depth-m_min_depth));
                depthMatHSV.data[3*i + 1] = static_cast<uchar>(255);
                depthMatHSV.data[3*i + 2] = static_cast<uchar>(255);
            }
            else
            {
                depthMatHSV.data[3*i + 0] = static_cast<uchar>(0.8*180.*(z-m_min_depth)/(m_max_depth-m_min_depth));
                depthMatHSV.data[3*i + 1] = static_cast<uchar>(255);
                depthMatHSV.data[3*i + 2] = static_cast<uchar>(0);
            }
        }

        for (uint idx=0; idx<m_nbCams; idx++)
        {
            if (m_davis[idx] != nullptr)
            {
                // Display events by polarity
                cv::imshow(m_polWin[idx],polMat[idx]);

                // Display events by age (timestamp)
                //cv::cvtColor(ageMatHSV0,ageMatRGB0,CV_HSV2BGR);
                //cv::imshow(m_ageWin0,ageMatRGB0);

                // Display frame
                if (m_calibrator->m_calibrateCameras)
                {
                    //cv::equalizeHist(m_grayFrame0,m_grayFrame0); // TO DEBUG
                    m_calibrator->calibrateCameras(m_grayFrame[idx],idx);
                }
                cv::imshow(m_frameWin[idx],m_grayFrame[idx]);
            }

            // Display filtered events
            if(m_filter[idx] != nullptr)
            {
                int x = m_filter[idx]->getX();
                int y = m_filter[idx]->getY();
                cv::circle(filtMat[idx],cv::Point2i(y,x),
                           3,cv::Scalar(0,255,0));
                cv::imshow(m_filtWin[idx],filtMat[idx]);
            }
        }

        if (m_triangulator != nullptr)
        {
            // Display depth map
            cv::cvtColor(depthMatHSV,depthMatRGB,CV_HSV2BGR);
            cv::imshow(m_depthWin,depthMatRGB);
        }

        // Reset the display matrices
        for (auto& v : polMat) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC3); }
        for (auto& v : filtMat) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC3); }
        for (auto& v : ageMatHSV) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC3); }

        // Wait (in ms)
        key = cv::waitKey(1);

        // Reset depth map
        if (key=='r')
        {
            m_depthMap.clear();
            m_depthMap.resize(m_rows*m_cols);
            printf("Reset depth map.\n\r");
        }

        // Calibrate cameras
        if (key=='c')
        {
            m_calibrator->m_calibrateCameras = true;
        }
    }
}
