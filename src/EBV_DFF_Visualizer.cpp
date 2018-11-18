#include <EBV_DFF_Visualizer.h>

#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

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

static void callbackTrackbarLaserLearningRate(int lrInt, void *data)
{
    LaserController* laser = static_cast<LaserController*>(data);
    laser->setLearningRate(lrInt/100.f);
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
    for (auto& v : m_polEvts) { v.resize(m_rows*m_cols); }
    for (auto& v : m_ageEvts) { v.resize(m_rows*m_cols); }
    for (auto& v : m_filtEvts) { v.resize(m_rows*m_cols); }
    for (auto& v : m_grayFrame) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC1); }
    m_depthMap.resize(m_rows*m_cols);

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
            m_freq[idx] = m_filter[idx]->getFreq();
            m_eps[idx] = m_filter[idx]->getEps();
            m_neighborSize[idx] = m_filter[idx]->getNeighborSize();
            m_threshA[idx] = m_filter[idx]->getThreshA();
            m_threshB[idx] = m_filter[idx]->getThreshB();
            m_threshAnti[idx] = m_filter[idx]->getThreshAnti();
            m_etaInt[idx] = static_cast<int>(100*m_filter[idx]->getEta());

            printf("\rFreq set to: %d. \n\r",m_freq[idx]);
            printf("Eps set to: %d. \n\r",m_eps[idx]);
            printf("Neighbor Size set to: %d. \n\r",m_neighborSize[idx]);
            printf("SupportsA set to: %d. \n\r",m_threshA[idx]);
            printf("SupportsB set to: %d. \n\r",m_threshB[idx]);
            printf("Anti-Supports set to: %d. \n\r",m_threshAnti[idx]);
            printf("Eta set to: %d. \n\r\n",m_etaInt[idx]);

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
        }
    }

    // Initialize triangulator
    if (m_triangulator!=nullptr)
    {
        // Listen to triangulator
        m_triangulator->registerTriangulatorListener(this);

        // Initialize window
        m_depthWin = "Depth Map";
        cv::namedWindow(m_depthWin,0);

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

    // Initialize laser
    if (m_laser!=nullptr)
    {
        m_laser->start();

        // Initialize tuning parameters (laser)
        m_laserX = m_laser->getX();
        m_laserY = m_laser->getY();
        m_lrInt = static_cast<int>(100*m_laser->getLearningRate());
        m_stepInt = static_cast<int>(1./m_laser->getStep());
        m_laserFreq = m_laser->getFreq();

        // Laser trackbars
        cv::createTrackbar("laserX",m_polWin[0],&m_laserX,4000,
                           &callbackTrackbarLaserX,static_cast<void*>(m_laser));
        cv::createTrackbar("laserY",m_polWin[0],&m_laserY,4000,
                           &callbackTrackbarLaserY,static_cast<void*>(m_laser));
        cv::createTrackbar("LearningRate",m_polWin[0],&m_lrInt,100,
                           &callbackTrackbarLaserLearningRate,static_cast<void*>(m_laser));
        cv::createTrackbar("step",m_polWin[0],&m_stepInt,1000,
                           &callbackTrackbarLaserStep,static_cast<void*>(m_laser));
        cv::createTrackbar("Laser frequency",m_polWin[0],&m_laserFreq,1000,
                           &callbackTrackbarLaserFreq,static_cast<void*>(m_laser));
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
    const double depth = Z;
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

        for (unsigned int idx=0; idx<2; idx++)
        {
            if (m_davis[idx] != nullptr)
            {
                // Display events by polarity
                cv::imshow(m_polWin[idx],polMat[idx]);

                // Display events by age (timestamp)
                //cv::cvtColor(ageMatHSV[idx],ageMatRGB[idx],CV_HSV2BGR);
                //cv::imshow(m_ageWin[idx],ageMatRGB[idx]);

                // Display frame
                if (m_calibrator->m_calibrateCameras)
                {
                    //cv::equalizeHist(m_grayFrame[idx],m_grayFrame[idx]);
                    m_calibrator->calibrateCameras(m_grayFrame[idx],idx);
                }
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

        // Reset the display matrices
        for (auto& v : polMat) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC3); }
        for (auto& v : filtMat) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC3); }
        for (auto& v : ageMatHSV) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC3); }

        // Wait (in ms)
        key = cv::waitKey(10);

        // Reset depth map
        if (key=='r')
        {
            m_depthMap.clear();
            m_depthMap.resize(m_rows*m_cols);
            printf("Reset depth map.\n\r");
        }

        if (key=='c')
        {
            m_calibrator->m_calibrateCameras = true;
        }

        if (key=='l')
        {
            m_calibrator->pointLaserToPixel(300,300,0);
            // Detect current laser position in camera
//            int rgoal = 20;
//            int cgoal = 200;

//            int r = m_filter[0]->getX();
//            int c = m_filter[0]->getY();

//            std::array<int,2> diff = {rgoal-r,cgoal-c};
//            long mse = diff[0]*diff[0] + diff[1]*diff[1];

//            printf("Current laser position (%d,%d). \n\r",r,c);
//            printf("MSE: %ld. \n\r",mse);

//            for (int k=0; k<10;k++)
//            {
//                r = m_filter[0]->getX();
//                c = m_filter[0]->getY();

//                diff[0] = rgoal-r;
//                diff[1] = cgoal-c;
//                mse = diff[0]*diff[0] + diff[1]*diff[1];

//                int x = m_laser->getX() + m_laser->getLearningRate()*diff[0];
//                int y = m_laser->getY() + m_laser->getLearningRate()*diff[1];
//                m_laser->pos(x,y);
//                printf("Diff: (%d,%d). \n\r",diff[0],diff[1]);
//                printf("Laser commands: (%d,%d,%d). \n\r",x,y,mse);
//            }
//            //m_calibrator->pointLaserToPixel(300,300,0);

//            r = m_filter[0]->getX();
//            c = m_filter[0]->getY();
//            printf("Current laser position (%d,%d). \n\r",r,c);
        }

    }
}
