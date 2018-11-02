#include <EBV_DFF_Visualizer.h>

#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

cv::VideoWriter m_video0("../calibration/calibCircles0.avi",
                         CV_FOURCC('M', 'J', 'P', 'G'),
                         10, cv::Size(240,180),true);
cv::VideoWriter m_video1("../calibration/calibCircles1.avi",
                         CV_FOURCC('M', 'J', 'P', 'G'),
                         10, cv::Size(240,180),true);

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

Visualizer::Visualizer(const unsigned int rows,
                       const unsigned int cols,
                       const unsigned int nbCams,
                       DAVIS240C* davis0, DAVIS240C* davis1,
                       Filter* filter0, Filter* filter1,
                       Triangulator* triangulator)
    : m_rows(rows), m_cols(cols), m_nbCams(nbCams),
      m_davis0(davis0), m_davis1(davis1),
      m_filter0(filter0), m_filter1(filter1),
      m_triangulator(triangulator)
{
    // Listen to Davis
    m_davis0->registerEventListener(this);
    m_davis0->registerFrameListener(this);

    m_davis1->registerEventListener(this);
    m_davis1->registerFrameListener(this);

    // Listen to filter
    m_filter0->registerFilterListener(this);
    m_filter1->registerFilterListener(this);

    // Listen to triangulator
    m_triangulator->registerTriangulatorListener(this);

    // Initialize laser
    m_laser.init("/dev/ttyUSB0");

    // Initialize data structure to hold the events (flatten matrices)
    m_polEvts0.resize(m_rows*m_cols);
    m_ageEvts0.resize(m_rows*m_cols);
    m_filtEvts0.resize(m_rows*m_cols);
    m_polEvts1.resize(m_rows*m_cols);
    m_ageEvts1.resize(m_rows*m_cols);
    m_filtEvts1.resize(m_rows*m_cols);
    m_depthMap.resize(m_rows*m_cols);

    // Initialize data structure to hold the frame
    m_grayFrame0.resize(m_rows*m_cols);
    m_grayFrame0 = cv::Mat::zeros(m_rows,m_cols,CV_8UC1);
    m_grayFrame1.resize(m_rows*m_cols);
    m_grayFrame1 = cv::Mat::zeros(m_rows,m_cols,CV_8UC1);

    // Windows names
    m_polWin0 = "Events by Polarity - Master"; //+ std::to_string(id);
    m_ageWin0 = "Events by Age - Master";
    m_frameWin0 = "Frame - Master";
    m_filtWin0 = "Filtered Events - Master";
    m_polWin1 = "Events by Polarity - Slave";
    m_ageWin1 = "Events by Age - Slave";
    m_frameWin1 = "Frame - Slave";
    m_filtWin1 = "Filtered Events - Slave";
    m_depthWin = "Depth Map";

    // Create display windows
    cv::namedWindow(m_polWin0,0);
    //cv::namedWindow(m_ageWin0,0);
    cv::namedWindow(m_frameWin0,0);
    cv::namedWindow(m_filtWin0,0);

    cv::namedWindow(m_polWin1,0);
    //cv::namedWindow(m_ageWin1,0);
    cv::namedWindow(m_frameWin1,0);
    cv::namedWindow(m_filtWin1,0);
    cv::namedWindow(m_depthWin,0);

    // Initialize tuning parameters
    m_thresh = 40e3; // 40ms
    m_freq0 = m_filter0->getFreq();
    m_eps0 = m_filter0->getEps();
    m_neighborSize0 = m_filter0->getNeighborSize();
    m_threshA0 = m_filter0->getThreshA();
    m_threshB0 = m_filter0->getThreshB();
    m_threshAnti0 = m_filter0->getThreshAnti();
    m_etaInt0 = static_cast<int>(100*m_filter0->getEta());

    m_freq1 = m_filter1->getFreq();
    m_eps1 = m_filter1->getEps();
    m_neighborSize1 = m_filter1->getNeighborSize();
    m_threshA1 = m_filter1->getThreshA();
    m_threshB1 = m_filter1->getThreshB();
    m_threshAnti1 = m_filter1->getThreshAnti();
    m_etaInt1 = static_cast<int>(100*m_filter1->getEta());

    printf("\rFreq set to: %d. \n\r",m_freq0);
    printf("Eps set to: %d. \n\r",m_eps0);
    printf("Neighbor Size set to: %d. \n\r",m_neighborSize0);
    printf("SupportsA set to: %d. \n\r",m_threshA0);
    printf("SupportsB set to: %d. \n\r",m_threshB0);
    printf("Anti-Supports set to: %d. \n\r",m_threshAnti0);
    printf("Eta set to: %d. \n\r\n",m_etaInt0);

    printf("Freq set to: %d. \n\r",m_freq1);
    printf("Eps set to: %d. \n\r",m_eps1);
    printf("Neighbor Size set to: %d. \n\r",m_neighborSize1);
    printf("SupportsA set to: %d. \n\r",m_threshA1);
    printf("SupportsB set to: %d. \n\r",m_threshB1);
    printf("Anti-Supports set to: %d. \n\r",m_threshAnti1);
    printf("Eta set to: %d. \n\r\n",m_etaInt1);

    // Filter tuning trackbars
    cv::createTrackbar("Threshold",m_ageWin0,&m_thresh,m_max_trackbar_val,nullptr);
    cv::createTrackbar("Frequency",m_filtWin0,&m_freq0,1000,
                       &callbackTrackbarFreq,(void*)(m_filter0));
    cv::createTrackbar("Epsilon",m_filtWin0,&m_eps0,20,
                       &callbackTrackbarEps,(void*)(m_filter0));
    cv::createTrackbar("Neighbor Size",m_filtWin0,&m_neighborSize0,100,
                       &callbackTrackbarNeighborSize,(void*)(m_filter0));
    cv::createTrackbar("SupportsA",m_filtWin0,&m_threshA0,100,
                       &callbackTrackbarThreshA,(void*)(m_filter0));
    cv::createTrackbar("SupportsB",m_filtWin0,&m_threshB0,100,
                       &callbackTrackbarThreshB,(void*)(m_filter0));
    cv::createTrackbar("Anti-Supports",m_filtWin0,&m_threshAnti0,100,
                       &callbackTrackbarThreshAnti,(void*)(m_filter0));
    cv::createTrackbar("Eta",m_filtWin0,&m_etaInt0,100,
                       &callbackTrackbarEta,(void*)(m_filter0));

    cv::createTrackbar("Frequency",m_filtWin1,&m_freq1,1000,
                       &callbackTrackbarFreq,(void*)(m_filter1));
    cv::createTrackbar("Epsilon",m_filtWin1,&m_eps1,20,
                       &callbackTrackbarEps,(void*)(m_filter1));
    cv::createTrackbar("Neighbor Size",m_filtWin1,&m_neighborSize1,100,
                       &callbackTrackbarNeighborSize,(void*)(m_filter1));
    cv::createTrackbar("SupportsA",m_filtWin1,&m_threshA1,100,
                       &callbackTrackbarThreshA,(void*)(m_filter1));
    cv::createTrackbar("SupportsB",m_filtWin1,&m_threshB1,100,
                       &callbackTrackbarThreshB,(void*)(m_filter1));
    cv::createTrackbar("Anti-Supports",m_filtWin1,&m_threshAnti1,100,
                       &callbackTrackbarThreshAnti,(void*)(m_filter1));
    cv::createTrackbar("Eta",m_filtWin1,&m_etaInt1,100,
                       &callbackTrackbarEta,(void*)(m_filter1));

    // Slider depth
    cv::createTrackbar("minDepth",m_depthWin,&m_min_depth,10000,nullptr);
    cv::createTrackbar("maxDepth",m_depthWin,&m_max_depth,10000,nullptr);

    // Saving events in .txt
    //m_recorder.open("closer_bottom_vertical.txt");

    m_davis0->init();
    m_davis0->start();
    m_davis0->listen();
    m_davis1->init();
    m_davis1->start();
    m_davis1->listen();
}


Visualizer::~Visualizer()
{
    m_laser.close();

    m_davis0->deregisterEventListener(this);
    m_davis0->deregisterFrameListener(this);
    m_davis0->stopListening();
    m_davis0->stop();
    m_davis0->close();
    m_filter0->deregisterFilterListener(this);


    m_davis1->deregisterEventListener(this);
    m_davis1->deregisterFrameListener(this);
    m_davis1->stopListening();
    m_davis1->stop();
    m_davis1->close();
    m_filter1->deregisterFilterListener(this);

    m_triangulator->deregisterTriangulatorListener(this);

    //m_recorder.close();
    m_video0.release();
    m_video1.release();
}


void Visualizer::receivedNewDVS128USBEvent(DVS128USBEvent& e)
{
    const unsigned int x = e.m_x;
    const unsigned int y = e.m_y;
    const unsigned int p = e.m_pol; // p={0,1}

    m_evtMutex0.lock();
        m_polEvts0[x*m_cols+y] += 2*p-1; // p={-1,1}
    m_evtMutex0.unlock();
}


void Visualizer::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                           const unsigned int id)
{
    const unsigned int x = e.m_x;
    const unsigned int y = e.m_y;
    const unsigned int p = e.m_pol; // p={0,1}
    const int t = e.m_timestamp;

    switch(id)
    {
        case 0:
            m_evtMutex0.lock();
                m_currenTime0 = t;
                m_polEvts0[x*m_cols+y] = 2*p-1; // p={-1,1}
                m_ageEvts0[x*m_cols+y] = t;
            m_evtMutex0.unlock();
        break;

        case 1:
            m_evtMutex1.lock();
                m_currenTime1 = t;
                m_polEvts1[x*m_cols+y] = 2*p-1; // p={-1,1}
                m_ageEvts1[x*m_cols+y] = t;
            m_evtMutex1.unlock();
        break;
    }
}


void Visualizer::receivedNewDAVIS240CFrame(DAVIS240CFrame& f,
                                           const unsigned int id)
{
    switch(id)
    {
        case 0:
        {
            m_frameMutex0.lock();
                m_currenTime0 = f.m_timestamp;
                m_grayFrame0 = cv::Mat(m_rows,m_cols,CV_8UC1,f.m_frame.data());
                //cv::Mat frame0;
                //cv::cvtColor(m_grayFrame0,frame0,cv::COLOR_GRAY2BGR);
                //m_video0.write(frame0);
            m_frameMutex0.unlock();
            break;
        }

        case 1:
        {
            m_frameMutex1.lock();
                m_currenTime1 = f.m_timestamp;
                m_grayFrame1 = cv::Mat(m_rows,m_cols,CV_8UC1,f.m_frame.data());
                //cv::Mat frame1;
                //cv::cvtColor(m_grayFrame1,frame1,cv::COLOR_GRAY2BGR);
                //m_video1.write(frame1);
            m_frameMutex1.unlock();
            break;
        }
    }
}


void Visualizer::receivedNewFilterEvent(DAVIS240CEvent& e,
                                        const unsigned int id)
{
    const unsigned int x = e.m_x;
    const unsigned int y = e.m_y;
    const int t = e.m_timestamp;
    //printf("Camera %d - Timestamp %f. \n\r",id,1e-3*e.m_timestamp);

    switch (id)
    {
        case 0:
        {
            m_filterEvtMutex0.lock();
                m_filtEvts0[x*m_cols+y] = t;
                //m_recorder << id << "\t" << x << "\t" << y << "\t" << e.m_timestamp << std::endl;
             m_filterEvtMutex0.unlock();
            break;
        }

        case 1:
        {
            m_filterEvtMutex1.lock();
                m_filtEvts1[x*m_cols+y] = t;
                //m_recorder << id << "\t" << x << "\t" << y << "\t" << e.m_timestamp << std::endl;
            m_filterEvtMutex1.unlock();
            break;
        }
    }
}


void Visualizer::receivedNewDepth(const unsigned int &u,
                                  const unsigned int &v,
                                  const double &depth)
{
    m_depthMutex.lock();
        m_depthMap[u*m_cols+v] = depth;
    m_depthMutex.unlock();
}

void Visualizer::run()
{   
    char key = ' ';

    // Initialize display matrices
    cv::Mat polMat0(m_rows,m_cols,CV_8UC3);
    cv::Mat ageMatHSV0(m_rows,m_cols,CV_8UC3);
    cv::Mat ageMatRGB0(m_rows,m_cols,CV_8UC3);
    cv::Mat filtMat0(m_rows,m_cols,CV_8UC3);
    cv::Mat polMat1(m_rows,m_cols,CV_8UC3);
    cv::Mat ageMatHSV1(m_rows,m_cols,CV_8UC3);
    cv::Mat ageMatRGB1(m_rows,m_cols,CV_8UC3);
    cv::Mat filtMat1(m_rows,m_cols,CV_8UC3);
    cv::Mat depthMatHSV(m_rows,m_cols,CV_8UC3);
    cv::Mat depthMatRGB(m_rows,m_cols,CV_8UC3);

    // Initialize laser position
    double t = 0;
    unsigned int cx = 2048, cy=2048, r=500; //r=1592;
    unsigned int x, y;
    int freq = 600;
    m_laser.blink(1e6/(2*freq)); // T/2
    printf("Blinking frequency set to %d.\n\r\n",freq);

    while(key != 'q')
    {
        // Laser control: draw a circle
        t += 1./10; // Need 10 increments for full cycle = 200ms (10*20ms waitKey)
        x = static_cast<unsigned int>(cx + r*cos(2.*3.14*t));
        //y = static_cast<unsigned int>(cx);
        y = static_cast<unsigned int>(cy + r*sin(2.*3.14*t));
        m_laser.pos(x,y);
        //m_laser.vel(8e4,8e4);

        for(unsigned int i=0; i<m_rows*m_cols; i++)
        {
            // Events by polarity - Master
            m_evtMutex0.lock();
                int pol = m_polEvts0[i];
                int dt = m_currenTime0 - m_ageEvts0[i];
            m_evtMutex0.unlock();

            if(dt<m_thresh)
            {
                polMat0.data[3*i + 0] = static_cast<unsigned char>(0);              // Blue channel
                polMat0.data[3*i + 1] = static_cast<unsigned char>(pol>0?255:0);    // Green channel
                polMat0.data[3*i + 2] = static_cast<unsigned char>(pol<0?255:0);    // Red Channel
            }

            // Events by polarity - Slave
            m_evtMutex1.lock();
                pol = m_polEvts1[i];
                dt = m_currenTime1 - m_ageEvts1[i];
            m_evtMutex1.unlock();

            if(dt<m_thresh)
            {
                polMat1.data[3*i + 0] = static_cast<unsigned char>(0);              // Blue channel
                polMat1.data[3*i + 1] = static_cast<unsigned char>(pol>0?255:0);    // Green channel
                polMat1.data[3*i + 2] = static_cast<unsigned char>(pol<0?255:0);    // Red Channel
            }

            // Events by age - Master
            m_evtMutex0.lock();
                dt = m_currenTime0 - m_ageEvts0[i];
            m_evtMutex0.unlock();

            if(dt>m_thresh) { dt = m_thresh; }
            ageMatHSV0.data[3*i + 0] = static_cast<unsigned char>(0.75*180*dt/float(m_thresh)); //H
            ageMatHSV0.data[3*i + 1] = static_cast<unsigned char>(255); //S
            ageMatHSV0.data[3*i + 2] = static_cast<unsigned char>(dt==m_thresh?0:255); //V

            // Events by age - Slave
            m_evtMutex1.lock();
                dt = m_currenTime1 - m_ageEvts1[i];
            m_evtMutex1.unlock();

            if(dt>m_thresh) { dt = m_thresh; }
            ageMatHSV1.data[3*i + 0] = static_cast<unsigned char>(0.75*180.*dt/float(m_thresh)); //H
            ageMatHSV1.data[3*i + 1] = static_cast<unsigned char>(255); //S
            ageMatHSV1.data[3*i + 2] = static_cast<unsigned char>(dt==m_thresh?0:255); //V

            // Filtered events - Master
            m_filterEvtMutex0.lock();
                dt = m_currenTime0 - m_filtEvts0[i];
            m_filterEvtMutex0.unlock();

            if(dt < m_thresh)
            {
                filtMat0.data[3*i + 0] = static_cast<unsigned char>(0);
                filtMat0.data[3*i + 1] = static_cast<unsigned char>(0);
                filtMat0.data[3*i + 2] = static_cast<unsigned char>(255);  // why not change color given laser frequency?
            }
            else
            {
                filtMat0.data[3*i + 0] = static_cast<unsigned char>(0);
                filtMat0.data[3*i + 1] = static_cast<unsigned char>(0);
                filtMat0.data[3*i + 2] = static_cast<unsigned char>(0);
            }

            // Filtered events - Slave
            m_filterEvtMutex1.lock();
                dt = m_currenTime1 - m_filtEvts1[i];
            m_filterEvtMutex1.unlock();

            if(dt < m_thresh)
            {
                filtMat1.data[3*i + 0] = static_cast<unsigned char>(0);
                filtMat1.data[3*i + 1] = static_cast<unsigned char>(0);
                filtMat1.data[3*i + 2] = static_cast<unsigned char>(255);  // why not change color given laser frequency?
            }
            else
            {
                filtMat1.data[3*i + 0] = static_cast<unsigned char>(0);
                filtMat1.data[3*i + 1] = static_cast<unsigned char>(0);
                filtMat1.data[3*i + 2] = static_cast<unsigned char>(0);
            }

            // Depth map
            m_depthMutex.lock();
                double z = m_depthMap[i];
            m_depthMutex.unlock();
            if (z>0)
            {
                if (z<m_min_depth){ z = m_min_depth; }
                else if (z>m_max_depth){ z = m_max_depth; }
                depthMatHSV.data[3*i + 0] = static_cast<unsigned char>(180.*(z-m_min_depth)/(m_max_depth-m_min_depth));
                depthMatHSV.data[3*i + 1] = static_cast<unsigned char>(255);
                depthMatHSV.data[3*i + 2] = static_cast<unsigned char>(255);
            }
            else
            {
                depthMatHSV.data[3*i + 0] = static_cast<unsigned char>(180.*(z-m_min_depth)/(m_max_depth-m_min_depth));
                depthMatHSV.data[3*i + 1] = static_cast<unsigned char>(255);
                depthMatHSV.data[3*i + 2] = static_cast<unsigned char>(0);
            }
        }

        // Display events by polarity
        cv::imshow(m_polWin0,polMat0);
        cv::imshow(m_polWin1,polMat1);

        // Display events by age (timestamp)
        //cv::cvtColor(ageMatHSV0,ageMatRGB0,CV_HSV2BGR);
        //cv::imshow(m_ageWin0,ageMatRGB0);
        //cv::cvtColor(ageMatHSV1,ageMatRGB1,CV_HSV2BGR);
        //cv::imshow(m_ageWin1,ageMatRGB1);

        // Display frame
        m_frameMutex0.lock();
            cv::imshow(m_frameWin0,m_grayFrame0);
        m_frameMutex0.unlock();
        m_frameMutex1.lock();
            cv::imshow(m_frameWin1,m_grayFrame1);
        m_frameMutex1.unlock();

        // Display trackers
        if(m_filter0 != nullptr)
        {
            m_filterEvtMutex0.lock();
                int x = static_cast<int>(m_filter0->getX());
                int y = static_cast<int>(m_filter0->getY());
            m_filterEvtMutex0.unlock();
            cv::circle(filtMat0,cv::Point2i(y,x),
                       3,cv::Scalar(0,255,0));
        }

        if(m_filter1 != nullptr)
        {
            m_filterEvtMutex1.lock();
                int x = static_cast<int>(m_filter1->getX());
                int y = static_cast<int>(m_filter1->getY());
            m_filterEvtMutex1.unlock();
            cv::circle(filtMat1,cv::Point2i(y,x),
                       3,cv::Scalar(0,255,0));
        }

        // Display filtered events + trackers
        cv::imshow(m_filtWin0,filtMat0);
        cv::imshow(m_filtWin1,filtMat1);

        // Display depth map
        cv::cvtColor(depthMatHSV,depthMatRGB,CV_HSV2BGR);
        cv::imshow(m_depthWin,depthMatRGB);

        // Reset the display matrices
        ageMatHSV0 = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);
        polMat0 = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);
        filtMat0 = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);
        ageMatHSV1 = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);
        polMat1 = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);
        filtMat1 = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);

        // Wait for 20ms
        key = cv::waitKey(20);
    }
}
