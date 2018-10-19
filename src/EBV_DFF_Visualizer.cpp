#include <EBV_DFF_Visualizer.h>

#include <string.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <EBV_Filter.h>

//#include <fstream>
//std::ofstream file;

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
    filter->setEta(newEta/100.);
}


Visualizer::Visualizer(int rows, int cols, int nbCams,
                       Filter* filter)
    : m_rows(rows), m_cols(cols), m_nbCams(nbCams)
{
    // Initialize laser
    m_laser.init("/dev/ttyUSB1");

    // Initialize filter
    this->setFilter(filter);

    // Initialize data structure to hold the events (flatten matrices)
    m_polEvts0.resize(m_rows*m_cols);
    m_ageEvts0.resize(m_rows*m_cols);
    m_filtEvts0.resize(m_rows*m_cols);
    m_polEvts1.resize(m_rows*m_cols);
    m_ageEvts1.resize(m_rows*m_cols);

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

    // Create display windows
    cv::namedWindow(m_polWin0,0);
    cv::namedWindow(m_ageWin0,0);
    cv::namedWindow(m_frameWin0,0);
    cv::namedWindow(m_filtWin0,0);
    //cv::namedWindow(m_polWin1,0);
    //cv::namedWindow(m_ageWin1,0);
    //cv::namedWindow(m_frameWin1,0);

    // Initialize tuning parameters
    m_thresh = 40e3; // 40ms
    m_freq = m_filter->getFreq();
    m_eps = m_filter->getEps();
    m_neighborSize = m_filter->getNeighborSize();
    m_threshA = m_filter->getThreshA();
    m_threshB = m_filter->getThreshB();
    m_threshAnti = m_filter->getThreshAnti();
    m_etaInt = static_cast<int>(100*m_filter->getEta());
    printf("Freq set to: %d. \n\r",m_freq);
    printf("Eps set to: %d%. \n\r",m_eps);
    printf("Neighbor Size set to: %d. \n\r",m_neighborSize);
    printf("SupportsA set to: %d. \n\r",m_threshA);
    printf("SupportsB set to: %d. \n\r",m_threshB);
    printf("Anti-Supports set to: %d. \n\r",m_threshAnti);
    printf("Eta set to: %d. \n\r",m_etaInt);

    // Filter tuning trackbars
    cv::createTrackbar("Threshold",m_ageWin0,&m_thresh,m_max_trackbar_val,0);
    cv::createTrackbar("Frequency",m_filtWin0,&m_freq,1000,
                       &callbackTrackbarFreq,(void*)(m_filter));
    cv::createTrackbar("Epsilon",m_filtWin0,&m_eps,20,
                       &callbackTrackbarEps,(void*)(m_filter));
    cv::createTrackbar("Neighbor Size",m_filtWin0,&m_neighborSize,100,
                       &callbackTrackbarNeighborSize,(void*)(m_filter));
    cv::createTrackbar("SupportsA",m_filtWin0,&m_threshA,100,
                       &callbackTrackbarThreshA,(void*)(m_filter));
    cv::createTrackbar("SupportsB",m_filtWin0,&m_threshB,100,
                       &callbackTrackbarThreshB,(void*)(m_filter));
    cv::createTrackbar("Anti-Supports",m_filtWin0,&m_threshAnti,100,
                       &callbackTrackbarThreshAnti,(void*)(m_filter));
    cv::createTrackbar("Eta",m_filtWin0,&m_etaInt,100,
                       &callbackTrackbarEta,(void*)(m_filter));

    // Saving events in .txt file
    //file.open("MyData.txt");
}


Visualizer::~Visualizer()
{
    m_laser.close();
    //file.close();
}


void Visualizer::receivedNewDVS128USBEvent(DVS128USBEvent& e)
{
    int x = e.m_x;
    int y = e.m_y;
    int p = e.m_pol; // p={0,1}

    m_evtMutex.lock();
        m_polEvts0[x*m_cols+y] += 2*p-1; // p={-1,1}
    m_evtMutex.unlock();
}


void Visualizer::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,int id)
{
    int x = e.m_x;
    int y = e.m_y;
    int p = e.m_pol; // p={0,1}
    unsigned int t = e.m_timestamp;

    switch(id)
    {
        case 0:
            m_evtMutex.lock();
                m_currenTime0 = e.m_timestamp;
                m_polEvts0[x*m_cols+y] = 2*p-1; // p={-1,1}
                m_ageEvts0[x*m_cols+y] = t;
            m_evtMutex.unlock();
        break;

        case 1:
            m_evtMutex.lock();
                m_currenTime1 = e.m_timestamp;
                m_polEvts1[x*m_cols+y] = 2*p-1; // p={-1,1}
                m_ageEvts1[x*m_cols+y] = t;
            m_evtMutex.unlock();
        break;
    }
}


void Visualizer::receivedNewDAVIS240CFrame(DAVIS240CFrame& f,int id)
{
    switch(id)
    {
        case 0:
            m_evtMutex.lock();
                m_currenTime0 = f.m_timestamp;
                m_grayFrame0 = cv::Mat(m_rows,m_cols,CV_8UC1,f.m_frame.data());
            m_evtMutex.unlock();
        break;

        case 1:
            m_evtMutex.lock();
                m_currenTime1 = f.m_timestamp;
                m_grayFrame1 = cv::Mat(m_rows,m_cols,CV_8UC1,f.m_frame.data());
            m_evtMutex.unlock();
        break;
    }
}


void Visualizer::receivedNewFilterEvent(DAVIS240CEvent& e, int id)
{
    int x = e.m_x;
    int y = e.m_y;
    m_filtEvts0[x*m_cols+y] = e.m_timestamp;
    //file << x << "\t" << y << "\t" << e.m_timestamp << std::endl;
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

    // Initialize laser position
    float t = 0;
    int cx = 2048, cy=2048, r=500; //r=1592;
    int x, y;
    m_laser.blink(1e6/(2*300)); // T/2


    while(key != 'q')
    {
        // Laser control: draw a circle
        t += 1./10; // Need 10 increments for full cycle = 200ms (10*20ms waitKey)
        x = cx + r*cos(2*3.14*t);
        y = cy + r*sin(2*3.14*t);
        m_laser.pos(x,y);
        //m_laser.vel(8e4,8e4);

        m_evtMutex.lock();
        for(int i=0; i<m_rows*m_cols; i++)
        {
            // Events by polarity - Master
            int pol = m_polEvts0[i];
            int dt = m_currenTime0 - m_ageEvts0[i];
            if(dt<m_thresh)
            {
                polMat0.data[3*i + 0] = (unsigned char)0;              // Blue channel
                polMat0.data[3*i + 1] = (unsigned char)(pol>0?255:0);    // Green channel
                polMat0.data[3*i + 2] = (unsigned char)(pol<0?255:0);    // Red Channel
            }

            // Events by polarity - Slave
            pol = m_polEvts1[i];
            dt = m_currenTime1 - m_ageEvts1[i];
            if(dt<m_thresh)
            {
                polMat1.data[3*i + 0] = (unsigned char)0;              // Blue channel
                polMat1.data[3*i + 1] = (unsigned char)(pol>0?255:0);    // Green channel
                polMat1.data[3*i + 2] = (unsigned char)(pol<0?255:0);    // Red Channel
            }

            // Events by age - Master
            dt = m_currenTime0 - m_ageEvts0[i];
            if(dt>m_thresh) { dt = m_thresh; }
            ageMatHSV0.data[3*i + 0] = (unsigned char)(0.75*180.*dt/float(m_thresh)); //H
            ageMatHSV0.data[3*i + 1] = (unsigned char)255; //S
            ageMatHSV0.data[3*i + 2] = (unsigned char)(dt==m_thresh?0:255); //V

            // Events by age - Slave
            dt = m_currenTime1 - m_ageEvts1[i];
            if(dt>m_thresh) { dt = m_thresh; }
            ageMatHSV1.data[3*i + 0] = (unsigned char)(0.75*180.*dt/float(m_thresh)); //H
            ageMatHSV1.data[3*i + 1] = (unsigned char)255; //S
            ageMatHSV1.data[3*i + 2] = (unsigned char)(dt==m_thresh?0:255); //V

            // Filtered events - Master
            dt = m_currenTime0 - m_filtEvts0[i];
            if(dt < m_thresh)
            {
                filtMat0.data[3*i + 0] = (unsigned char)0;
                filtMat0.data[3*i + 1] = (unsigned char)0;
                filtMat0.data[3*i + 2] = (unsigned char)255;  // why not change color given laser frequency?
            }
            else
            {
                filtMat0.data[3*i + 0] = (unsigned char)0;
                filtMat0.data[3*i + 1] = (unsigned char)0;
                filtMat0.data[3*i + 2] = (unsigned char)0;
            }
        }
        m_evtMutex.unlock();

        // Display events by polarity
        cv::imshow(m_polWin0,polMat0);
        //cv::imshow(m_polWin1,polMat1);

        // Display events by age
        cv::cvtColor(ageMatHSV0,ageMatRGB0,CV_HSV2BGR);
        cv::imshow(m_ageWin0,ageMatRGB0);
        //cv::cvtColor(ageMatHSV1,ageMatRGB1,CV_HSV2BGR);
        //cv::imshow(m_ageWin1,ageMatRGB1);

        // Display frame
        cv::imshow(m_frameWin0,m_grayFrame0);
        //cv::imshow(m_frameWin1,m_grayFrame1);

        // Display filtered events
        if(m_filter != NULL)
        {
            int x = m_filter->getX();
            int y = m_filter->getY();
            cv::circle(filtMat0,cv::Point2i(y,x),
                       3,cv::Scalar(0,255,0));
        }
        cv::imshow(m_filtWin0,filtMat0);

        // Reset the display matrices
        ageMatHSV0 = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);
        ageMatHSV1 = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);
        polMat1 = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);
        polMat0 = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);
        filtMat0 = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);

        // Wait for 20ms
        key = cv::waitKey(20);
    }
}
