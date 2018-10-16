#include <EBV_DFF_Visualizer.h>

#include <string.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//#include <fstream>
//std::ofstream file;


Visualizer::Visualizer(int rows, int cols, int nbCams)
    : m_rows(rows), m_cols(cols), m_nbCams(nbCams)
{
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
    m_polWin1 = "Events by Polarity - Slave";
    m_ageWin0 = "Events by Age - Master";
    m_ageWin1 = "Events by Age - Slave";
    m_frameWin0 = "Frame - Master";
    m_frameWin1 = "Frame - Slave";
    m_filtWin0 = "Filtered Events - Master";

    // Create display windows
    cv::namedWindow(m_polWin0,0);
    cv::namedWindow(m_polWin1,0);
    cv::namedWindow(m_ageWin0,0);
    cv::namedWindow(m_ageWin1,0);
    cv::namedWindow(m_frameWin0,0);
    cv::namedWindow(m_frameWin1,0);
    cv::namedWindow(m_filtWin0,0);

    m_thresh = 40e3; // 40ms
    cv::createTrackbar("Threshold",m_ageWin0,&m_thresh,m_max_trackbar_val,0);

    // Initialize laser
    m_laser.init("/dev/ttyUSB0");

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
    m_filtEvts0[x*m_cols+y] = e.m_timestamp; // p={-1,1}
    //file << x << "\t" << y << "\t" << e.m_timestamp << std::endl;
}


void Visualizer::run()
{
    char key = ' ';

    // Initialize display matrices
    cv::Mat polMat0(m_rows,m_cols,CV_8UC3);
    cv::Mat ageMatHSV0(m_rows,m_cols,CV_8UC3);
    cv::Mat ageMatRGB0(m_rows,m_cols,CV_8UC3);
    cv::Mat polMat1(m_rows,m_cols,CV_8UC3);
    cv::Mat ageMatHSV1(m_rows,m_cols,CV_8UC3);
    cv::Mat ageMatRGB1(m_rows,m_cols,CV_8UC3);
    cv::Mat filtMat0(m_rows,m_cols,CV_8UC3);

    // Initialize laser position
    //float t = 0;
    //int cx = 2048, cy=2048, r=1592;
    //int x, y;
    //m_laser.blink(0);
    //m_laser.blink(1e6/204);

    while(key != 'q')
    {
        // Laser control: draw a circle
        //t += 1./10; // 10*20ms => 1 cycle in 200ms
        //x = cx + r*cos(2*3.14*t);
        //y = cy + r*sin(2*3.14*t);
        //m_laser.pos(x,y);
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
                //m_grayFrame0.data[3*i + 2] = (unsigned char)255;
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
        cv::imshow(m_polWin1,polMat1);

        // Display events by age
        cv::cvtColor(ageMatHSV0,ageMatRGB0,CV_HSV2BGR);
        cv::imshow(m_ageWin0,ageMatRGB0);
        cv::cvtColor(ageMatHSV1,ageMatRGB1,CV_HSV2BGR);
        cv::imshow(m_ageWin1,ageMatRGB1);

        // Display filtered events
        cv::imshow(m_filtWin0,filtMat0);

        // Display frame
        cv::imshow(m_frameWin0,m_grayFrame0);
        cv::imshow(m_frameWin1,m_grayFrame1);

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
