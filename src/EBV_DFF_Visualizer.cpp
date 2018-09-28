#include <EBV_DFF_Visualizer.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string.h>

Visualizer::Visualizer(int rows, int cols, int id)
    : m_rows(rows), m_cols(cols)
{
    // Create a data structure to hold the events
    // (a row*col matrix flattened in a vector)
    m_polEvts.resize(m_rows*m_cols);
    m_ageEvts.resize(m_rows*m_cols);

    // Create a data structure to hold the frame
    m_grayFrame.resize(m_rows*m_cols);

    // Create windows to display events by polarity and age
    m_polWin = "Events by Polarity " + std::to_string(id);
    m_ageWin = "Events by Age " + std::to_string(id);
    m_frameWin = "Frame " + std::to_string(id);

    m_grayFrame = cv::Mat::zeros(m_rows,m_cols,CV_8UC1);
    cv::namedWindow(m_polWin,0);
    cv::namedWindow(m_ageWin,0);
    cv::namedWindow(m_frameWin,0);

    m_thresh = 40e3; // 40ms
    cv::createTrackbar("Threshold",m_ageWin,&m_thresh,m_max_trackbar_val,0);
}

Visualizer::~Visualizer()
{

}

void Visualizer::receivedNewDVS128USBEvent(DVS128USBEvent& e)
{
    int x = e.m_x;
    int y = e.m_y;
    int p = e.m_pol; // p={0,1}

    m_evtMutex.lock();
        m_polEvts[x*m_cols+y] += 2*p-1; // p={-1,1}
    m_evtMutex.unlock();
}

void Visualizer::receivedNewDAVIS240CEvent(DAVIS240CEvent& e)
{
    int x = e.m_x;
    int y = e.m_y;
    int p = e.m_pol; // p={0,1}
    unsigned int t = e.m_timestamp;

    // Store event polarity p and timestamp t at a given (x,y) pixel (here vectorized for performance)
    m_evtMutex.lock();
        m_currenTime = e.m_timestamp;
        m_polEvts[x*m_cols+y] = 2*p-1; // p={-1,1}
        m_ageEvts[x*m_cols+y] = t;
    m_evtMutex.unlock();
}

// Store event info in a vectorized datastructure (function called each time the DAVIS receives an event)
void Visualizer::receivedNewDAVIS240CFrame(DAVIS240CFrame& f)
{
    m_evtMutex.lock();
        m_currenTime = f.m_timestamp;
        m_grayFrame = cv::Mat(180,240,CV_8UC1,f.m_frame.data());
                /*
        for(int i=0; i<m_rows*m_cols; i++)
        {
            m_grayFrame.data[i] = (unsigned char)f.m_frame[i];
        }
                */
    m_evtMutex.unlock();
}


void Visualizer::run()
{
    char key = ' ';

    // Initialize display matrices
    cv::Mat polMat(m_rows,m_cols,CV_8UC3);
    cv::Mat ageMatHSV(m_rows,m_cols,CV_8UC3);
    cv::Mat ageMatRGB(m_rows,m_cols,CV_8UC3);

    while(key != 'q')
    {
        // TODO: Explain Mutex and high-level of for loop
        m_evtMutex.lock();
        for(int i=0; i<m_rows*m_cols; i++)
        {
            // Get the polarity of i-th event
            int pol = m_polEvts[i];

            // Fill the display matrix (color by polarity)
            if((m_currenTime-m_ageEvts[i])<m_thresh)
            {
                polMat.data[3*i + 0] = (unsigned char)0;              // Blue channel
                polMat.data[3*i + 1] = (unsigned char)(pol>0?255:0);    // Green channel
                polMat.data[3*i + 2] = (unsigned char)(pol<0?255:0);    // Red Channel
            }

            // Compute age of event and set a maximum age threshold.
            int dt = m_currenTime - m_ageEvts[i];
            if(dt>m_thresh) { dt = m_thresh; }

            // Fill the display matrix (color by age)
            ageMatHSV.data[3*i + 0] = (unsigned char)(0.75*180.*dt/float(m_thresh)); //H
            ageMatHSV.data[3*i + 1] = (unsigned char)255; //S
            ageMatHSV.data[3*i + 2] = (unsigned char)(dt==m_thresh?0:255); //V
        }
        m_evtMutex.unlock();

        // Display the "polarity" and "age" event matrices
        cv::imshow(m_polWin,polMat);
        cv::cvtColor(ageMatHSV,ageMatRGB,CV_HSV2BGR);
        cv::imshow(m_ageWin,ageMatRGB);
        cv::imshow(m_frameWin,m_grayFrame);

        // Reset the display matrices
        ageMatHSV = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);
        polMat = cv::Mat::zeros(m_rows,m_cols,CV_8UC3);
        //std::fill(m_evts.begin(),m_evts.end(),0);
        key = cv::waitKey(40);
    }
}
