#include <EBV_DFF_Visualizer.h>

#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// Exporting depth maps into txt file
static void writeMatToFile(std::vector<float>& v, const std::string filename)
{
    std::ofstream fout(filename);
    for(size_t i=0; i<v.size(); ++i) { fout << v[i] << std::endl; }
    fout.close();
}

// Setting laser ROI with depthmap window
static void onMouse(int event, int x, int y, int, void* data)
{
    LaserController* laser = static_cast<LaserController*>(data);

    // Remapping (TODO: fix offset on screen)
    int y_bis = x*static_cast<int>(4000.f/240.f) - 200;
    int x_bis = y*static_cast<int>((3500.f/180.f)) + 700;

    if (event == cv::EVENT_LBUTTONDOWN)
    {
        printf("Set laser boundaries to xMin: %d, yMin: %d. \n\r",x_bis,y_bis);
        laser->setMinX(x_bis);
        laser->setMinY(y_bis);
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
        if (x_bis>laser->getMinX() && y_bis>laser->getMinY())
        {
            printf("Set laser boundaries to xMax: %d, yMax: %d. \n\r",x_bis,y_bis);
            laser->setMaxX(x_bis);
            laser->setMaxY(y_bis);
        }
    }
}

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

Visualizer::Visualizer(const int nbCams,
                       DAVIS240C* davis0, DAVIS240C* davis1,
                       Filter* filter0, Filter* filter1,
                       StereoCalibrator* calibrator,
                       Triangulator* triangulator,
                       LaserController* laser)
    : m_nbCams(nbCams),
      m_davis{davis0,davis1},
      m_laser(laser),
      m_filter{filter0,filter1},
      m_calibrator(calibrator),
      m_triangulator(triangulator)
{
    // Initialize data structure
    for (auto& v : m_pol_evts) { v.resize(m_rows*m_cols,false); }
    for (auto& v : m_filt_evts) { v.resize(m_rows*m_cols,0); }
    for (auto& v : m_age_evts) { v.resize(m_rows*m_cols,0); }
    for (auto& v : m_frame) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC1); }
    m_depthmap.resize(m_rows*m_cols,0);
    m_mask = cv::Mat::zeros(m_rows,m_cols,CV_8UC1);

    for (int idx=0; idx<2; idx++)
    {
        // Initialize davis
        if (m_davis[idx])
        {
            // Initialize windows
            m_pol_win[idx] = "Events by Polarity - " + std::to_string(idx);
            m_frame_win[idx] = "Frame - "  + std::to_string(idx);

            cv::namedWindow(m_pol_win[idx],
                            cv::WindowFlags::WINDOW_NORMAL |
                            cv::WindowFlags::WINDOW_KEEPRATIO |
                            cv::WindowFlags::WINDOW_GUI_EXPANDED);
            cv::moveWindow(m_pol_win[idx],idx*500,10);

            cv::namedWindow(m_frame_win[idx],
                            cv::WindowFlags::WINDOW_NORMAL |
                            cv::WindowFlags::WINDOW_KEEPRATIO |
                            cv::WindowFlags::WINDOW_GUI_EXPANDED);
            cv::moveWindow(m_frame_win[idx],idx*500,370);

            // Age trackbars
            cv::createTrackbar("Threshold",m_pol_win[idx],&m_age_thresh,m_max_trackbar_val,nullptr);

            // Listen to davis
            m_davis[idx]->init();
            m_davis[idx]->start();
            m_davis[idx]->registerEventListener(this);
            m_davis[idx]->registerFrameListener(this);
            m_davis[idx]->listen();
        }

        // Initialize filter
        if (m_filter[idx])
        {
            m_filter[idx]->registerFilterListener(this);

            // Initialize window
            m_filt_win[idx] = "Filtered Events - "  + std::to_string(idx);
            cv::namedWindow(m_filt_win[idx],
                            cv::WindowFlags::WINDOW_NORMAL |
                            cv::WindowFlags::WINDOW_KEEPRATIO |
                            cv::WindowFlags::WINDOW_GUI_EXPANDED);
            cv::moveWindow(m_filt_win[idx],idx*500,730);

            // Initialize filter tuning parameters
            m_filter_freq[idx] = m_filter[idx]->getFreq();
            m_filter_eps[idx] = m_filter[idx]->getEps();
            m_filter_etaInt[idx] = static_cast<int>(100*m_filter[idx]->getEta());
            m_filter_sigma[idx] = static_cast<AdaptiveFilter*>(m_filter[idx])->getSigma();
            m_filter_max_t[idx] = static_cast<AdaptiveFilter*>(m_filter[idx])->getMaxT();

            // Filter trackbars
            cv::createTrackbar("Filter frequency",m_filt_win[idx],&m_filter_freq[idx],1500,
                               &callbackTrackbarFilterFreq,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("Epsilon",m_filt_win[idx],&m_filter_eps[idx],20,
                               &callbackTrackbarFilterEps,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("Sigma",m_filt_win[idx],&m_filter_sigma[idx],100,
                               &callbackTrackbarFilterSigma,static_cast<void*>(m_filter[idx]));
            cv::createTrackbar("Eta",m_filt_win[idx],&m_filter_etaInt[idx],100,
                               &callbackTrackbarEta,static_cast<void*>(m_filter[idx]));
        }
    }

    // Initialize laser
    if (m_laser)
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
        cv::createTrackbar("Laser frequency",m_pol_win[0],&m_laser_freq,1500,
                           &callbackTrackbarLaserFreq,static_cast<void*>(m_laser));
        cv::createTrackbar("laserMinX",m_pol_win[0],&m_laser_boundaries[0],4000,
                           &callbackTrackbarLaserMinX,static_cast<void*>(m_laser));
        cv::createTrackbar("laserMaxX",m_pol_win[0],&m_laser_boundaries[1],4000,
                           &callbackTrackbarLaserMaxX,static_cast<void*>(m_laser));
        cv::createTrackbar("laserMinY",m_pol_win[0],&m_laser_boundaries[2],4000,
                           &callbackTrackbarLaserMinY,static_cast<void*>(m_laser));
        cv::createTrackbar("laserMaxY",m_pol_win[0],&m_laser_boundaries[3],4000,
                           &callbackTrackbarLaserMaxY,static_cast<void*>(m_laser));
        cv::createTrackbar("laserX",m_pol_win[0],&m_laser_pos[0],4000,
                           &callbackTrackbarLaserX,static_cast<void*>(m_laser));
        cv::createTrackbar("laserY",m_pol_win[0],&m_laser_pos[1],4000,
                           &callbackTrackbarLaserY,static_cast<void*>(m_laser));
    }

    // Initialize triangulator
    if (m_triangulator)
    {
        // Listen to triangulator
        m_triangulator->registerTriangulatorListener(this);

        // Initialize window
        m_depth_win = "Depth Map";
        cv::namedWindow(m_depth_win,
                        cv::WindowFlags::WINDOW_NORMAL |
                        cv::WindowFlags::WINDOW_KEEPRATIO |
                        cv::WindowFlags::WINDOW_GUI_EXPANDED);
        cv::moveWindow(m_depth_win,1200,0);
        cv::resizeWindow(m_depth_win,cv::Size(700,500));

        m_depth_inpainted_win = "Depth Map Inpainted";
        cv::namedWindow(m_depth_inpainted_win,
                        cv::WindowFlags::WINDOW_NORMAL |
                        cv::WindowFlags::WINDOW_KEEPRATIO |
                        cv::WindowFlags::WINDOW_GUI_EXPANDED);
        cv::moveWindow(m_depth_inpainted_win,1200,500);
        cv::resizeWindow(m_depth_inpainted_win,cv::Size(700,500));

        // Initialize matcher parameters
        m_matcherEps = m_triangulator->m_matcher->getEps();

        // Matcher trackbars
        cv::createTrackbar("Matcher eps",m_depth_win,&m_matcherEps,1e5,
                           &callbackTrackbarMatcherEps,static_cast<void*>(m_triangulator->m_matcher));

        // Depth trackbars
        cv::createTrackbar("minDepth",m_depth_win,&m_min_depth,100,nullptr);
        cv::createTrackbar("maxDepth",m_depth_win,&m_max_depth,100,nullptr);
        cv::createTrackbar("Blending",m_depth_win,&m_alphaInt,100,nullptr);

        // Laser
        cv::createTrackbar("Laser step",m_depth_win,&m_laser_step,300,
                           &callbackTrackbarLaserStep,static_cast<void*>(m_laser));
        cv::createTrackbar("Laser ratio",m_depth_win,&m_laser_ratio_int,300,
                           &callbackTrackbarLaserRatio,static_cast<void*>(m_laser));

        // Mouse
        cv::setMouseCallback(m_depth_win,&onMouse,static_cast<void*>(m_laser));
    }

    // Initialize calibrator
    if (m_calibrator)
    {
        m_lrInt = static_cast<int>(100*m_calibrator->getLearningRate());
        cv::createTrackbar("LearningRate",m_pol_win[0],&m_lrInt,100,
                           &callbackTrackbarLaserLearningRate,static_cast<void*>(m_calibrator));
    }
}


Visualizer::~Visualizer()
{
    for (int idx=0; idx<2; idx++)
    {
        if (m_davis[idx])
        {
            m_davis[idx]->stopListening();
            m_davis[idx]->deregisterEventListener(this);
            m_davis[idx]->deregisterFrameListener(this);
            m_davis[idx]->stop();
        }

        if (m_filter[idx])
        {
            m_filter[idx]->deregisterFilterListener(this);
        }
    }

    if (m_triangulator)
    {
        m_triangulator->deregisterTriangulatorListener(this);
    }

    if (m_laser)
    {
        m_laser->stop();
    }
}

// Runs in DAVIS240 thread
void Visualizer::receivedNewDAVIS240CEvent(DAVIS240CEvent& e,
                                           const int id)
{
    const int x = e.m_x;
    const int y = e.m_y;
    const bool p = e.m_pol; // p={0,1}
    const int t = e.m_timestamp;
    m_currenTime[id] = t;
    m_pol_evts[id][x*m_cols+y] = p;
    m_age_evts[id][x*m_cols+y] = t;
}

// Runs in DAVIS240 thread
void Visualizer::receivedNewDAVIS240CFrame(DAVIS240CFrame& f,
                                           const int id)
{
    auto frame = f.m_frame;
    const int t = f.m_timestamp;
    m_frame[id] = cv::Mat(m_rows,m_cols,CV_8UC1,frame.data());
    m_currenTime[id] = t;
}

// Runs in Filter thread
void Visualizer::receivedNewFilterEvent(DAVIS240CEvent& e,
                                        const int id)
{
    const int x = e.m_x;
    const int y = e.m_y;
    const int t = e.m_timestamp;
    m_filt_evts[id][x*m_cols+y] = t;
}

void Visualizer::receivedNewDepth(const int &u,
                                  const int &v,
                                  const double &X,
                                  const double &Y,
                                  const double &Z)
{
    // Update depth value
    const float depth = Z;
    const float last_depth = m_depthmap[u*m_cols+v];
    if (last_depth>0)
    {
        m_depthmap[u*m_cols+v] = 0.5*(last_depth + depth);
    }
    else
    {
        m_depthmap[u*m_cols+v] = depth;
    }
}

void Visualizer::run()
{
    // Wait (in ms)
    char key = ' ';

    // Initialize display matrices
    std::array<cv::Mat,2> polMat;
    std::array<cv::Mat,2> filtMat;
    cv::Mat depthMatHSV = cv::Mat(m_rows,m_cols,CV_8UC3);
    cv::Mat depthMatRGB = cv::Mat(m_rows,m_cols,CV_8UC3);
    cv::Mat depthMatRGBInpainted = cv::Mat(m_rows,m_cols,CV_8UC3);
    for (auto& v : polMat) { v = cv::Mat(m_rows,m_cols,CV_8UC3); }
    for (auto& v : filtMat) { v = cv::Mat(m_rows,m_cols,CV_8UC3); }

    while(key != 'q')
    {
        for(int i=0; i<m_rows*m_cols; i++)
        {
            // Updating matrix placeholders
            for (int idx=0; idx<2; idx++)
            {
                bool pol = m_pol_evts[idx][i];
                int dt = m_currenTime[idx] - m_age_evts[idx][i];

                // Events by polarity
                if(dt<m_age_thresh)
                {
                    polMat[idx].data[3*i + 0] = static_cast<uchar>(0);              // Blue channel
                    polMat[idx].data[3*i + 1] = static_cast<uchar>(pol>0?255:0);    // Green channel
                    polMat[idx].data[3*i + 2] = static_cast<uchar>(pol==0?255:0);    // Red Channel
                }
                else { dt = m_age_thresh; }

                // Filtered events
                dt = m_currenTime[idx] - m_filt_evts[idx][i];

                if(dt < m_age_thresh)
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

            // Update depthmap
            float z = m_depthmap[i];
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

        // Displaying matrices
        for (int idx=0; idx<2; idx++)
        {
            if (m_davis[idx])
            {
                // Display events by polarity
                cv::imshow(m_pol_win[idx],polMat[idx]);

                // Display frame
                cv::imshow(m_frame_win[idx],m_frame[idx]);
            }

            // Display filtered events + trackers
            if(m_filter[idx])
            {
                int x = m_filter[idx]->getX();
                int y = m_filter[idx]->getY();
                cv::circle(filtMat[idx],cv::Point2i(y,x),3,cv::Scalar(0,255,0));
                cv::imshow(m_filt_win[idx],filtMat[idx]);
            }
        }

        // Display depth map + Blending with master frame
        if (m_triangulator)
        {
            cv::cvtColor(depthMatHSV,depthMatRGB,CV_HSV2BGR);
            cv::Mat frameRGB;
            cv::cvtColor(m_frame[0], frameRGB, cv::COLOR_GRAY2BGR);
            cv::addWeighted(frameRGB, m_alphaInt/100., depthMatRGB, 1-m_alphaInt/100., 0.0, depthMatRGB);
            cv::imshow(m_depth_win,depthMatRGB);
        }

        // User keyboard input
        key = cv::waitKey(10);
        switch (key)
        {
        case 'c': // Calibrate cameras and laser
            if (m_calibrator && m_laser)
            {
                if(!m_laser->m_laser_on) { m_laser->start(); }
                printf("Starting laser calibration. \n\r");
                m_calibrator->calibrateCamLaser(m_frame[0],m_frame[1],0);
            }
            break;

        case 'd': // Print triangulated points coordinates
           printf("Switch debug mode. \n\r");
           m_triangulator->m_debug = !m_triangulator->m_debug;
           break;

        case 'e': // Export current depthmap as a .txt file
            static int nb_maps = 1;
            printf("Export new depthmap.\n\r");
            writeMatToFile(m_depthmap,m_depthmap_path + std::to_string(nb_maps) + ".txt");
            nb_maps++;
            break;

        case 'i': // Inpaint current depthmap
            printf("Inpainting. \n\r");
            cv::inpaint(depthMatRGB,m_mask,depthMatRGBInpainted,2,cv::INPAINT_NS);
            cv::imshow(m_depth_inpainted_win,depthMatRGBInpainted);
            break;

        case 'o': // Switch laser on/off
            if (m_laser) { m_laser->toogleState(); }
            break;

        case 'r': // Reset depthmap
            printf("Reset depth map.\n\r");
            m_depthmap.clear();
            m_depthmap.resize(m_rows*m_cols,0);
            break;


        case 's': // Switch sweeping on/off
            if (m_laser) { m_laser->toogleSweep(); }
            break;

        case 't': // Switch stereo pair
            m_triangulator->switchMode();
            break;

        case 'z': // Switch triangulator recording on/off
            if (m_triangulator->m_record==false)
            {
                m_triangulator->m_recorder.open(m_triangulator->m_eventRecordFile);
            }
            m_triangulator->m_record = !m_triangulator->m_record;
            printf("Switch triangulator recording on/off. \n\r");
            break;
        }

        // Reset the display matrices
        for (auto& v : polMat) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC3); }
        for (auto& v : filtMat) { v = cv::Mat::zeros(m_rows,m_cols,CV_8UC3); }
    }
}
