#include <EBV_MagneticMirrorLaser.h>

#include <sstream>

MagneticMirrorLaser::MagneticMirrorLaser()
{}

MagneticMirrorLaser::~MagneticMirrorLaser()
{
    this->close();
}

int MagneticMirrorLaser::init(std::string device)
{
    m_device = device;

    struct termios tio;
    struct termios stdio;

    tcgetattr(STDOUT_FILENO,&m_oldStdio);

    memset(&stdio,0,sizeof(stdio));
    stdio.c_iflag=0;
    stdio.c_oflag=0;
    stdio.c_cflag=0;
    stdio.c_lflag=0;
    stdio.c_cc[VMIN]=1;
    stdio.c_cc[VTIME]=0;
    tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
    tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking

    memset(&tio,0,sizeof(tio));
    tio.c_iflag=0;
    tio.c_oflag=0;
    tio.c_cflag=CS8|CREAD|CLOCAL;           	// 8n1, see termios.h for more information
    tio.c_lflag=0;
    tio.c_cc[VMIN]=1;
    tio.c_cc[VTIME]=5;

    std::cout << "> Opening " << m_device << " on: " << std::endl;
    m_ttyFd = open(m_device.c_str(), O_RDWR | O_NONBLOCK);
    if(m_ttyFd < 0)
    {
        std::cout << "\t Error opening " << m_device << std::endl;
        return -1;
    }
    else
    {
        std::cout << "\t Opened " << m_device << " successfully on file descriptor: " << m_ttyFd << std::endl;
    }

    cfsetospeed(&tio,B115200);
    cfsetispeed(&tio,B115200);

    tcsetattr(m_ttyFd,TCSANOW,&tio);
    return 0;
}

int MagneticMirrorLaser::toggle(bool toggle)
{
    if(toggle)
        return sendCommand("L+\n");
    else
        return sendCommand("L-\n");
}

int MagneticMirrorLaser::blink(unsigned int us)
{
    unsigned int us10 = us / 10;

    std::ostringstream us10S;
    us10S << us10;

    std::string cmd("T");
    cmd.append(us10S.str().c_str())
        .append("\n");

    return sendCommand(cmd);
}

int MagneticMirrorLaser::pos(unsigned int x,unsigned int y)
{
    x = 4095 - x;
    y = 4095 - y;

    // Flip because the coordinate system is
    // < y, ^ x in the bottom right corner
    // and we want
    // y >, `' x in the top left corner
    std::ostringstream xS;
    xS << x;

    std::ostringstream yS;
    yS << y;

    std::string cmd("P");
    cmd.append(xS.str().c_str())
        .append(",")
        .append(yS.str().c_str())
        .append("\n");

    return sendCommand(cmd);
}

int MagneticMirrorLaser::vel(unsigned int x,unsigned int y)
{
    std::ostringstream xS;
    xS << x;

    std::ostringstream yS;
    yS << y;

    std::string cmd("V");
    cmd.append(xS.str().c_str())
        .append(",")
        .append(yS.str().c_str())
        .append("\n");

    return sendCommand(cmd);
}

void MagneticMirrorLaser::close(void)
{
    this->toggle(true);   // laser on
    this->vel(0,0);       // velocity 0
    this->pos(2048,2048); // neutral position
    this->blink(0);       // don't blink

    std::cout << "> Closing " << m_device << " on " << m_ttyFd << std::endl;
    if(::close(m_ttyFd) < 0)
        std::cout << "\t Error closing " << m_device << std::endl;
    else
        std::cout << "\t Closed " << m_device << " successfully" << std::endl;
    tcsetattr(STDOUT_FILENO,TCSANOW,&m_oldStdio);
}

int MagneticMirrorLaser::sendCommand(std::string cmd)
{
    // DEBUG Command
    //std::cout << "\t Input command: " << cmd.substr(0,cmd.length()-1) << std::endl;

    int w = write(m_ttyFd,
                  cmd.c_str(),
                  cmd.length());

    return !(w==cmd.length());
}
