# Stereo-Event-Based-Reconstruction-with-Active-Laser-Features
[[Slides presentation](https://docs.google.com/presentation/d/1tXoAg83I_oNXGtQD3tcpglnKlZFLjmrbY52141NRQi4/edit?usp=sharing)]
[[Report](https://drive.google.com/file/d/1eHIvLH24uCGfen_UDQDYRRNrPE4c19DG/view?usp=sharing)]


Semester project carried out in V4RL & INI at ETHZ & UZH (Autumn 2018)
Author: Frédéric Debraine

Supervisors:
Dr. Yulia Sandamirskaya,
Prof. Dr. Margarita Chli,
Julien Martel,
Ignacio Alzugaray

![Demo](https://drive.google.com/uc?export=view&id=1EvW5rJT9xDO2pCx4hF6H0edUn1f78MSy)

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

This project requires to have OpenCV3 (with contribs) and Eigen3 installed on your machine. The other dependencies will be automatically installed when building the project.

If you do not have OpenCV3 installed, you can run the following commands:

```
git clone https://github.com/opencv/opencv.git && \
cd opencv && \
git checkout 3.4.0 && \
mkdir build && \
cd build && \
cmake 	-D CMAKE_BUILD_TYPE=RELEASE \
  -D BUILD_NEW_PYTHON_SUPPORT=ON \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D INSTALL_C_EXAMPLES=OFF \
  -D INSTALL_PYTHON_EXAMPLES=OFF \
  -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
  -D PYTHON_EXECUTABLE=/usr/bin/python2.7 \
  -D BUILD_EXAMPLES=OFF /opt/opencv && \
make -j $(nproc) && \
make install && \
ldconfig && \
apt-get purge -y git && \
apt-get clean && rm -rf /var/lib/apt/lists/* && \
rm -rf /opt/opencv*
```

### Installing

After cloning the repository, build the project by running the following commands:

```
cd davis-laser-tracking-depth
mkdir build && cd build
cmake ..
make -j8
```

### Running the code

```
sudo ./demo
```

### Troubleshoots

**Error opening /dev/ttyUSB0**: Make sure you connected the laser. You can check the device path in /dev/ and modify it if needed in the header EBV_LaserController.h.

**The depthmap is always empty**: Make sure the calibration .yaml files exist and change the location direction in the header EBV_Triangulator.h if necessary. You can also recalibrate the setup by launching the demo executable and press C  while having the calibration chessboard in front of the DAVIS (repeat until calibration parameters are computed, i.e. about 10 poses). It is important not to move the calibration target while the laser is moving. 

**The triangulated point cloud is too sparse**: Slightly defocus both cameras and turn the aperture diaphragm such as there are no saturated pixels in the frames. 
