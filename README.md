# Stereo-Event-Based-Reconstruction-with-Active-Laser-Features

A Semester project carried out in V4RL & INI at ETHZ & UZH.
Author: Frédéric Debraine

Supervisors:
Dr. Yulia Sandamirskaya,
Prof. Dr. Margarita Chli,
Julien Martel,
Ignacio Alzugaray

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
