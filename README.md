# davis-laser-tracking-depth
Stereo Depth Estimation using Event-Based Cameras & Active Laser Features
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

### Installing

After cloning the repository, build the project by running the following commands:

```
cd davis-laser-tracking-depth
mkdir build && cd build
cmake ..
make -j8
make install
```

### Running the code

```
cd bin/Release
sudo ./demo
```
