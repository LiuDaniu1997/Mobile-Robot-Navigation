# Mobile Robot Navigation
[![LinkedIn][linkedin-shield]][linkedin-url]

<!-- PROJECT LOGO -->
<br />
<p align="center">
   <img src="images/demo.gif" alt="Cover">
</p>


## Table of Contents
* [About the project](#about-the-project)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
* [Usage](#usage)
* [Rodamap](#roadmap)
* [Contributing](#contributing)
* [License](#license)
* [Contact](#contact)
* [Acknowledgements](#acknowledgements)

<!-- ABOUT THE PROJECT -->  
## About the Project
This repository contains simulations of mobile robots moving through indoor scenes in Gazebo. It includes building a mobile robot as well as sensor models, localizing the robot's position, and using Nav2 for indoor navigation.

<!-- GETTING STARTED -->
## Getting Started
### Prerequisites
This project is based on the C++ version of ROS2. To prepare your PC you need:
* Install Ubuntu 22.04 on PC or in Virtual Machine
Download the ISO [Ubuntu 22.04](https://ubuntu.com/download/) for your PC
* Install [ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your Ubuntu 22.04
* Install [Gazebo 11](https://classic.gazebosim.org/) on your Ubuntu 22.04
* Install libraries that are not included in the standard ROS 2 package. Install them with:
```sh
sudo apt-get update && sudo apt-get install -y \
     ros-humble-ros2-controllers \
     ros-humble-gazebo-ros \
     ros-humble-gazebo-ros-pkgs \
     ros-humble-ros2-control \
     ros-humble-gazebo-ros2-control \
     ros-humble-joint-state-publisher-gui \
     ros-humble-robot-localization \
     ros-humble-tf-transformations
```
* Install [Nav2](https://navigation.ros.org/getting_started/index.html#installation) on your Ubuntu 22.04

<!-- USAGE -->
## Usage
To Launch the Simulation of the Robot 
1. Clone the repo
```sh
git clone 
```
2. Build the ROS 2 workspace
```sh
colcon build
```
3. Source the ROS 2 Workspace
```sh
. install/setup.bash
```
4. Launch the Gazebo simulation
```sh
ros2 launch mrobot_bringup mrobot.launch.py
```
To launch Nav2
1. Open a new terminal and source the ROS2 Workspace
```sh
. install/setup.bash
```
2. Launch the Nav2
```sh
ros2 launch mrobot_nav2 mrobot_nav2.launch.py
```

<!-- ROADMAP -->
## Roadmap
TODOS:
 - Realization of VIO (Visual Inertial Odometry)
 - Project management with Docker

<!-- CONTRIBUTING -->
## Contributing


<!-- LICENSE -->
## License

Distributed under the Apache 2.0 License. See `LICENSE` for more information.


<!-- CONTACT -->
## Contact

[![LinkedIn][linkedin-shield]][linkedin-url]

<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements
* [Nav2](https://navigation.ros.org/index.html)



<!-- MARKDOWN LINKS & IMAGES -->
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/kai-liu-27749017b/