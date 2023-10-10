Mutual Gaze Detector
==============

This repository contains a ROS2 implementation of a mutual gaze detector designed for Human Robot Interaction applications.

https://github.com/idsia-robotics/mutual_gaze_detector/assets/20441485/e95dc618-c3be-4af8-9e8e-b94049df0757



## Installation

We provide both instructions to install everything from scratch and docker containers to simplify the setup.

The documentation for the docker part can be found [here](docker/README.md).
Note that one of the container has been created to test the mutual gaze detector on a pre-recorded rosbag file.
In this way, one can see the detector at work without any hardware requirement (e.g. Azure Kinect Camera and GPU).

### Pre-requisites

#### Host

The code in this repository has been tested with a machine running:
* Ubuntu 22.04 LTS 
* Kernel Version: 6.1.0-1013-oem
* ROS2 Humble
* Python 3.10.12

#### ROS2

Install Humble desktop full version of ROS2, following the [official instructions](https://docs.ros.org/en/humble/Installation.html).
and then install colcon  
```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```

### Setup

Create a ROS2 workspace and clone this repo with
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src 
git clone git@github.com:idsia-robotics/mutual_gaze_detector.git
cd ..
```

Source your ROS2 installation manually if it's not in your .bashrc file with 
```bash
source /opt/ros/humble/setup.bash
```

All the next instruction assumes you have a terminal opened in the base folder of the ROS2 workspace.

#### Additional dependencies
Install Azure Kinect SDK by using the script in `config/k4a_install_on_ubuntu_22_04.sh` relative path with this repo with 
```bash
chmod +x ./src/mutual_gaze_detector/config/k4a_install_on_ubuntu_22_04.sh
./src/mutual_gaze_detector/config/k4a_install_on_ubuntu_22_04.sh
```

You also need to install PyKDL and tf_transformation dependencies with
```bash
sudo apt-get update
sudo apt-get -y install python3-pykdl 
sudo apt install ros-humble-tf-transformations
```

#### Python environment

The Python version used to develop this packages is 3.10.12 which is the default choice in Ubuntu 22.04 LTS.

The user is adviced to install the required Python packages inside a virtual environment, if you decide to do so please create it OUTSIDE the ROS2 environment. 

The Python dependencies are listed in the file `config/requirements.txt` and can be installed with 
```bash
pip install -r ./src/mutual_gaze_detector/config/requirements.txt
```

An additional custom package has to be installed with
```bash
pip install ./src/mutual_gaze_detector/non_ros/users_landmarks_utils
```

If you decided to use a virtual environment you will have to either add this command
```bash
PYTHONPATH=$PYTHONPATH:<ABS_PATH_VENV_BASE_FOLDER>/lib/python3.10/site-packages
```
to the end of `<ABS_PATH_VENV_BASE_FOLDER>/bin/activate` file or manually run it every time a terminal needs to run something which depends on the custom library.


### Build

#### Building Azure Kinect ROS driver

In order to build the ROS drivers for the Azure Kinect use 
```bash
colcon build
```
when in the base folder of the ROS2 workspace.

#### Mutual gaze detector packages

In order to build the custom packages for mutual gaze detection use 
```bash
colcon build --symlink-install --packages-select users_landmarks_tracking mutual_gaze_detector_ros
```
when in the base folder of the ROS2 workspace.

### Use

To use the packages open three terminal and run in each of them
```bash
source install/setup.bash
source <ABS_PATH_VENV_BASE_FOLDER>/bin/activate
```
when in the base folder of the ROS2 workspace.

In the first launch the sensor driver using
```bash
ros2 launch azure_kinect_ros_driver driver.launch.py depth_mode:=NFOV_UNBINNED color_resolution:=3072P fps:=15 body_tracking_enabled:=true body_tracking_cpu:=true rectify_images:=false imu_rate_target:=100
```

In the second launch the face landmarks processing pipeline with 
```bash
ros2 launch users_landmarks_tracking users_landmarks_tracking.launch.xml rectified_image_input:=false debug_node:=true
```

In the last launch the actual mutual gaze detector node with 
```bash
ros2 launch mutual_gaze_detector_ros mutual_gaze_detector_ros.launch.xml
```
this last command will cause an OpenCV GUI to open where you will be able to see on top a simple plot of the predicted mutual gaze probability history for the first tracked user. On the bottom there will be the actual value of the probability in a color changing box. The box will be green if the probability exceed the set threshold of 0.5 and red if down not.    

Optionally the use can visualize the camera stream images with superimposed landmarks by using RQT gui with 
```bash
rqt
```
by using `Plugin/Visualization/Image View` plugin and then, in the top down menu, selecting the `/face_landmarks_node/full_landmarks_debug_image` topic.
