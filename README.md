Mutual Gaze Detector
==============

This repository contains a ROS2 implementation of a mutual gaze detector designed for Human Robot Interaction applications which requires reasonable tracking up 5 meters.

[![Mutual Gaze Detector at Work](https://github.com/idsia-robotics/mutual_gaze_detector/blob/hri/assets/readme.gif)](https://raw.githubusercontent.com/idsia-robotics/mutual_gaze_detector/hri/assets/readme.mp4)

We provide two ways of installation: from scratch on native Ubuntu 22.04 or Docker images to simplify the setup procedure.

## System Hardware Requirements
The execution of the code contained in this repository has the following hardware requirements:
* Seventh Gen Intel® CoreTM i5 Processor (Quad Core 2.4 GHz or faster)
* Nvidia GPU (NVIDIA GEFORCE GTX 1050 or above)

These requirements are inherited by the [Azure Kinect sensor SDK system requirements](https://learn.microsoft.com/en-us/azure/kinect-dk/system-requirements).

## Dockerized Solutions
Two distinct Docker images are released. The first image can be used to create a container for running a demo to test the mutual gaze detector on a pre-recorded rosbag file.

This Demo Docker setup has been tested with two different OS:
* Windows, by using Windows Subsystem for Linux (WSL2)
* Ubuntu 22.04 LTS

This image allows to display the capabilities of the software without needing specific hardware requirement (e.g. Azure Kinect Camera and GPU) as the body tracking is not running online.

The second one is instead already set up to run the full pipeline provided that the right hardware is available.
For this second setup, the host has the requirement to run a Linux distro. This is due to some known issues in Windows in the connection to a Azure Kinect from a process running inside a Docker container.

[Here](docker/README.md) is reported a more detailed and comprehensive documentation for the Docker part.

## Native Installation

### Host System Software Requirements
The code in this repository has been developed and tested with a machine running:
* Ubuntu 22.04 LTS (Kernel Version: 6.1.0-1013-oem)
* ROS2 Humble
* Python 3.10.12
  
Therefore these are to be considered the software requirements. The kernel version is provided only for reference, the specific number is not mandatory as it should not hinder the code execution.

### Installation Guide
#### ROS2 General Installation

Install Humble desktop full version of ROS2, following the [official instructions](https://docs.ros.org/en/humble/Installation.html).
and then install colcon  
```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```

#### ROS2 Workspace Setup

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

###### Additional dependencies
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
colcon build --symlink-install --packages-select users_landmarks_tracking mutual_gaze_detector
```
when in the base folder of the ROS2 workspace.

### Code Usage
Once all the pre-requisites are installed and the ROS2 packages built. Running the whole pipeline comes down to launching three different groups of nodes.
Firstly open three terminal and run in each of them
```bash
source install/setup.bash
source <ABS_PATH_VENV_BASE_FOLDER>/bin/activate
```
from the base folder of the ROS2 workspace.

In the first terminal launch the sensor driver node (camera drivers together with body tracking) using
```bash
ros2 launch azure_kinect_ros_driver driver.launch.py depth_mode:=NFOV_UNBINNED color_resolution:=3072P fps:=15 body_tracking_enabled:=true body_tracking_cpu:=false rectify_images:=false imu_rate_target:=100
```

In the second terminal launch the pre-processing pipeline nodes (face landmarks and gravity aligning nodes) with 
```bash
ros2 launch users_landmarks_tracking users_landmarks_tracking.launch.xml rectified_image_input:=false debug_node:=true
```

In the last terminal launch the actual mutual gaze detector node with 
```bash
ros2 launch mutual_gaze_detector_ros mutual_gaze_detector_ros.launch.xml
```
this last command will cause an OpenCV GUI to open where you will be able to see on top a simple plot of the predicted mutual gaze probability history for the first tracked user. On the bottom there will be the actual value of the probability in a color changing box. The box will be green if the probability exceed the set threshold of 0.5 and red if down not.    

The mutual gaze detector node also publish the output of the detector on a ROS2 topic called "/mutual_gaze_output" to be further used in downstream applications. The message publish is of the custom type [MutualGazeOutput](ros/mutual_gaze_detector_msgs/msg/MutualGazeOutput.msg).

Optionally the use can visualize the camera stream images with superimposed landmarks by using RQT gui with 
```bash
rqt
```
by using `Plugin/Visualization/Image View` plugin and then, in the top down menu, selecting the `/face_landmarks_node/full_landmarks_debug_image` topic.

## Demo
Please refer to the [Docker solution documentation](docker/README.md) and in particular to the Dockerfile_test part to read how run a demonstration of the code. 

## Remarks on Code Deployment

The pre-processing stage in this pipeline is designed to generate meaningful data to be used in downstream tasks, while making them anonymous. This is achieved by using human joints pose tracking and facial landmarks tracking instead of raw RGB images. This should facilitate acceptance of this solution by ethical commitees even in case data needs to be saved. 

## Maintenance

Please note that while this release provides the current version of the code utilized in the referenced paper, ongoing development might come at a later stage, as well as bug fixes, and updates to the documentation.Feedback and contributions are appreciated to shape the future iterations of this code.

## License

The code in this repository is released under the MIT License. This grants users the freedom to utilize, modify, and distribute the code, subject to the terms outlined in the MIT License agreement. We encourage collaboration and innovation, welcoming contributions and adaptations while ensuring the code's accessibility and continued evolution within the guidelines of this license.
