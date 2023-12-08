Mutual Gaze Detector - Docker
==============

## Description
We provide Dockerfiles for 2 separate images:
* [Dockerfile_full](#dockerfile_full): builds an image that allows one to replicate our entire setup and run our code
* [Dockerfile_test](#dockerfile_test): builds an image that allows to test and visualize the detector on a recorded rosbag

We release the Dockerfiles for convenience, but images are available to be pulled from docker repository. The `docker-compose` files are already configured to pull these containers, so  there is no need to build the images from the Dockerfiles.

## Dockerfile_full

This Dockerfile builds an image that allows one to replicate our setup and run our entire pipeline. In particular, it will install the Azure Kinect ROS driver and the rest of our code.

### docker-compose
The `docker-compose.yml` file available [here](docker-compose.yml) launches all the nodes required to run (I) data acquisition (i.e. Kinect driver); (II) data preprocessing and feature extraction; (III) mutual gaze detection.

First, we need to pull the Docker image. To do so, clone this repository and run:

```
git clone -b hri https://github.com/idsia-robotics/mutual_gaze_detector.git
cd mutual_gaze_detector/docker
docker compose -f docker-compose.yml pull
```

This may take a while. Then, to start the entire pipeline, first run:
```
xhost +local:docker
```
This is necessary to map a display inside the container. Without this, the Kinect driver won't work and the container won't be able to visualize GUIs.

Finally, launch:
```
docker compose -f docker-compose.yml up
```
If everything works correctly, one should be able to visualize a live plot of the mutual gaze probability for each person detected in the field of view of the sensor, like in the screenshot.

## Dockerfile_test
This Dockerfile builds an image that allows testing and visualizing the detector on a recorded rosbag. The rosbag within the container already contains preprocessed data. In this way, this container does not have any hardware requirement, so we can easily visualize the mutual gaze detector on any PC.

This container was tested on an Ubuntu 22.04 host machine running docker `24.0.2`. It was also tested on Windows 10 Enterprise 21H1 running `Docker Desktop 4.24.1` with `WSL 2` backend. We were not able to test on `macOS`.
Here we report the steps needed to run the containers on both tested OS.

### docker-compose setup

<details>
  <summary>UBUNTU</summary>
  
  1. Install [docker](https://docs.docker.com/get-docker) and [docker-compose](https://docs.docker.com/compose/install/). Check also this [post-install paragraph](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) to manage docker as a non-root user (otherwise remember to run all docker commands with `sudo`).
  2. Clone this repository locally.
  3. Step into the `docker` folder of the repository and pull the container:
   ```
   git clone -b hri https://github.com/idsia-robotics/mutual_gaze_detector.git
   cd mutual_gaze_detector/docker
   docker compose -f docker-compose-test.yml pull
   ```
   
</details>

<details>
  <summary>WINDOWS</summary>
  
  1. Install [docker](https://docs.docker.com/desktop/windows/install/) using `Windows Subsystem for Linux (WSL2)` backend. `docker-compose` is included in this installation.
  2. Install [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/). This is needed to get GUI apps running from Linux containers.
  3. Open a terminal.
  4. Clone this repository locally.
  5. Step into the `docker` folder and then in each subfolder to pull the container for each component.
   ```
   git clone -b hri https://github.com/idsia-robotics/mutual_gaze_detector.git
   cd mutual_gaze_detector
   cd docker
   docker compose -f docker-compose-test-win.yml pull
   ```
   
</details>


### Running the test

<details>
  <summary>UBUNTU</summary>

First, we map a display within the container. Run:
```
xhost +local:docker
```
Then, start the appropriate `compose` file:
```
docker compose -f docker-compose-test.yml up
```

</details>

<details>
  <summary>WINDOWS</summary>

First, we map a display within the container.
1. Start XLaunch
2. Leave default settings and press `Next` (Multiple windows, Display number -1)
3. Leave default settings and press `Next` (Start no client)
4. Remove the flag from `Native opengl` and press `Next`
5. Press `Finish`

Then, start the appropriate `compose` file:
```
docker compose -f docker-compose-test-win.yml up
```

</details>

If everything works, you should see a `rqt` window where you should be able to select the appropriate topic to subscribe to. Also, you should see a plot with the mutual gaze detector output.

After resizing both windows, you should see a video played in a loop, like the one below.


[![Mutual Gaze Detector test from docker](https://github.com/idsia-robotics/mutual_gaze_detector/blob/hri/assets/readme_docker.gif)](https://raw.githubusercontent.com/idsia-robotics/mutual_gaze_detector/hri/assets/readme_docker.mp4)
