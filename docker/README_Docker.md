Mutual Gaze Detector - Docker
==============

## About

All the relevant files can be found in the `docker` folder.
We provide Dockerfiles for 2 separate images:
* [Dockerfile_full](#dockerfile_full): builds an image that allows to replicate our entire setup and run our code
* [Dockerfile_test](#dockerfile_test): builds an image that allows to test and visualize the detector on a recorded rosbag

We release the Dockerfiles for convenience, but images are available to be pulled from docker repository. The `docker-compose` files are already configured to pull these containers, so  there is no need to build the images from the Dockerfiles.


## Dockerfile_full

This Dockerfile builds an image that allows to replicate our entire setup and run our code. In particular, it will install them Azure Kinect ROS driver and the rest of our code.

### Requirements
This image has been tested succesfully only on Ubuntu 22.02. We could not test it on Windows since there is no easy way, at the time of writing, to make the Kinect available in a docker container from a Windows host.

The body tracking of the Azure Kinect requires a GPU to work properly. Check the [kinect requirements](https://learn.microsoft.com/en-us/azure/kinect-dk/system-requirements).
So, a GPU and a docker installation supporting CUDAs are required. We tested it following [this guide](https://linuxhint.com/use-nvidia-gpu-docker-containers-ubuntu-22-04-lts/).

Finally, [docker compose](https://docs.docker.com/compose/install/) must be installed to easily run our code.

Of course, this setup requires an Azure Kinect camera.

### docker-compose
The `docker-compose.yml` file available [here](docker-compose.yml) launches all the nodes required to run (I) data acquisition (i.e. Kinect driver); (II) data preprocessing and feature extraction; (III) mutual gaze detection.

First, we need to pull the docker image. To do so, run:

```
docker compose -f docker-compose.yml pull
```

This may take a while. Then, to start the entire pipeline, first run:
```
xhost +local:docker
```
This is necessary to map a display inside the container. Without this, the kinect driver won't work and the container won't be able to visualize GUIs.

Finally, launch:
```
docker compose -f docker-compose.yml up
```
If everything works correctly, one should be able to visualize a live plot of the mutual gaze probability for each person detected in the field of view of the sensor, like in the screenshot.

## Dockerfile_test
This Dockerfile builds an image that allows to test and visualize the detector on a recorded rosbag. The rosbag within the container already contains preprocessed data. In this way, this container does not have any hardware requirement, so we can easily visualize the mutual gaze detector on any PC.

This container was tested on Ubuntu 22.04 host machine running docker `20.10.7`. It was also tested on Windows 10 Enterprise 21H1 running `Docker Desktop 4.1.1` with `WSL 2` backend. We were not able to test on `macOS`.
Here we report the steps needed to run the containers on both tested OS.

### docker-compose setup

<details>
  <summary>UBUNTU</summary>
  
  1. Install [docker](https://docs.docker.com/get-docker) and [docker-compose](https://docs.docker.com/compose/install/). Check also this [post-install paragraph](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) to manage docker as a non-root user (otherwise remember to run all docker commands with `sudo`).
  2. Clone this repository locally.
  3. Step into the `docker` folder of the repository and pull the container:
   ```
   cd REPO_ROOT_FOLDER/docker
   docker compose -f docker-compose-test.yml pull
   ```
   
</details>

<details>
  <summary>WINDOWS</summary>
  
  1. Install [docker](https://docs.docker.com/desktop/windows/install/) using `Windows Subsystem for Linux (WSL2)` backend. `docker-compose` is included in this installation.
  2. Install [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/). This is needed to get GUI apps running from linux containers.
  3. Open a WSL terminal (search for WSL in windows). Note that everything will work from a regular/PowerShell terminal as well, but the syntax may differ from what we report in the readme (which is linux based), in particular when setting environmental variables.
  4. Clone this repository locally.
  5. Step into the `docker` folder and then in each subfolder to pull the container for each component.
   ```
   cd REPO_ROOT_FOLDER
   cd docker
   docker compose -f docker-compose-test-windows.yml pull
   ```
   
</details>


### Running the test
First, we map a display within the container.

<details>
  <summary>UBUNTU</summary>

Simply run:
```
xhost +local:docker
```

</details>

<details>
  <summary>WINDOWS</summary>

1. Start XLaunch
2. Leave default settings and press `Next` (Multiple windows, Display number -1)
3. Leave default settings and press `Next` (Start no client)
4. Remove flag from `Native opengl` and press `Next`
5. Press `Finish`
</details>

Then, run start the appropriate `compose` file:
```
# Ubuntu
docker compose -f docker-compose-test.yaml up

# Windows
docker compose -f docker-compose-test-win.yaml up
```

If everything worked, you should see an `rqt` window where you should be able to select the appropriate topic to subscribe. Also, you should see a plot with the mututal gaze detector output.

After resizing both windows, you should see a video being played in loop, like in the image below.

