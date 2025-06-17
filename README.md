# ROS1 Package for the VXS Sensor

A ROS1 package for publishing sensor data in various forms (depth image, pointcloud, event stream, etc.).

## Optional retrieval of docker container with ROS1 binaries and necessary depencies

The best way to obtain an environment that meets all depdencies folr the `vxs_sensor_ros1` package to build and run, is to directly download the pre-built docker container of an ubuntu 20.04 system with **ROS1 Noetic** installed.

### Install docker.io
First, install docker whether on [windows](https://docs.docker.com/desktop/setup/install/windows-install/) or [linux](https://docs.docker.com/engine/install/). In any case, it should be straightforward to do.

#### Pos-installation steps in the docker setup (Linux)
Create a docker user,

``sudo groupadd docker``

and add it to to sudoers:

``sudo usermod -aG docker $USER``

Restart the docker daemon for the changes to take effect:

``sudo systemctl restart docker``

Finally install the nvidia container toolkit (see [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) ). Configure repository first:

``curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list``

Then, update:

``sudo apt update``

Finally, install the container toolkit:

``sudo apt-get install -y nvidia-container-toolkit``

### Download vxs docker image
Now, you can download the **vxs docker image** with ros1 binaries, OpenCV, VXSDK, and other depedencies by executing on the command line,


for an AMD system with Nvidia hardware,

``docker pull terzakig/vxs_ros1:amd64``

for an AMD system **without** Nvidia hardware,

``docker pull terzakig/vxs_ros1:amd64_no_gpu``

This should take about an hour, depending on the connection. When done, verify that the docker container is there by executing,

``docker image list``

You should be able to see a container named `vxs_ros1` with a tag `amd64`. To run the container, use the bash script `run.sh` (or run_no_gpu.sh` if using the `amd64_no_gpu` image) provided in the `docker` directory of this repository. Use,

``./run.sh vxs_ros1:amd64``

for the `amd64` (with nvidia) image, or,

``./run_no_gpu.sh vxs_ros1:amd64``

for the `amd64_no_gpu` image.

Note that the script assumes that you have created directories `~/vxs_ws/catkin_ws`  and `~/sandbox` in the host (i.e., your machine) which will become shared between the container and the host. If these directories don't exist, the container will run anyway and it will create them on the host side as well. Not sure about this behavior in windows, as the paths are not structured the same way, but I am assuming the same will happen, just in an arbitrarry path related to the docker executable.

## ROS1 Workspace setup \& Build

### Setting up the workspace directories

First, create the appropriate workspace directories (if not there yet; if present, skip to the next section). In the home directory of the Ubuntu host (or someplace else if on Windows), 

``cd ~``

Create a **sandbox** directory that will be shared netween the host and the ROS2 docker container. It will be useful for storing data.

``mkdir sandbox``

Now create the **vxs  workspace** directory:

``mkdir vxs_ws && cd vxs_ws``

Create the **ROS1 (catkin) workspace** below:

``mkdir catkin_ws && cd catkin_ws``

``mkdir src``

### Download ROS1 Noetic packages that must be built from source, including the `vxs_sensor_ros1` package

To populate the workspace, you will need to execute a **rosinstall** script. The script is located in the `rosinstall/` directory of this repository. Thus, first clone the current repository inside `src`:

``cd src``

and clone,

``git clone https://github.com/VoxelSensors/vxs_sensor_ros1.git``

Then, copy the rosinstall script from the `vxs_sensor_ros1` repository into `src` as follows:

``cp vxs_sensor_ros1/rosinstall/rosinstall ./.rosinstall``

Now, execute the rosinstall 

``rosinstall .``

## Build the workspace

Move back to the ros workspace directory, i.e., `/home/vxs/vxs_ws/catkin_ws`:

``cd ..``

The `rosinstall` script should have installed all repositories correctly in the `catkin_ws/src`. To build everything, execute the following inside `catkin_ws`:

``catkin build``

Alternatively, provided that the ROS1 packages are built, then the `vx_sensor_ros1` package can be specifically built,

``catkin build vxs_sensor_ros1``

## Running the node

To run the `vxs_node` connect the sensor. You need to enable access to the USB:

``sudo chmod -R 7777 /dev/bus/usb/``

Now run the node with the following:

``rosrun vxs_sensor_ros1 vxs_node _config_json:=/home/vxs/vxs_ws/ros_ws/src/vxs_sensor_ros2/config/and2_median_golden.json _calib_json:=/home/vxs/vxs_ws/ros_ws/src/vxs_sensor_ros2/config/default_calib.json _fps:=20

### ROS publisher node (vxs_node) arguments

- **publish_depth_image (bool)** : Will force the node to initialize **frame-based** communications with the sensor and publish a *depth image* in topic `depth/image`. Note that this arguments will override setting **publish_events** to **true**.
- **publish_pointcloud (bool)**  : Will force the node to initialize **frame-based** communications with the sensor and publish a *pointcloud* in topic `pcloud/cloud`. It also overrides **publish_events** as above.
- **publish_events (bool)**      : If **publish_depth_image** or **publish_pointcloud** are not specified then setting this argument to **true** will force the node to initialize communications in **streaming mode** with the sensor. In this communications mode, the nose will publish a **stamped pointcloud** which will represent events (`XYZt`) in 3D space and time between two time instances defined by a a period `1000/fps (ms)` (see below about argument **fps**).      
- **fps (int)**                  : If using **frame-based mode** (see first two arguments), it specifies the frame-rate. For frame-based mode, then **valid fps values are 1, 15, 30, 60, 90, 180**. Otherwise, if the node is on **streaming mode**, then **fps** can have any positive value and will determine the **period throughout which it will capture events (i.e. `XYZt` data).
- **config_json (string)**       : The full path to the SDK configuration json.
- **calib_json (string)**        : The full path to the calibration json.
- **binning_amount (int)**              : (Filtering arg. 1). Default: 0
- **prefiltering_threshold (float)**    : (Filtering arg. 2). Default: 2.0
- **filterP1 (float)**                  : (Filtering arg. 3). Default: 0.1
- **temporal_threshold (int)**          : (Filtering arg. 4). Default: 4
- **spatial_threshold (int)**           : (Filtering arg. 5). Default: 10

**NOTE**: If none of the three first arguments that determine sensor communication mode are set, then the node will internally set **publish_depth_image** to **true**.


### Observe topics
You can now start a new docker window and observe the data in the ros topics published by the node (`/depth/image` and `/depth/camera_info`):

``rostopic list``

You should see something like,

![image](https://github.com/user-attachments/assets/1dd4a3a1-e3e3-4cdb-a967-a2315cd96a2e)

Note that the depth image is published as a 16-bit integer image. You can display that image with,

``rqt_image_view``

Another way is to print the cotents of topics, e.g.,

``rostopic echo /depth/camera_info``

or,

``rostopic echo /depth/image``

