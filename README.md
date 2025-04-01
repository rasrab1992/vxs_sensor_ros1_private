# ROS2 Package for the VXS Sensor

A ROS2 package for publishing sensor data in various forms (depth imagbe, pointcloud, event stream, etc.).

## Optional retrieval of docker container with ROS2 binaries and necessary depencies

The best way to obtain an environment that meets all depdencies folr the `vxs_sensor_ros2` package to build and run, is to directly download the pre-built docker container of an ubuntu 22.04 system with **ROS2 Humble** installed.

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
Now, you can download the **vxs docker image** with ros2 binaries, OpenCV, VXSDK, and other depedencies by executing on the command line,


for an AMD system with Nvidia hardware,

``docker pull terzakig/vxs:amd``

for an AMD system **without** Nvidia hardware,

``docker pull terzakig/vxs:amd_no_gpu``

This should take about an hour, depending on the connection. When done, verify that the docker container is there by executing,

``docker image list``

You should be able to see a container named `vxs` with a tag `amd`. To run the container, use the bash script `run.sh` (or run_no_gpu.sh` if using the `amd_no_gpu` image) provided in the `docker` directory of this repository. Use,

``./run.sh vxs:amd``

for the `amd` (with nvidia) image, or,

``./run_no_gpu.sh vxs:amd``

for the `amd_no_gpu` image.

Note that the script assumes that you have created directories `~/vxs_ws/ros_ws`  and `~/sandbox` in the host (i.e., your machine) which will become shared between the container and the host. If these directories don't exist, the container will run anyway and it will create them on the host side as well. Not sure about this behavior in windows, as the paths are not structured the same way, but I am assuming the same will happen, just in an arbitrarry path related to the docker executable.

## ROS2 Workspace setup \& Build

### Setting up the workspace directories

First, create the appropriate workspace directories (if not there yet; if present, skip to the next section). In the home directory of the Ubuntu host (or someplace else if on Windows), 

``cd ~``

Create a **sandbox** directory that will be shared netween the host and the ROS2 docker container. It will be useful for storing data.

``mkdir sandbox``

Now create the **vxs  workspace** directory:

``mkdir vxs_ws && cd vxs_ws``

Create the **ROS2 workspace** below:

``mkdir ros_ws && cd ros_ws``

``mkdir src``

### Download ROS2 packages that must be built from source, including the [vxs_sensor_ros2](https://github.com/VoxelSensors/vxs_ros_workspace_install) package

To populate the workspace, you will need to execute a **rosinstall** script. The script is located in the `rosinstall/` directory of this repository. Thus, first clone the current repository inside `src`:

``cd src``

and clone,

``git clone https://github.com/VoxelSensors/vxs_sensor_ros2.git``

Then, copy the rosinstall script from the `vxs_sensor_ros2` rpository into `src` as follows:

``cp vxs_sensor_ros2/rosinstall/rosinstall ./.rosinstall``

Now, execute the rosinstall 

``rosinstall .``

## Build the workspace

Move back to the ros workspace directory, i.e., `/home/vxs/vxs_ws/ros_ws`:

``cd ..``

The `rosinstall` script should have installed all repositories correctly in the `ros_ws/src`. To build everything, execute the following inside `ros_ws`:

``colcon build``

Alternatively, provided that the ROS2 packages are built, then the `vx_sensor_ros2` package can be specifically built,

``colcon build --packages-select vxs_sensor_ros2``

## Running the node

To run the `vxs_node` connecft the sensor. You need to enable access to the USB:

``sudo chmod -R 7777 /dev/bus/usb/``

Now run the node with the following:

``ros2 run vxs_sensor_ros2 vxs_node --ros-args -p "config_json:=/home/vxs/vxs_ws/ros_ws/src/vxs_sensor_ros2/config/and2_median_golden.json" -p "calib_json:=/home/vxs/vxs_ws/ros_ws/src/vxs_sensor_ros2/config/default_calib.json" -p "fps:=20"``

You can now start a new docker window and get observe the data in the ros topics published by the node (`/depth/image` and `/depth/camera_info`):

``ro2 topic list``

You should see something like,

![image](https://github.com/user-attachments/assets/1dd4a3a1-e3e3-4cdb-a967-a2315cd96a2e)

Note that the depth image is published as a 16-bit integer image. You can display that image with,

``ros2 run image_view image_view --ros-args --remap image:=/depth/image``

Another way is to print the cotents of topics, e.g.,

``ros2 topic echo /depth/camera_info``

or,

``ros2 topic echo /depth/image``

