#!/bin/bash

# Defaults
OFFLINE=false
DOCKER_IMAGE=""
ROS1_WORKSPACE="/home/$USER/vxs_ws/catkin_ws"

function DisplayHelp() {
  echo "Usage:"
  echo "     run.sh -h               show this help message"
  echo "     run.sh <image>"
}

# Parse arguments

# Process the first argument
OPTIND=1

DOCKER_IMAGE=$1
if [ "$1" = "-h" ]; then
  DisplayHelp
  exit 0
elif [ "$DOCKER_IMAGE" = "" ]; then
  # First param is always the image name.
  echo "Please specify the docker image that has to be run."
  DisplayHelp
  exit 1
fi

# Check the local workspace exists, else we get weird behaviour.
if [ ! -d "$ROS1_WORKSPACE" ]; then
  echo "Catkin workspace '$ROS1_WORKSPACE' does not exist."
fi

# Shift to the next argument (TODO: extra drive mappings)
shift
#MAPPINGS = $1

# Set the hostname for when in the container (append by .local) 
CONTAINER_HOSTNAME=$(hostname -s).local

# Store container history
touch $CATKIN_WORKSPACE/.bash_history

# Prevent docker from creating a folder
touch /home/${USER}/.bash_aliases

# Prevent docker from making the ccache for you
[[ -d /home/${USER}/.ccache ]] || mkdir /home/${USER}/.ccache

# yield permissions to the xhost (see: http://wiki.ros.org/docker/Tutorials/GUI , section 1. "The simple way")
xhost +local:

docker run -it --privileged --env="DISPLAY" --gpus all \
  --init \
  --ulimit rtprio=50 \
  --env="HOSTNAME=$CONTAINER_HOSTNAME" \
  --env "NVIDIA_DISABLE_REQUIRE=1" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="USER=$USER" \
  --env="ROS_HOSTNAME=$CONTAINER_HOSTNAME" \
  --net=host \
  --volume="/dev:/dev" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
  --volume="/sys:/sys" \
  --volume="/tmp:/tmp" \
  --volume="/home/$USER/.config:/home/vxs/.config" \
  --volume="/home/$USER/.ssh:/home/vxs/.ssh" \
  --volume="/home/$USER/.ccache:/home/vxs/.ccache" \
  --volume="/home/$USER/.bash_aliases:/home/vxs/.bash_aliases" \
  --volume="/home/$USER/sandbox:/home/vxs/sandbox" \
  --volume="/home/$USER/vxs_ws:/home/vxs/vxs_ws" \
  --volume="/home/$USER/sandbox:/home/vxs/sandbox" \
  --volume="/home/$USER/.bash_history:/home/vxs/.bash_history" \
  --volume="/dev:/dev" \
  -u $(id -u):$(id -g) \
  ${DOCKER_IMAGE} fixuid -q sh -c "cd ~/;exec bash -l"

# revoke access controls (see: http://wiki.ros.org/docker/Tutorials/GUI , section 1. "The simple way")
xhost -local:
