#!/usr/bin/env bash

IMAGE_NAME=hrilab/ubuntu-18.04-opencv4-devel


## Check args
#if [ "$#" -lt 1 ]; then
#  echo "usage: ./run.sh IMAGE_NAME"
#  exit 1
#fi

#set -e

#IMAGE_NAME=$1 && shift 1

BASEDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

THIS_HOST=`hostname`
docker_run_cmd="docker run --rm"

# Run the container with NVIDIA Graphics acceleration, shared network interface, shared hostname, shared X11
$(echo $docker_run_cmd) \
    --net=host \
    --ipc=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e "QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority -e XAUTHORITY=/root/.Xauthority \
    -v $BASEDIR:/home/docker/code \
    -v $HOME/.clang-format-v6:/home/docker/.clang-format \
    -v $HOME/.bash_aliases:/home/docker/.bash_aliases \
    -v $HOME/.bashrc:/home/docker/.bashrc \
    -it $IMAGE_NAME "$@"

