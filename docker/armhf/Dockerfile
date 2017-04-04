FROM jeguzzi/ros:armhf-kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
   ros-kinetic-rosbridge-server \
   ros-kinetic-web-video-server \
   ros-kinetic-diagnostic-common-diagnostics \
   python-psutil \
   python-netifaces \
   python-pip

RUN pip install docker-py

RUN git clone https://github.com/tork-a/roswww.git src/roswww

RUN catkin build

COPY . src/ros_docker_ui

RUN catkin build
