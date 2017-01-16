FROM ros:jade
MAINTAINER Jerome Guzzi "jerome@idsia.ch"


RUN apt-get update && apt-get install -y \
   ros-jade-rosbridge-server \
   ros-jade-roswww \
   ros-jade-web-video-server \
   python-psutil \
   python-netifaces \
   python-pip


RUN pip install docker-py

RUN apt-get install -y ros-jade-diagnostic-common-diagnostics

RUN mkdir -p /home/root/catkin_ws/src
COPY . /home/root/catkin_ws/src/ros_docker_ui

RUN /bin/bash -c '. /opt/ros/jade/setup.bash; catkin_init_workspace /home/root/catkin_ws/src; catkin_make -C /home/root/catkin_ws;'
RUN /bin/sed -i \
    '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/home\/root\/catkin_ws\/devel\/setup.bash"' \
    /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
