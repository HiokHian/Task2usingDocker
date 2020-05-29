# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:melodic-ros-core-bionic
MAINTAINER Hiok Hian

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# get linux tools
RUN apt-get update && apt-get install -y --no-install-recommends \
	apt-utils lsb-release sudo unzip wget ssh vim curl\
	&& rm -rf /var/lib/apt/lists/*

# install catkin
RUN apt-get update && apt-get install -y python-catkin-tools --no-install-recommends 

# install what is required to upload Arduino code from terminal
RUN apt-get install arduino-mk -y
    
# install rosserial
RUN apt-get install ros-melodic-rosserial-arduino -y \
    ros-melodic-rosserial

# copy all the files used for Task 2
COPY ./Task2FinalPython /usr/src/Task2FinalPython
COPY ./ToggleLEDArduinoSubscriberClient.ino /usr/src/sketchbook/
COPY ./Makefile /usr/src/sketchbook/

# change working directory to sketchbook
WORKDIR /usr/src/Task2FinalPython

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'




