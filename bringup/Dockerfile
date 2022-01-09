#START WITH ROS BASE IMAGE
FROM cyberbotics/webots:R2021a-ubuntu20.04
ARG DEBIAN_FRONTEND=noninteractive

# setup sources.list
# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
RUN apt-get update
RUN apt-get install -y apt-utils git ssh nano

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update
RUN apt install -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt install -y ros-noetic-desktop-full

# Clone repos
RUN mkdir -p /usr/local/webots_ws/src
RUN cd /usr/local/webots_ws/src && git clone https://github.com/Soft-illusion/Robotics_PicoDegree
RUN cd /usr/local/webots_ws/src && git clone --recursive https://github.com/leggedrobotics/darknet_ros

# setup entrypoint
ENV ROS_DISTRO noetic
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

#ROS Dependacies
RUN apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
RUN rosdep init && rosdep update
RUN cd /usr/local/webots_ws && rosdep install --from-paths src --ignore-src -r -y 

# make workspace
COPY catkin_compile.sh /
RUN chmod +x /catkin_compile.sh
# RUN ./catkin_compile.sh
# RUN source /opt/ros/noetic/setup.bash && cd /usr/local/webots_ws/ && catkin_make -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /usr/local/devel/setup.bash" >> ~/.bashrc
