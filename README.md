# Robotics PicoDegree

If you wish to get into shoes of a Robotics Software Engineer and see the complete cycle of mobile robot development, this is the right place. 
This project will help you learn and impelement robotics concepts using ROS with a great simulator named Webots. 
At the end of the tutorial series you will have a fully functional mobile robot which will be able to localize, navigate, avoid obstacles and a lot more. 

This PicoDegree will be delivered in the form of a YouTube videos series:

[Robotics PicoDegree YouTube Playlist](https://www.youtube.com/playlist?list=PLt69C9MnPchkrzYf68RvtIpd5bj7VCcwR)

## Hero of the movie
The tutorials use the following Stark Robot created by us.

![StarkRobot_features](https://user-images.githubusercontent.com/40532456/126117030-899f98e6-20e7-4a3e-94a1-f938d6471bbd.png)


You can either use the Stark robot created by us or create your own custom robot using the link below which creates a robot with features like GPS, IMU, Camera etc:

[YouTube playlist to create custom robot](https://www.youtube.com/playlist?list=PLt69C9MnPchlZl4FAZ7z-wBhFpUtwJvEG)
 
## The Seven Episodes 
Before starting the series it's good to get acquainted with ROS as well as Webots. 
The tutotial series has been divided into the following 7 videos:

![Untitled (1)](https://user-images.githubusercontent.com/40532456/126115492-5644ecb8-1135-4997-a214-d8bcbf84157a.png)

**1. Introduction video** - This video shows you how to get started with the github repo that we are developing, and installing the pre-requisites. It also explains how you can either create your own robot or use the Stark robot made by us.

**2. Static and Dynamic URDF** - This helps ROS understand the actual physical structure of the different links on the robot and where all the sensors and actuators are located.

 **3. Teleop** - This video will show how you can control your robot using the keyboard. The video also explains how Webots services is used to enable the different sensors, and command actuators using sensor values.
 
 **4. Mapping** - Here you will learn how to have your robot understand and get a feel of its environment using a LiDAR.
 
 **5. Navigate** - Once the robot undertands it's world, we can use different path planning and optimization algorithms to make it go from point A to point B. This will also include the configuration of costmap and different planners.
 
 **6. Obstacle Avoidance** - Now when the robot is traveling from point A to B we don’t want it crashing into obstacles or humans. Hence we need good obstacle avoidance algorithms. It will also plan a path around obstacles.
 
 **7. Applications** - Finally we will use all the above-learned knowledge to enable our robot to perform specific tasks/applications.


## To run docker instance of the project.

[Install docker](https://docs.docker.com/engine/install/ubuntu/)

[Install nvidia drivers for docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

```
xhost +local:root > /dev/null 2>&1
sudo docker run --gpus=all -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw softillusion/robotics_picodegree:latest
roslaunch bringup master.launch
```

## To run the project on native ubuntu machine.

```
mkdir -p ~/webots_ws/src 
cd ~/webots_ws/src && git clone https://github.com/Soft-illusion/Robotics_PicoDegree
git clone --recursive https://github.com/leggedrobotics/darknet_ros
```

Install rosdep to install necessary packages

```
sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y 
source /opt/ros/noetic/setup.bash
cd ~/webots_ws/src && catkin_make -DCMAKE_BUILD_TYPE=Release
```

Its time to run the package

```
source /opt/ros/noetic/setup.bash
source ~/webots_ws/devel/setup.bash
roslaunch bringup master.launch
```
