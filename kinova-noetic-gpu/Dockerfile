FROM nvidia/cuda:12.1.0-devel-ubuntu20.04
CMD nvidia-smi


# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y git && apt update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# to have same alias as in local laptop
RUN echo "# alias " >> ~/.bashrc
RUN echo "alias sb='source ~/.bashrc' " >> ~/.bashrc
RUN echo "alias eb='gedit ~/.bashrc' " >> ~/.bashrc
RUN echo "alias cm='cd ~/catkin_ws/ && catkin_make' " >> ~/.bashrc
RUN echo "alias update='sudo apt update'" >> ~/.bashrc
RUN echo "alias upgrade='sudo apt upgrade'" >> ~/.bashrc
RUN echo "alias cw='cd ~/catkin_ws/ '" >> ~/.bashrc
RUN echo "alias c='clear'" >> ~/.bashrc
RUN source ~/.bashrc


# for ease of use 
RUN apt-get install -y gedit 
# cv related 

RUN apt-get -y install python3
RUN apt-get -y install python3-pip
RUN pip3 install torchvision 
RUN pip3 install matplotlib
RUN pip3 install opencv-python
RUN pip3 install numpy
RUN pip3 install scipy
RUN pip3 install scikit-image
RUN pip3 install scikit-learn
RUN pip3 install pandas
RUN apt-get install -y python3-tk
RUN pip3 install shapely
RUN pip install ultralytics
RUN pip install pyrealsense2


RUN mkdir -p ~/catkin_ws/src 
RUN cd ~/catkin_ws 
RUN . /opt/ros/noetic/setup.sh && cd ~/catkin_ws/ && catkin_make

RUN mkdir  ~/hello
RUN sudo apt-get install -y ros-noetic-realsense2-camera
RUN git clone -b pepper-people-devel https://github.com/GetAGrip-dot-Ai/realsense-ros.git &&\
cd realsense-ros/ &&\
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1` &&\
cd ..

RUN pip3 install conan==1.59 && conan config set general.revisions_enabled=1 && conan profile new default --detect > /dev/null && conan profile update settings.compiler.libcxx=libstdc++11 default 
RUN cd ~/catkin_ws/src && git clone -b noetic-devel https://github.com/Kinovarobotics/ros_kortex.git
# RUN cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -y 

RUN echo "source ~/catkin_ws/devel/setup.bash " >> ~/.bashrc &&source ~/.bashrc


# BUILD source 
RUN source /opt/ros/noetic/setup.bash

# Installing MoveIt
RUN sudo apt-get update
RUN sudo apt-get install -y ros-noetic-moveit

RUN apt-get -y update && apt-get -y upgrade && apt-get -y dist-upgrade
RUN apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
RUN apt-get install ros-noetic-moveit ros-noetic-moveit-plugins ros-noetic-moveit-planners ros-noetic-control-toolbox

CMD cm
