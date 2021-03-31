#!/bin/bash
sudo apt-get update
sudo apt-get install -y python3-pip xterm python3-rosdistro python3-rosdep python3-vcstools
sudo pip3 install -U vcstool


echo "assumming we are located in the smacc folder"
mkdir -p ros2_repos 

# --------- SMACC REPOS ------------
echo "smacc2 repos"
vcs import ros2_repos < smacc2.repos
vcs pull ros2_repos

# --------- ROS2 REPOS ------------
echo "installing ros2 repos"
wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
vcs import ros2_repos < ros2.repos
vcs pull ros2_repos

# -------- NAVIGATION2 REPOS --------------
echo "installing navigation repos"
vcs import ros2_repos < submodules/navigation2/tools/underlay.repos 
vcs pull ros2_repos

# --------- MOVEIT REPOS ------------
echo "installing moveit repos"
wget https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos
vcs import ros2_repos < moveit2.repos
vcs pull ros2_repos

# -----------------------------------
rosdep install -r --from-paths . --ignore-src --rosdistro rolling -y