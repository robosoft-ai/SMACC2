sudo apt-get install xterm
echo "assumming we are located in the smacc folder"
cd dependencies
vcs import < ../submodules/navigation2/tools/underlay.repos 
cd ..
rosdep install -q -y --from-paths . --skip-keys "slam_toolbox"


sudo apt-get install ros-rolling-cyclonedds ros-rolling-rmw-cyclonedds-cpp
