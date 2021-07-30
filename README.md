# SMACC2

## dependencies and submodules
SMACC2 main branch is attached to the rolling ros2 version. Because of that, there are some dependencies to some packages that are not available via apt-get. The most important dependency is the navigation2 stack. Because of that we add that repo as a submodule and also there is a setup_build.bash

## basic build

Download the git repository in our workspace folder.
Then we invokde the rosdep command:
```
rosdep install --from-paths src --ignore-src -r -y
```
And then, the usual way to build the project is the following:

```
colcon build --cmake-args -D CMAKE_BUILD_TYPE=Debug
``` 


## basic example
To launch sm_dance_bot do the following:

```
ros2 launch sm_dance_bot sm_dance_bot_launch.py 
```
