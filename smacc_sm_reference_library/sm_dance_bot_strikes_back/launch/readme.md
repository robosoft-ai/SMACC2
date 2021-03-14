# Dance Bot State Machine
## Setup and requirements

Make sure that you have installed all the dependencies. For example this demo depends on rigeback robot simulation.

```
rosdep install --from-paths src --ignore-src -r -y 
```

After that you have to build the smacc ros workspace using catkin_make.

```
catkin_make
```

After you build, remember to source the proper devel folder.

```
source ~/catkin_ws/devel/setup.bash
```

## Default demo launching
Below is the default launch command to launch this demo:

```
roslaunch sm_dance_bot_strikes_back sm_dance_bot_strikes_back.launch
```

## Lightweight demo launching

We use this launch command to start the process hiding the xterminals for all the servers, the state machine, the smacc_viewer and the gazebo client.

```
roslaunch sm_dance_bot_strikes_back sm_dance_bot_strikes_back.launch sm_xterm:=nice server_nodes_xterms:=nice show_gz_client:=false show_smacc_viewer:=false
```
