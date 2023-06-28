## Pre Reqs
From https://www.ibm.com/docs/en/maximo-vi/8.2.0?topic=planning-installing-docker-nvidia-docker2

Make sure you have Docker installed...
```
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu bionic stable"
sudo apt-get update
sudo apt-get install docker-ce
```
Make sure you have nvidia-docker2 installed

```
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install nvidia-docker2
sudo systemctl restart docker.service
```
For each userid that will run Docker®, add the userid to the docker group:
```
sudo usermod -a -G docker <userid>
```
Then test..
```
sudo nvidia-docker run --rm nvidia/cuda:10.2-base-ubuntu18.04 nvidia-smi
```
## Unreal Engine Docker Image

This repository provides a prebuilt Docker image containing Unreal Engine. Follow the instructions below to download, load, and run the image.

### Downloading the Prebuilt Image

Download the prebuilt Docker image for Unreal Engine:


### Loading the Docker Image

Load the downloaded image into your Docker image database using the following command:

```
sudo docker load -i ue_editor_rclue.tar
```


### (Alternative) Building docker image locally

In the case you want to change the docker image via Dockerfile you will need to rebuild the docker image.
Before doing that you need to make a link to the UE5.1 folder into sm_dancebot_ue/docker folder.

```
mount --bind <UE5.1DIR> UE5.1/UnrealEngine
```

Then you can build the image locally. You can do that like this:

```
./build_docker_humble.sh
```

## Using the docker Image

### Running/Creating a new container from the ue_editor_rcl docker image

To run the unreal editor inside the container we will need some auxiliar scripts that are located in the sm_dancebot_ue example, first, download the current SMACC2 repository. Then, navigate to the `sm_dancebot_ue/docker` folder and execute the following command:

```
./run_docker_container_editor.sh
```

The Unreal Engine editor will automatically open in "edition mode." By clicking the "play" button, you can launch the simulation with Turtlebot topics accessible from both the container and the host computer.

When you close the editor, the container also will finish, but the container "keeps installed", you can reopen again the editor using the command:

```
./start_container.sh
```

### (Alternative) Running/Creating a new container for container debugging

There is a secondary way of creating the container in a daemon mode so that the lifetime of the container is not tied to the unreal editor window. This is useful specially for developing new features for the container. To create the container in this mode you have to run:

```
./run_docker_container_bash.sh
```

This will create and start a new container as a daemon (should be available on restarting). This container is able to open de unreal engine editor with ROS2, but the editor will not open until you run the following command:

```
./join_editor.sh
```

The Unreal Engine editor will automatically open in "edition mode." By clicking the "play" button, you can launch the simulation with Turtlebot topics accessible from both the container and the host computer.


### Stopping and removing running container

You may want to reset everyting if you messed the container. Before you create a new container you need
to stop and remove the existing one.

```
./stop_container.sh
./remove_container.sh
```


### Join to the container via bash
You may need to enter into the docker container to debug, or test stuff from commandline.
Then you can use the following command:

```
./join_bash.sh
```


## Connecting the container to VPN
Prototype already done -> todo DOC


### Important notes about the solution

*Repos versioning*
Not all the versions of rclUE turtlebot3Editor and RapyputaPlugins are compatible to each other. 
Keeping them consistent is not always easy, specially taking into account that the container maps/volumes some volumes with external contents.

The container is already designed to have a correct combination of all of them, however this is hardcoded in the Dockerfile and it could be improved.

*DDS Configuration*
We noticed problems communicating the docker container nodes with host nodes. At the current moment the workaround we  have for a correct
communication is using in the host cyclonedds (Until we find the causes and the solution to have everything uniform using fastrpts).
To do that, add to the .bashrc file in the host the following line:

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```


### Optionally Rebuild and run a smacc state machine inside the container

Notice that ue_editor_rcl containers are run mapping the current smacc2 source folder so it is possible to do a mixed development between the host (to develop from vscode) and the container (to compile and run the state machines).

Smacc2 source code is already prebuilt inside the image consecuently it is available to any container.
You could need to modify smacc2 code or examples and rebuilt it. 

If that is the case you can do the following:

```
cd ~/
```
```
source /opt/ros/humble/setup.bash
```
```
colcon build
```
To run your current demo, follow these steps after executing `colcon build`:

```
source install/setup.bash
```
```
ros2 launch sm_dancebot_ue sm_dancebot_ue_launch.py
```

Enjoy experimenting with Unreal Engine in your Docker environment!

