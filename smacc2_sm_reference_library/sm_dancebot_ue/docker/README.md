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

## Pre-Requisites
Before proceeding with the instructions, ensure that you have the necessary prerequisites installed on your system.

### Docker Installation
To install Docker, follow these steps:

1. Update the package list:
   ```
   sudo apt-get update
   ```

2. Install the required dependencies:
   ```
   sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
   ```

3. Add Docker's official GPG key:
   ```
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
   ```

4. Add the Docker repository:
   ```
   sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu bionic stable"
   ```

5. Update the package list again:
   ```
   sudo apt-get update
   ```

6. Install Docker:
   ```
   sudo apt-get install docker-ce
   ```

### NVIDIA-Docker2 Installation
Ensure that you have NVIDIA-Docker2 installed by executing the following commands:

1. Add the NVIDIA-Docker GPG key:
   ```
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   ```

2. Determine the distribution:
   ```
   distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
   ```

3. Add the NVIDIA-Docker repository:
   ```
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   ```

4. Update the package list:
   ```
   sudo apt-get update
   ```

5. Install NVIDIA-Docker2:
   ```
   sudo apt-get install nvidia-docker2
   ```

6. Restart the Docker service:
   ```
   sudo systemctl restart docker.service
   ```

### Adding User to Docker Group
To allow a user to run Docker, add the user to the Docker group. Replace `<userid>` with the actual user ID:

```
sudo usermod -a -G docker <userid>
```

### Testing the Installation
To test your Docker and NVIDIA-Docker2 installations, run the following command:

```
sudo nvidia-docker run --rm nvidia/cuda:10.2-base-ubuntu18.04 nvidia-smi
```

## Unreal Engine Docker Image
This section provides instructions for downloading, loading, and running the prebuilt Docker image containing Unreal Engine.

### Downloading the Prebuilt Image
Download the prebuilt Docker image for Unreal Engine from the provided source.

### Loading the Docker Image
To load the downloaded image into your Docker image database, use the following command:

```
sudo docker load -i ue_editor_rclue.tar
```

### (Alternative) Building the Docker Image Locally
If you want to modify the Docker image using a Dockerfile and rebuild it, follow these steps:

1. Create a link to the UE5.1 folder inside the `sm_dancebot_ue/docker` folder:
   ```
   mount --bind <UE5.1DIR> UE5.1/UnrealEngine
   ```

2. Build the image locally by running the following command:
   ```
   ./build_docker_humble.sh
   ```

## Using the Docker Image
This section explains how to run and create a new container from the `ue_editor_rcl` Docker image.

### Running/Creating a New Container from the ue_editor_rcl Docker Image

To run the Unreal Editor inside the container, you need to use some auxiliary scripts located in the `sm_dancebot_ue` example. Follow these steps:

1. Download the current SMACC2 repository.

2. Navigate to the `sm_dancebot_ue/docker` folder.

3. Execute the following command:
   ```
   ./run_docker_container_editor.sh
   ```

   This will run and create a new container using the `ue_editor_rcl` Docker image.

4. The Unreal Engine editor will automatically open in "edition mode." You can launch the simulation with Turtlebot topics accessible from both the container and the host computer by clicking the "play" button.

   Note: When you close the editor, the container will also be finished, but it will remain installed. You can reopen the editor using the command:
   ```
   ./start_container.sh
   ```

### (Alternative) Running/Creating a New Container for Container Debugging

There is an alternative way to create the container in daemon mode, where the lifetime of the container is not tied to the Unreal Editor window. This is useful, especially for developing new features for the container. To create the container in this mode, follow these steps:

1. Execute the following command:
   ```
   ./run_docker_container_bash.sh
   ```

   This will create and start a new container as a daemon. The container will be available even after restarting. It is capable of opening the Unreal Engine editor with ROS2, but the editor will not open until you run the following command:
   ```
   ./join_editor.sh
   ```

2. The Unreal Engine editor will automatically open in "edition mode." You can launch the simulation with Turtlebot topics accessible from both the container and the host computer by clicking the "play" button.

### Stopping and Removing a Running Container

If you need to reset everything and remove the existing container, follow these steps:

1. Execute the following command:
   ```
   ./stop_container.sh
   ```

   This will stop the running container.

2. Execute the following command:
   ```
   ./remove_container.sh
   ```

   This will remove the existing container.

### Joining the Container via Bash

To enter the Docker container and debug or test things from the command line, use the following command:

```
./join_bash.sh
```

## Connecting the Container to VPN

The prototype for connecting the container to a VPN has already been completed. For detailed instructions, please refer to the relevant documentation.

### Important Notes about the Solution

Here are some important notes regarding the solution:

**Repos Versioning:** Not all versions of `rclUE`, `turtlebot3Editor`, and `RapyputaPlugins` are compatible with each other. Ensuring consistency among them can be challenging, especially when considering that the container maps/volumes some volumes with external contents. The container is already designed to have a correct combination of all of them, but this is hardcoded in the Dockerfile and can be improved.

**DDS Configuration:** There may be communication issues between the Docker container nodes and host nodes. As a workaround, we currently use `cyclonedds` on the host (until a uniform solution using `fastrtps` is found). To set this up, add the following line to the `.bashrc` file on the host:
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Optionally Rebuilding and Running a SMACC State Machine inside the Container

Note that the `ue_editor_rcl` containers are run by mapping the current `smacc2` source folder, enabling mixed development between the host (using