# SMACC2 and UNREAL ENGINE EDITOR
The README.md file provides a comprehensive guide for setting up and utilizing the Unreal Engine Docker environment. The document covers the necessary prerequisites, such as installing Docker and NVIDIA-Docker2, and provides step-by-step instructions for downloading and loading the prebuilt Docker image containing Unreal Engine. Additionally, it explains how to run the Unreal Editor within the container, create new containers for debugging purposes, and connect the container to a VPN if required. The README.md also includes important notes and optional instructions for rebuilding and running SMACC state machines inside the container. Whether you are new to Docker or an experienced user, this document will help you navigate the process of working with Unreal Engine in a Docker environment effectively.

## Important Notes about the Solution

Here are some important notes regarding the solution:

**Repos Versioning:** Not all versions of `rclUE`, `turtlebot3Editor`, and `RapyputaPlugins` are compatible with each other. Ensuring consistency among them can be challenging, especially when considering that the container maps/volumes some volumes with external contents. The container is already designed to have a correct combination of all of them, but this is hardcoded in the Dockerfile and can be improved.

**DDS Configuration:** There may be communication issues between the Docker container nodes and host nodes. As a workaround, we currently use `cyclonedds` on the host (until a uniform solution using `fastrtps` is found). To set this up, add the following line to the `.bashrc` file on the host:
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

***Automatic container nvidia driver update***
The host and the container must have the same nvidia-driver in order to run the ue editor and simulation. 
There is a mechanism implemented to automatically sync the driver. The current driver version is passed from the host to the container and then it is updated in the container if that is required. That is done in the nvidia-check.sh script.

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
To get the docker image, the best delivery mechanism we’ve got at the moment is to email brett@robosoft.ai with the subject line: DOCKER IMAGE REQUEST and I’ll send it over via a wetransfer. It’s kind of big (75gb) but it’s not that bad and we’re going to try to make it smaller going forward.

### Loading the Docker Image
To load the downloaded image into your Docker image database, use the following command:

```
sudo docker load -i ue_editor_rclue.tar
```

### (Alternative) Building the Docker Image Locally
If you want to modify the Docker image using a Dockerfile and rebuild it, follow these steps:

1. Create a link to the UE5.1 folder inside the `sm_dancebot_mine_ue/docker` folder:
   ```
   mount --bind <UE5.1DIR> UE5.1/UnrealEngine
   ```
 * Recall the <UE5.1DIR> must contain the following folders: `Engine`,  `FeaturePacks` and `Template`

2. Build the image locally by running the following command:
   ```
   ./build_docker_humble.sh
   ```

## Using the Docker Image
This section explains how to run and create a new container from the `ue_editor_rcl` Docker image.

### Running/Creating a New Container from the ue_editor_rcl Docker Image

To run the Unreal Editor inside the container, you need to use some auxiliary scripts located in the `sm_dancebot_mine_ue` example. Follow these steps:

1. Download the current SMACC2 repository.

2. Navigate to the `sm_dancebot_mine_ue/docker` folder.

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

There is an alternative way to create the container in daemon mode, where the lifetime of the container is not tied to the Unreal Editor window. This is useful, especially for developing new features for the container. To create the container in this mode,:

1. Execute the following command:
   ```
   ./run_docker_container_bash.sh
   ```

   This will create and start a new container as a daemon. The container will be available even after restarting. It is capable of opening the Unreal Engine editor with ROS2, but the editor will not open automatically.

2. Execute the editor proccess:
   ```
   ./join_editor.sh
   ```

The Unreal Engine editor will automatically open in "edition mode." You can launch the simulation with Turtlebot topics accessible from both the container and the host computer by clicking the "play" button.

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

### Connecting the Container to VPN

The prototype for connecting the container to a VPN has already been completed. For detailed instructions, please refer to the relevant documentation.

### Optionally Rebuilding and Running a SMACC State Machine inside the Container

Note that the `ue_editor_rcl` containers are run by mapping the current `smacc2` source folder, allowing for mixed development between the host (using VSCode) and the container (for compiling and running the state machines).

The SMACC2 source code is already prebuilt inside the image, making it available to any container. However, if you need to modify the `smacc2` code or examples and rebuild it, follow these steps:

1. Change to the home directory:
   ```
   cd ~/
   ```

2. Source the Humble ROS 2 installation:
   ```
   source /opt/ros/humble/setup.bash
   ```

3. Build the `smacc2` code:
   ```
   colcon build
   ```

4. To run your current demo, execute the following commands after building:
   ```
   source install/setup.bash
   ros2 launch sm_dancebot_mine_ue sm_dancebot_mine_ue_launch.py
   ```

Enjoy experimenting with Unreal Engine in your Docker environment!



# Brett's runtime notes

You'll need to open three terminals for this demo.
   One for the container where you'll run Unreal Engine
   One to run the state machine (on the host)
   One for the RTA (on the host)

### Terminal 1 - For UE5 Simulation in the container

1. Download the current SMACC2 repository.
   ```
   cd ~/workspace/humble_ws/src
   git clone https://github.com/robosoft-ai/SMACC2.git

   ```
2. Build the workspace

   ```
   cd ~/workspace/humble_ws/
   source /opt/ros/humble/setup.bash
   colcon build
   
   ```
   Once everything is done building...
3. Navigate to the `sm_dancebot_mine_ue/docker` folder.
   
   ```
   cd ~/workspace/humble_ws/src/SMACC2/smacc2_sm_reference_library/sm_dancebot_mine_ue/docker
   ```
4. Execute the following command:
   ```
   ./run_docker_container_editor.sh
   ```
   This will create and start a new container as a daemon. The container will be available even after restarting. It is capable of opening the Unreal Engine editor with ROS2, but the editor will not open automatically.
   The Unreal Engine editor will automatically open in "edition mode." You can launch the simulation with Turtlebot topics accessible from both the container and the host computer by clicking the "play" button.

### Terminal 2 - for the state machine on the host

1. Add the following line to the `.bashrc` file on the host:
   ```
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
2. Source the workspace you just built
   ```
   source ~/workspace/humble_ws/install/setup.bash
   ```
3. Launch the state machine
  ```
   ros2 launch sm_dancebot_mine_ue sm_dancebot_mine_ue_launch.py
  ```
  This will launch the state machine application, rviz and other required nodes.

### Terminal 3 - for the SMACC2_RTA on the host
1. Source the install
   ```
   source /opt/ros/humble/setup.bash
   ```
2. Launch the SMACC2_RTA
   ```
   ros2 run smacc2_rta smacc2_rta
   ```
3. Once the RTA is launched, in the upper left corner select State Machine/Available State Machines/SmDanceBotUE

And you should be all set.
