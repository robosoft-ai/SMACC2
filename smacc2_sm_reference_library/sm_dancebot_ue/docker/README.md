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

### Running a New Container from the Image

To run the Docker image, first, download the current SMACC2 repository. Then, navigate to the Docker folder and execute the following command:

```
./run_docker_bash_humble.sh
```

This will start a new container, and the Unreal Engine editor will automatically open in "edition mode." By clicking the "play" button, you can launch the simulation with Turtlebot topics accessible from both the container and the host computer.

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
