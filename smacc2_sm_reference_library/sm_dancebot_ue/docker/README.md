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
```
source install/setup.bash
```



To run your current demo, follow these steps after executing `colcon build`:

```
ros2 launch sm_dancebot_ue sm_dancebot_ue_launch.py
```

Enjoy experimenting with Unreal Engine in your Docker environment!
