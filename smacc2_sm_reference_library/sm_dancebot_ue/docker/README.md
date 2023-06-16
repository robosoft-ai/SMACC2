## Unreal Engine Docker Image

This repository provides a prebuilt Docker image containing Unreal Engine. Follow the instructions below to download, load, and run the image.

### Downloading the Prebuilt Image

Use the provided link to download the prebuilt Docker image for Unreal Engine:

```
wget https://www.dropbox.com/scl/fi/73dcb6whgy290xgpuduwj/unreal_editor_smacc-16th-Jun-2023.tar?dl=0&rlkey=60ueyxvtwtdmz2ujirkukjfy5 -O ue_editor_rclue.tar
```

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

To run your current demo, follow these steps after executing `colcon build`:

```
ros2 launch sm_dancebot_ue sm_dancebot_ue_launch.py
```

Enjoy experimenting with Unreal Engine in your Docker environment!