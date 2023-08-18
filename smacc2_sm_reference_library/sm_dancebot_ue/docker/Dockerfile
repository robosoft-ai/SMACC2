ARG ROS_DISTRO
ARG GIT_BRANCH
ARG UBUNTU_VERSION

FROM ros:$ROS_DISTRO-ros-base-$UBUNTU_VERSION

ARG ROS_DISTRO
ARG GIT_BRANCH
ARG LOCAL_FOLDER_SOURCE=1
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get clean

RUN apt-get update && apt-get install -q -y --no-install-recommends \
  dirmngr \
  gnupg2 \
  lsb-release \
  python3-colcon-ros \
  && apt-get clean \
  && apt upgrade -y --with-new-pkgs

RUN apt-get install -y nvidia-driver-525

RUN apt install -y xdg-user-dirs

ENV USER=ros2_ws
RUN bash -c "useradd -ms /bin/bash $USER"
RUN usermod -aG sudo $USER
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN chown -R $USER:$USER /home/$USER

USER $USER
WORKDIR /home/$USER

WORKDIR "/home/$USER/src"

RUN echo "copying current code version to docker image:"
#ADD . /home/$USER/src/SMACC2
#WORKDIR "/home/$USER"

# install dependencies and build
# RUN vcs import  src --skip-existing --input src/SMACC2/.github/SMACC2-not-released.$ROS_DISTRO.repos
# RUN ls src

# RUN apt update
# RUN rosdep install --from-paths src --ignore-src -r -y
# RUN apt-get update && apt-get install -q -y --no-install-recommends xterm

# RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.sh && colcon build --merge-install --parallel-workers 1"

COPY --chown=$USER:$USER smacc2_sm_reference_library/sm_dancebot_ue/docker/UE5.1/UnrealEngine /home/$USER/src/UE5.1/UnrealEngine

ENV UE5_DIR=/home/$USER/src/UE5.1/UnrealEngine
ENV PROJECT_BRANCH=main
ENV PROJECT_NAME=ue_project_1

RUN sudo apt-get install -y wget gpg
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | sudo gpg --dearmor > packages.microsoft.gpg
RUN sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
RUN sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list' -f packages.microsoft.gpg

RUN sudo apt install -y apt-transport-https
RUN sudo apt update
RUN sudo apt install -y code git-lfs nano # or code-insiders

# smacc2_devel
RUN git clone --recurse-submodules https://github.com/robosoft-ai/${PROJECT_NAME}.git -b $PROJECT_BRANCH

WORKDIR "/home/$USER/src/${PROJECT_NAME}"
# RUN git checkout abdf483ce14f477d8f255fe19f3527975b3a9dbb
# RUN cd Plugins/rclUE && git checkout bae993fa

#RUN rm -rf Plugins/RapyutaSimulationPlugins && git clone https://github.com/robosoft-ai/UE-Plugins.git Plugins/RapyutaSimulationPlugins
#RUN cd Plugins/RapyutaSimulationPlugins && git checkout bfac015

RUN git-lfs pull && git submodule foreach git-lfs pull
RUN ls && echo ${UE5_DIR}
COPY --chown=$USER:$USER smacc2_sm_reference_library/sm_dancebot_ue/docker/update_project_files.sh update_project_files.sh
ENV UE5_DIR=/home/$USER/src/UE5.1

# installing Dependencies
RUN wget http://security.ubuntu.com/ubuntu/pool/universe/t/tinyxml2/libtinyxml2-6a_7.0.0+dfsg-1build1_amd64.deb
RUN wget http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.0g-2ubuntu4_amd64.deb
RUN sudo dpkg -i libtinyxml2-6a_7.0.0+dfsg-1build1_amd64.deb
RUN sudo dpkg -i libssl1.1_1.1.0g-2ubuntu4_amd64.deb

ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/$USER/src/${PROJECT_NAME}/Plugins/rclUE/ThirdParty/ros/lib/
ENV UE5_DIR=/home/$USER/src/UE5.1/UnrealEngine

COPY --chown=$USER:$USER smacc2_sm_reference_library/sm_dancebot_ue/docker/nvidia-driver-check.sh /home/$USER/src/${PROJECT_NAME}/nvidia-driver-check.sh
WORKDIR /home/$USER/src/${PROJECT_NAME}
RUN echo "nvidia-driver-check.sh"
# RUN cat update_project_files.sh
# RUN ../UE5.1/UnrealEngine/Engine/Build/BatchFiles/Linux/GenerateProjectFiles.sh -CurrentPlatform ue_project_1.uproject
# #RUN bash

# RUN make ${PROJECT_NAME}Editor

#  # COPY smacc2 /home/$USER/src/SMACC2/smacc2
# RUN mkdir -p /home/$USER/src/SMACC2/
# WORKDIR "/home/$USER/src/SMACC2"
# COPY --chown=$USER:$USER smacc2_client_library /home/$USER/src/SMACC2/smacc2_client_library
# COPY --chown=$USER:$USER smacc2_event_generator_library /home/$USER/src/SMACC2/smacc2_event_generator_library
# COPY --chown=$USER:$USER smacc2_performance_tools /home/$USER/src/SMACC2/smacc2_performance_tools
# COPY --chown=$USER:$USER smacc2_state_reactor_library /home/$USER/src/SMACC2/smacc2_state_reactor_library
# COPY --chown=$USER:$USER smacc2 /home/$USER/src/SMACC2/smacc2
# COPY --chown=$USER:$USER smacc2_dev_tools /home/$USER/src/SMACC2/smacc2_dev_tools       
# COPY --chown=$USER:$USER smacc2_msgs /home/$USER/src/SMACC2/smacc2_msgs                     
# #smacc2_sm_reference_library  

# #RUN git clone
# RUN sudo apt-get update 
# RUN rosdep update
# RUN rosdep install --ignore-src --from-paths . -y -r
# RUN sudo apt-get update && sudo apt-get install -y ros-humble-nav2-bringup ros-humble-rviz2

# RUN sudo apt-get install -y openvpn
# #COPY --chown=$USER:$USER smacc2_sm_reference_library/sm_dancebot_ue/docker/run_editor_smacc.sh /home/$USER/src/${PROJECT_NAME}/run_editor_smacc.sh


# RUN touch /home/$USER/src/${PROJECT_NAME}/COLCON_IGNORE
# RUN touch /home/$USER/src/UE5.1/COLCON_IGNORE
# WORKDIR "/home/$USER/"
# RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# # BASIC VPN FRAMEWORK ---
RUN mkdir /home/$USER/ovpnconfig

WORKDIR "/home/$USER/src/${PROJECT_NAME}"

# #ENV LD_LIBRARY_PATH=/home/ros2_ws/src/ue_project_1/Plugins/rclUE/ThirdParty/ros/lib/:/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:
