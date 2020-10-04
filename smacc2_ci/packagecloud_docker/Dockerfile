
ARG ROS_DOCKER_BASE
FROM $ROS_DOCKER_BASE

ARG UBUNTU_VERSION
ARG ROS_VERSION_NAME
ARG GITHUB_USER
ARG GITHUB_TOKEN
ARG PACKAGE_CLOUD_USER
ARG PACKAGE_CLOUD_TOKEN

ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

ENV GITHUB_USER=$GITHUB_USER
ENV GITHUB_TOKEN=$GITHUB_TOKEN
ENV PACKAGE_CLOUD_USER=$PACKAGE_CLOUD_USER
ENV PACKAGE_CLOUD_TOKEN=$PACKAGE_CLOUD_TOKEN
ENV UBUNTU_VERSION=$UBUNTU_VERSION
ENV ROS_VERSION_NAME=$ROS_VERSION_NAME

RUN echo "rosversion: $ROS_VERSION_NAME"
RUN echo "ubuntu version: $UBUNTU_VERSION"
RUN echo "github user: $GITHUB_USER"
RUN echo "github token: $GITHUB_TOKEN"
RUN echo "packagecloud user: $PACKAGE_CLOUD_USER"
RUN echo "packagecloud token: $PACKAGE_CLOUD_TOKEN"

ENV HOME /root
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \
    ros-$ROS_VERSION_NAME-robot \
&& rm -rf /var/lib/apt/lists/*

# SYSTEM DEPENDENCIES
#----------------------------------------------------------
RUN export DEBIAN_FRONTEND="noninteractive"; apt-get update && apt-get install -y apt-utils && apt-get install -y git python-catkin-tools fakeroot

RUN apt-get update
RUN apt-get install -y python-argcomplete python-bloom dh-make

# INSTALL PACKAGE CLOUD SOFTWARE
# --------------------------------------------------------------
RUN apt-get -y install ruby-dev nano
RUN gem install rake
RUN gem install package_cloud

# DOWNLOAD MAIN REPOSITORY
#----------------------------------------------------------
RUN echo "regen "
RUN echo "downloading smacc repo"
RUN git clone https://github.com/reelrbtx/SMACC.git /root/src/SMACC
RUN echo "downloading smacc_viewer repo with github user: ${GITHUB_USER}"

RUN git clone https://github.com/reelrbtx/reelrbtx_msgs.git /root/src/reelrbtx_msgs
WORKDIR /root



#RUN echo "regen"
# BUILD SMACC
# -----------------------------------------------------------------
RUN bash -c "source /opt/ros/$ROS_VERSION_NAME/setup.bash; cd /root; rosdep install --from-paths src --ignore-src -r -y; "
RUN bash -c "source /opt/ros/$ROS_VERSION_NAME/setup.bash; cd /root; catkin build"

# BUILD DEBIAN FILES
# ------------------------------------------------------------------------

RUN echo "yaml file:/root/src/SMACC/smacc_ci/rosdep_${ROS_VERSION_NAME}.yaml" > /etc/ros/rosdep/sources.list.d/50-my-packages.list 
RUN rosdep update

RUN echo "regen"
ADD generate_debs.py /root/src/SMACC/smacc_ci/packagecloud_docker/generate_debs.py

WORKDIR /root
RUN echo "... ROS_VERSION: $ROS_VERSION_NAME"
RUN echo "... UBUNTU_VERSION, $UBUNTU_VERSION"
RUN bash -c 'source devel/setup.bash; python  src/SMACC/smacc_ci/packagecloud_docker/generate_debs.py -ros_version="$ROS_VERSION_NAME" -smacc_src_folder="src/SMACC" -smacc_viewer_src_folder="src/SMACC_Viewer" -token="$PACKAGE_CLOUD_TOKEN" -repo_owner="$PACKAGE_CLOUD_USER" -ubuntu_version="$UBUNTU_VERSION"'
