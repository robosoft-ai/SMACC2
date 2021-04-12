FROM osrf/ros2:nightly

ENV HOME /root
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get upgrade && apt-get update
RUN apt-cache search  libboost
RUN apt-get -y install libboost-dev libboost-thread-dev

RUN mkdir -p /root/smacc2_ws/src
ADD SMACC2 /root/smacc2_ws/src/
WORKDIR /root/smacc2_ws
RUN find .
#ADD "https://www.random.org/cgi-bin/randbyte?nbytes=10&format=h" skipcache
#RUN bash -c "source /opt/ros/foxy/setup.bash && colcon build"
RUN bash -c "colcon build"

#ENTRYPOINT "/usr/bin/find /root/"
# SYSTEM DEPENDENCIES
#----------------------------------------------------------
#RUN echo "regen"
#RUN export DEBIAN_FRONTEND="noninteractive"; apt-get update && apt-get install -y apt-utils && apt-get install -y curl

#RUN curl -s https://packagecloud.io/install/repositories/reelrbtx/SMACC/script.deb.sh | sudo bash
#RUN apt-get install -y ros-melodic-smacc ros-melodic-sm-dance-bot-strikes-back ros-melodic-sm-atomic

#RUN curl -s https://b0e12e65a4f16bfc4594206c69dce2a49a5eabd04efb7540:@packagecloud.io/install/repositories/reelrbtx/SMACC_viewer/script.deb.sh  | bash
#RUN apt-get -y install ros-melodic-smacc-viewer

WORKDIR /root
