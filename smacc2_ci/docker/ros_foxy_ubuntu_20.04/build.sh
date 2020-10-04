#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ../../../..
echo "Building DOCKER from directory `pwd`"
sudo docker build -f $DIR/Dockerfile -t smacc2_foxy_ubuntu_2004 .
cd $DIR
