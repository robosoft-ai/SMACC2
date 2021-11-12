#!/bin/sh
DIR="$(dirname "$(realpath "$0")")"
echo $DIR
cd $DIR/..
echo `pwd`

sudo docker build -t smacc2 -f docker/Dockerfile .
