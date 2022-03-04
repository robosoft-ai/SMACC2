#!/bin/sh
DIR="$(dirname "$(realpath "$0")")"
echo $DIR
$DIR/build_docker.sh rolling master focal
