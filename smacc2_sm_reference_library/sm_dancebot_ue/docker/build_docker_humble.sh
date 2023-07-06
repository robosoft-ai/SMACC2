#!/bin/sh
DIR="$(dirname "$(realpath "$0")")"
echo $DIR
$DIR/build_docker.sh humble humble jammy
