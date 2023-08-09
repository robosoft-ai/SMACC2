#!/bin/bash

OUTPUT_PATH=${1}

if [ -z "$OUTPUT_PATH" ]; then
    echo "Usage: $0 <output_path>"
    exit 1
fi

DATE_SUFFIX=`date +%Y%m%d_%H%M%S`
docker save -o "$OUTPUT_PATH/unreal_editor_smacc_$DATE_SUFFIX.tar" ue_editor_rclue:humble
