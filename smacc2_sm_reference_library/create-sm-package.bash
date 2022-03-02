#!/bin/bash
#
# Copyright 2021 Stogl Robotics Consulting
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Author: Denis Stogl
#

usage="create-sm-package.bash sm_name (lowercase letters with '_' split words)"

script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"

sm_name=$1
if [ -z "$1" ]; then
  echo -e "\e[0;31mYou should provide state machine name as argument!\e[0m"
  echo -e "\e[1;33mUsage: '$usage'\e[0m"
  exit 0
fi

echo ""
echo -e "\e[0;34mATTENTION: Setting up state machine package '$sm_name' in folder '`pwd`'. Is this source folder of your workspace?\e[0m"
echo ""
read -p "If correct press <ENTER>, otherwise <CTRL>+C and start the script again from your workspace source folder and/or with correct state machine name."

# Reset terminal colors
tput init


# Copy template package
cp -r $script_own_dir/_smacc2_sm_template $sm_name

# Define files where names have to be adjusted
SM_YAML="config/sm_name.yaml"
SM_CONFIG_YAML="config/sm_name_config.yaml"
SM_HPP="include/sm_name/sm_name.hpp"
SM_ST1_HPP="include/sm_name/states/st_state_1.hpp"
SM_ST2_HPP="include/sm_name/states/st_state_2.hpp"
SM_OR_TIMER_HPP="include/sm_name/orthogonals/or_timer.hpp"
SM_LAUNCH="launch/sm_name.launch"
SM_CPP="src/sm_name/sm_name_node.cpp"
SM_CMAKELISTS="CMakeLists.txt"
SM_PACKAGE="package.xml"
SM_README="README.md"

SM_FOLDERS=(
  "include/sm_name/orthogonals"
  "include/sm_name/states"
  "include/sm_name"
  "src/sm_name"
)

FILES_TO_SED=(
  $SM_YAML
  $SM_CONFIG_YAML
  $SM_HPP
  $SM_ST1_HPP
  $SM_ST2_HPP
  $SM_OR_TIMER_HPP
  $SM_LAUNCH
  $SM_CPP
  $SM_CMAKELISTS
  $SM_PACKAGE
  $SM_README
)

FILES_TO_RENAME=(
  $SM_YAML
  $SM_CONFIG_YAML
  $SM_HPP
  $SM_ST1_HPP
  $SM_ST2_HPP
  $SM_OR_TIMER_HPP
  $SM_LAUNCH
  $SM_CPP
)

# Rename package.xml because if it has proper name immediately then ROS-tools are making problems
mv "$sm_name/$SM_PACKAGE.template" "$sm_name/$SM_PACKAGE"

# sed all needed files
cd $sm_name
for SED_FILE in "${FILES_TO_SED[@]}"; do
  sed -i "s/\\\$sm_name\\\$/${sm_name}/g" $SED_FILE
  SmName=`echo $sm_name | sed 's/_/ /g'` # replace "_" with " "
  SmName=`echo $SmName | sed 's/^./\U&/g' | sed 's/ ./\U&/g'` # Capitalize first and all letters following " "
  SmName=`echo $SmName | sed 's/ //g'` # remove all " "
  sed -i "s/\\\$SmName\\\$/${SmName}/g" $SED_FILE
  sed -i "s/\\\$SM_NAME\\\$/${sm_name^^}/g" $SED_FILE
done
cd -

# Rename files
cd $sm_name

## Create missing folders
echo "Creating missing folders"
for FOLDER in "${SM_FOLDERS[@]}"; do
  mkdir -p `echo $FOLDER | sed "s/sm_name/${sm_name}/g"`
done

echo "Renaming files"
for FILE in "${FILES_TO_RENAME[@]}"; do
  mv $FILE `echo $FILE | sed "s/sm_name/${sm_name}/g"`
done

# Delete old folders
echo "Deleting old folders"
for FOLDER in "${SM_FOLDERS[@]}"; do
  rm -r $FOLDER
done
rm "COLCON_IGNORE"

# Remove currently not used files
rm "launch/NOT_USED_sm_name_with_arguments.launch"

cd -

echo -e "\e[0;32mState machine package is generated\e[0m"
echo "Go to the root folder of your workspace and build package using 'colcon build'."
