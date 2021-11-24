// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <move_group_interface_client/client_behaviors/cb_move_known_state.hpp>

namespace cl_move_group_interface
{
CbMoveKnownState::CbMoveKnownState(std::string pkg, std::string config_path)
: pkg_(pkg), config_path_(config_path)
{
}

CbMoveKnownState::~CbMoveKnownState() {}

void CbMoveKnownState::onEntry()
{
  jointValueTarget_ = loadJointStatesFromFile(pkg_, config_path_);
  CbMoveJoints::onEntry();
}

#define HAVE_NEW_YAMLCPP
std::map<std::string, double> CbMoveKnownState::loadJointStatesFromFile(
  std::string pkg, std::string filepath)
{
  //auto pkgpath = ros::package::getPath(pkg);
  std::string pkgpath = ament_index_cpp::get_package_share_directory(pkg);

  std::map<std::string, double> jointStates;

  if (pkgpath == "")
  {
    RCLCPP_ERROR_STREAM(
      getLogger(), "[" << getName() << "] package not found for the known poses file: " << pkg
                       << std::endl
                       << " [IGNORING BEHAVIOR]");
    return jointStates;
  }

  filepath = pkgpath + "/" + filepath;

  RCLCPP_INFO_STREAM(
    getLogger(), "[" << getName() << "] Opening file with joint known state: " << filepath);

  if (std::filesystem::exists(filepath))
  {
    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] known state file exists: " << filepath);
  }
  else
  {
    RCLCPP_ERROR_STREAM(
      getLogger(), "[" << getName() << "] known state file does not exists: " << filepath);
  }

  std::ifstream ifs(filepath.c_str(), std::ifstream::in);
  if (ifs.good() == false)
  {
    RCLCPP_ERROR_STREAM(
      getLogger(),
      "[" << getName() << "] Error opening file with joint known states: " << filepath);
    throw std::string("joint state files not found");
  }

  try
  {
#ifdef HAVE_NEW_YAMLCPP
    YAML::Node node = YAML::Load(ifs);
#else
    YAML::Parser parser(ifs);
    parser.GetNextDocument(node);
#endif

#ifdef HAVE_NEW_YAMLCPP
    const YAML::Node & wp_node_tmp = node["joint_states"];
    const YAML::Node * wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
#else
    const YAML::Node * wp_node = node.FindValue("waypoints");
#endif

    if (wp_node != NULL)
    {
      try
      {
        for (YAML::const_iterator it = wp_node->begin(); it != wp_node->end(); ++it)
        {
          std::string key = it->first.as<std::string>();
          double value = it->second.as<double>();
          RCLCPP_DEBUG_STREAM(getLogger(), " joint - " << key << ": " << value);
          jointStates[key] = value;
        }

        return jointStates;
      }
      catch (std::exception & ex)
      {
        RCLCPP_ERROR(getLogger(), "trying to convert to map, failed, errormsg: %s", ex.what());
      }

      RCLCPP_INFO_STREAM(getLogger(), "Parsed " << jointStates.size() << " joint entries.");
    }
    else
    {
      RCLCPP_WARN_STREAM(getLogger(), "Couldn't find any jointStates in the provided yaml file.");
    }
  }
  catch (const YAML::ParserException & ex)
  {
    RCLCPP_ERROR_STREAM(
      getLogger(), "Error loading the Waypoints YAML file. Incorrect syntax: " << ex.what());
  }
  return jointStates;
}
}  // namespace cl_move_group_interface
