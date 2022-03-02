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

#include <smacc2/client_bases/smacc_ros_launch_client.hpp>

namespace smacc2
{
namespace client_bases
{
ClRosLaunch::ClRosLaunch(std::string packageName, std::string launchFilename)
: packageName_(packageName), launchFileName_(launchFilename), cancellationToken_(false)
{
}

ClRosLaunch::~ClRosLaunch() {}

void ClRosLaunch::launch()
{
  this->result_ = executeRosLaunch(
    packageName_, launchFileName_, [this]() { return this->cancellationToken_.load(); });
}

void ClRosLaunch::stop() { cancellationToken_.store(true); }

std::future<std::string> ClRosLaunch::executeRosLaunch(
  std::string packageName, std::string launchFileName, std::function<bool()> cancelCondition)
{
  return std::async(std::launch::async, [=]() {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("smacc2"), "[ClRosLaunch static] starting ros launch thread ");

    std::stringstream cmd;
    cmd << "ros2 launch " << packageName << " " << launchFileName;

    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.str().c_str(), "r"), pclose);
    if (!pipe)
    {
      throw std::runtime_error("popen() failed!");
    }

    std::stringstream ss;
    bool cancelled = false;
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr &&
           !(cancelled = cancelCondition()))
    {
      ss << buffer.data();
    }

    result = ss.str();
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("smacc2"), "[ClRosLaunch static]] RESULT = \n " << ss.str());
    return result;
  });
}

}  // namespace client_bases
}  // namespace smacc2
