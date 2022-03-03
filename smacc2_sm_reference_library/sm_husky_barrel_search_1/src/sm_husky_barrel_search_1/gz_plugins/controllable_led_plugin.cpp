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


#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/World.hh"
#include "LedPlugin.hh"
#include "gazebo/transport/Node.hh"

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

namespace smacc2
{
class ControllableLed : public LedPlugin
{
public:
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_;

  ControllableLed() : LedPlugin()
  {
    gzmsg << "Loading ControllableLed plugin\n";
    // Transport initialization
    // this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
    // this->gzNode->Init();

    // Subscribe to ContainPlugin output
    //std::string topic("/test_cmd");

    // const bool containSub = this->dataPtr->node.Subscribe(topic,  &ControllableLed::OnCommand2, this);
    // //containSub = this->gzNode->Subscribe(topic, &ControllableLed::OnCommand, this);

    // if (!containSub)
    // {
    //   gzerr << "Failed to subscribe to [" << topic << "]\n";
    // }
    // else
    // {
    //   gzmsg << "Subscribed to [" << topic << "]\n";
    // }

    // Subscribe to ContainPlugin output
    // std::string topic("contain_example/contain");
    // std::function<void(const ignition::msgs::Boolean &)> cb =
    //   [=](const ignition::msgs::Boolean & _msg)
    // {
    //   //TurnOnLightPlugin::OnContainPluginMsg(_msg);
    // };
    // const bool containSub = this->node.Subscribe(topic, cb);
    // if (!containSub)
    // {
    //   gzerr << "Failed to subscribe to [" << topic << "]\n";
    // }
  }

  void LightCmd(const std_msgs::msg::Int8::SharedPtr msg)
  {
    gzmsg << "Turning on light\n";

    if (msg->data == 1)
    {
      gzmsg << "Turning on light\n";
      this->TurnOnAll();
      // lightMsg.set_range(15.0);
    }
    else if (msg->data == 0)
    {
      gzmsg << "Turning off light\n";
      this->TurnOffAll();
      // lightMsg.set_range(0.0);
    }
  }

  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
  {
    // Subscribe to wrench messages
    // ros_node_ = gazebo_ros::Node::Get(_sdf);

    // subscription_ = ros_node_->create_subscription<std_msgs::msg::Int8>(
    //   "led_cmd", rclcpp::SystemDefaultsQoS(),
    //   std::bind(&ControllableLed::LightCmd, this, std::placeholders::_1));

    // // Make a publisher for the light topic
    // this->lightPub = this->gzNode->Advertise<msgs::Light>("~/light/modify");

    LedPlugin::Load(_parent, _sdf);
    //this->TurnOnAll();
  }

public:
  void OnCommand(ConstIntPtr & _msg)
  {

  }
};
}  // namespace gazebo

GZ_REGISTER_MODEL_PLUGIN(smacc2::ControllableLed)
