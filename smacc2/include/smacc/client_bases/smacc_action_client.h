/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc_client.h>
#include <smacc/smacc_state_machine.h>

#include <rclcpp_action/client.hpp>

namespace smacc
{
namespace client_bases
{
// This class interface shows the basic set of methods that
// a SMACC "resource" or "plugin" Action Client has
class ISmaccActionClient : public ISmaccClient
{
public:
  ISmaccActionClient();

  // The destructor. This is called when the object is not
  // referenced anymore by its owner
  virtual ~ISmaccActionClient();

  // Gets the ros path of the action...
  inline std::string getNamespace() const
  {
    return name_;
  }

  virtual void cancelGoal() = 0;

  virtual std::shared_ptr<rclcpp_action::ClientBase> getClientBase() = 0;

protected:
  // The ros path where the action server is located
  std::string name_;
};
}  // namespace client_bases
}  // namespace smacc