#pragma once

#include <map>
#include <moveit_msgs/msg/collision_object.hpp>
#include <smacc2/component.hpp>
#include <smacc2/smacc.hpp>

namespace cl_move_group_interface
{
class GraspingComponent : public smacc2::ISmaccComponent
{
private:
  std::map<std::string, moveit_msgs::msg::CollisionObject> graspingObjects;

public:
  std::vector<std::string> fingerTipNames;

  std::optional<std::string> currentAttachedObjectName;

  bool getGraspingObject(std::string name, moveit_msgs::msg::CollisionObject & object);

  void createGraspableBox(
    std::string frameid, float x, float y, float z, float xl, float yl, float zl);
};

}  // namespace cl_move_group_interface
