#include <move_group_interface_client/components/cp_grasping_objects.h>

namespace cl_move_group_interface
{
   bool GraspingComponent::getGraspingObject(std::string name, moveit_msgs::CollisionObject &object)
   {
      if (this->graspingObjects.count(name))
      {
         object = this->graspingObjects[name];
         return true;
      }
      else
      {
         return false;
      }
   }

   void GraspingComponent::createGraspableBox(std::string frameid, float x, float y, float z, float xl, float yl, float zl)
   {
      moveit_msgs::CollisionObject collision;
      auto boxname = frameid;
      ;
      collision.id = boxname;
      collision.header.frame_id = frameid;

      collision.primitives.resize(1);
      collision.primitives[0].type = collision.primitives[0].BOX;
      collision.primitives[0].dimensions.resize(3);

      collision.primitives[0].dimensions[0] = xl;
      collision.primitives[0].dimensions[1] = yl;
      collision.primitives[0].dimensions[2] = zl;

      collision.primitive_poses.resize(1);
      collision.primitive_poses[0].position.x = x;
      collision.primitive_poses[0].position.y = y;
      collision.primitive_poses[0].position.z = z;
      collision.primitive_poses[0].orientation.w = 1.0;

      graspingObjects[boxname] = collision;
   }
} // namespace cl_move_group_interface
