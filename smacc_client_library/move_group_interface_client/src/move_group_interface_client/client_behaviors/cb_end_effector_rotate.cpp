#include <move_group_interface_client/client_behaviors/cb_end_effector_rotate.h>

namespace cl_move_group_interface
{
    CbEndEffectorRotate::CbEndEffectorRotate(double deltaRadians, std::string tipLink)
        : CbCircularPivotMotion(tipLink)
    {
        deltaRadians_ = deltaRadians;
    }

    CbEndEffectorRotate::~CbEndEffectorRotate()
    {

    }

    void CbEndEffectorRotate::onEntry()
    {
        // autocompute pivot pose

        tf::TransformListener tfListener;
        // tf::StampedTransform globalBaseLink;
        tf::StampedTransform endEffectorInPivotFrame;

        int attempts = 3;

        while (attempts > 0)
        {
            try
            {
                this->requiresClient(movegroupClient_);
                if (!tipLink_ || *tipLink_ == "")
                {
                    tipLink_ = this->movegroupClient_->moveGroupClientInterface.getEndEffectorLink();
                }

                auto pivotFrame = this->movegroupClient_->moveGroupClientInterface.getPlanningFrame();

                tf::StampedTransform endEffectorInPivotFrame;

                tfListener.waitForTransform(pivotFrame, *tipLink_, ros::Time(0), ros::Duration(10));
                tfListener.lookupTransform(pivotFrame, *tipLink_, ros::Time(0), endEffectorInPivotFrame);

                tf::poseTFToMsg(endEffectorInPivotFrame, this->planePivotPose_.pose);
                this->planePivotPose_.header.frame_id = endEffectorInPivotFrame.frame_id_;
                this->planePivotPose_.header.stamp = endEffectorInPivotFrame.stamp_;
                break;
            }
            catch (const std::exception &e)
            {
                ROS_ERROR_STREAM(e.what() << ". Attempt countdown: " << attempts);
                ros::Duration(0.5);
                attempts--;
            }
        }

        CbCircularPivotMotion::onEntry();
    }
} // namespace cl_move_group_interface