/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace cl_move_base_z
{
    using namespace std::chrono_literals;

    // static
    std::shared_ptr<tf2_ros::Buffer> Pose::tfListener_;
    std::mutex Pose::listenerMutex_;

    Pose::Pose(std::string targetFrame, std::string referenceFrame)
        : poseFrameName_(targetFrame), referenceFrame_(referenceFrame), isInitialized(false)
    {
        this->pose_.header.frame_id = referenceFrame_;
        RCLCPP_INFO(getNode()->get_logger(),"[Pose] Creating Pose tracker component to track %s in the reference frame %s", targetFrame.c_str(), referenceFrame.c_str());

        {
            //singleton
            std::lock_guard<std::mutex> guard(listenerMutex_);
            if(tfListener_==nullptr)
                tfListener_ = std::make_shared<tf2_ros::Buffer>(getNode()->get_clock());
        }
    }

    void Pose::waitTransformUpdate(rclcpp::Rate r)
    {
        bool found = false;
        while (rclcpp::ok() && !found)
        {
            tf2::Stamped<tf2::Transform> transform;
            try
            {
                {
                    std::lock_guard<std::mutex> lock(listenerMutex_);
                    
                    auto transformstamped = tfListener_->lookupTransform(referenceFrame_, poseFrameName_,
                                                                    rclcpp::Time(0));
                    tf2::fromMsg(transformstamped, transform);
                }

                {
                    std::lock_guard<std::mutex> guard(m_mutex_);
                    tf2::toMsg(transform, this->pose_.pose);
                    this->pose_.header.stamp = tf2_ros::toRclcpp(transform.stamp_);
                    found = true;
                    this->isInitialized = true;
                }
            }
            catch (tf2::TransformException ex)
            {
                RCLCPP_ERROR_STREAM_THROTTLE(getNode()->get_logger(),*(getNode()->get_clock()), 1, "[Component pose] (" << poseFrameName_ << "/[" << referenceFrame_ << "] ) is failing on pose update : " << ex.what());
            }

            r.sleep();
            rclcpp::spin_some(getNode());
        }
    }

    void Pose::update()
    {
        tf2::Stamped<tf2::Transform> transform;
        try
        {
            {
                std::lock_guard<std::mutex> lock(listenerMutex_);
                auto transformstamped = tfListener_->lookupTransform(referenceFrame_, poseFrameName_,
                                                  rclcpp::Time(0));
                tf2::fromMsg(transformstamped,transform) ;
            }

            {
                std::lock_guard<std::mutex> guard(m_mutex_);
                tf2::toMsg(transform, this->pose_.pose);
                this->pose_.header.stamp = tf2_ros::toRclcpp(transform.stamp_);
                this->isInitialized = true;
            }
        }
        catch (tf2::TransformException ex)
        {
            RCLCPP_ERROR_STREAM_THROTTLE(getNode()->get_logger(), *(getNode()->get_clock()), 1, "[Component pose] (" << poseFrameName_ << "/[" << referenceFrame_ << "] ) is failing on pose update : " << ex.what());
        }
    }
} // namespace cl_move_base_z
