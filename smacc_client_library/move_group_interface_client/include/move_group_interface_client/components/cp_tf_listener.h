/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/component.h>
#include <smacc/smacc_updatable.h>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <mutex>
#include <thread>

namespace cl_move_base_z
{
    struct TfPoseTrack
    {
        std::mutex mutex_;
        geometry_msgs::msgs::PoseStamped pose_;
        std::string targetPoseFrame_;
        std::string referenceBaseFrame_;
        bool once = true;
        bool isInitialized = false;
    };

    class CpTFListener : public smacc::ISmaccComponent, public smacc::ISmaccUpdatable
    {
    public:
        CpTFListener();

        virtual void update() override
        {
            for(auto& poseTrack: poseTracks_)
            {
                tf::StampedTransform transform;
                try
                {
                    {
                        std::lock_guard<std::mutex> lock(listenerMutex_);
                        tfListener_->lookupTransform(poseTrack->referenceBaseFrame_,poseTrack->targetPoseFrame_,
                                                    ros::Time(0), transform);
                    }

                    {
                        std::lock_guard<std::mutex> guard(m_mutex_);
                        tf::poseTFToMsg(transform, poseTrack->pose_.pose);
                        poseTrack->pose_.header.stamp = transform.stamp_;
                        poseTrack->pose_.header.frame_id = poseTrack->referenceBaseFrame_;
                        poseTrack->isInitialized = true;
                    }
                }
                catch (tf::TransformException ex)
                {
                    RCLCPP_ERROR_STREAM_THROTTLE(1, "[Component pose] (nh_->get_logger(), " << poseFrameName_ << "/[" << referenceFrame_ << "] ) is failing on pose update : " << ex.what());
                }
            }
        }

        void getLastTransform(std::string &targetPoseFrameName, std::string &referenceBaseFrame, geometry_msgs::msgs::Pose &out)
        {
        }

        std::future<geometry_msgs::msgs::Pose> waitForNextTransform(std::string &targetName, std::string &referenceBaseFrame)
        {
            tracks_
        }

    private:
        static std::shared_ptr<tf::TransformListener> tfListener_;
        static std::mutex listenerMutex_;

        std::mutex m_mutex_;
        std::list<std::shared_ptr<TfPoseTrack>> poseTracks_;
    };
} // namespace cl_move_base_z
