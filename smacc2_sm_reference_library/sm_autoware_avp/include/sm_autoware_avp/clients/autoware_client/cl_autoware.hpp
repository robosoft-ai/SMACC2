#pragma once

#include <smacc2/client_base_components/cp_topic_publisher.hpp>
#include <smacc2/client_base_components/cp_topic_subscriber.hpp>

//#include <autoware_auto_msgs//>
#include <smacc2/smacc_client.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace sm_autoware_avp
{
    namespace clients
    {
        class ClAutoware : public smacc2::ISmaccClient
        {
            void onInitialize() override
            {
                //auto cppub = this->createComponent<smacc2::components::CpTopicPublisher<geometry_msgs::msg::PoseStamped>>();


                // /localization/initialpose - geometry_msgs/msg/PoseWithCovarianceStamped
                // /planning/goal_pose - geometry_msgs/msg/PoseStamped
            }
        };
    }
}