/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <chrono>
#include <optional>
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

namespace smacc
{
class ISmaccUpdatable
{
public:
    ISmaccUpdatable();
    ISmaccUpdatable(rclcpp::Duration duration);

    void executeUpdate(rclcpp::Node::SharedPtr node);
    void setUpdatePeriod(rclcpp::Duration duration);
    
protected:
    virtual void update() = 0;

private:
    std::optional<rclcpp::Duration> periodDuration_;
    rclcpp::Time lastUpdate_;
};
} // namespace smacc