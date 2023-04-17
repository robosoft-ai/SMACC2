/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <iostream>
#include <boost/signals2.hpp>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <boost/signals2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/callback_counter_semaphore.hpp>

namespace smacc2
{
    CallbackCounterSemaphore::CallbackCounterSemaphore(std::string name, int count) : count_(count), name_(name) {}

    bool CallbackCounterSemaphore::acquire() {
        std::unique_lock<std::mutex> lock(mutex_);
        RCLCPP_DEBUG(rclcpp::get_logger(name_),"[CallbackCounterSemaphore] acquire callback %s %ld",name_.c_str(), (long)this);

        if(finalized)
        {
            RCLCPP_DEBUG(rclcpp::get_logger(name_),"[CallbackCounterSemaphore] callback rejected %s %ld",name_.c_str(), (long)this);
            return false;
        }

        ++count_;
        cv_.notify_one();

        RCLCPP_DEBUG(rclcpp::get_logger(name_),"[CallbackCounterSemaphore] callback accepted %s %ld",name_.c_str(), (long)this);
        return true;
    }

    void CallbackCounterSemaphore::release() {
        std::unique_lock<std::mutex> lock(mutex_);
        --count_;
        cv_.notify_one();

        RCLCPP_DEBUG(rclcpp::get_logger(name_),"[CallbackCounterSemaphore] callback finished %s %ld",name_.c_str(), (long)this);
    }

    void CallbackCounterSemaphore::finalize() {
        std::unique_lock<std::mutex> lock(mutex_);

        while (count_ > 0) {
            cv_.wait(lock);
        }
        finalized = true;

        for(auto conn: connections_)
        {
            conn.disconnect();
        }

        connections_.clear();
        RCLCPP_DEBUG(rclcpp::get_logger(name_),"[CallbackCounterSemaphore] callbacks finalized %s %ld",name_.c_str(), (long)this);
    }

    void CallbackCounterSemaphore::addConnection(boost::signals2::connection conn)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        if(finalized)
        {
            RCLCPP_DEBUG(rclcpp::get_logger(name_),"[CallbackCounterSemaphore] ignoring adding callback, already finalized %s %ld",name_.c_str(), (long)this);
            return;
        }

        connections_.push_back(conn);
    }

}