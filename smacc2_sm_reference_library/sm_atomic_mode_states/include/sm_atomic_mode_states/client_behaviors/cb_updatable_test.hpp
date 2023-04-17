#pragma once

#include <smacc2/smacc.hpp>

class CbUpdatableTest : public smacc2::ISmaccUpdatable, public smacc2::ISmaccClientBehavior
{
public:
    CbUpdatableTest()
    {
        this->counter_ = 0;
    }

    virtual void onEntry()
    {
        RCLCPP_INFO(getLogger(),"CbUpdatableTest::onEntry");
    }

    virtual void onExit()
    {
        RCLCPP_INFO(getLogger(),"CbUpdatableTest::onExit");
    }

    virtual void update() override
    {
        auto currentState = this->getCurrentState();
        this->counter_++;
        RCLCPP_INFO(getLogger(),"CbUpdatableTest::onUpdate %d in state %s", this->counter_, currentState->getClassName().c_str());
    }

    int counter_;
};
