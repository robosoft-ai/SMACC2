
#pragma once
#include <smacc/smacc_client_behavior.h>
#include <ros_publisher_client/cl_ros_publisher.h>

namespace cl_ros_publisher
{

class CbDefaultPublishLoop : public smacc::SmaccClientBehavior,
                             public smacc::ISmaccUpdatable
{
private:
    std::function<void()> deferedPublishFn;
    ClRosPublisher *client_;

public:
    CbDefaultPublishLoop()
        : deferedPublishFn(nullptr)
    {
    }

    template <typename TMessage>
    CbDefaultPublishLoop(const TMessage &data)
    {
        this->setMessage(data);
    }

    template <typename TMessage>
    void setMessage(const TMessage &data)
    {
        deferedPublishFn = [=]() {
            client_->publish(data);
        };
    }

    virtual void onEntry() override
    {
        this->requiresClient(client_);
    }

    virtual void update()
    {
        if (deferedPublishFn != nullptr)
            deferedPublishFn();
    }

    virtual void onExit() override
    {
    }
};
} // namespace cl_ros_publisher