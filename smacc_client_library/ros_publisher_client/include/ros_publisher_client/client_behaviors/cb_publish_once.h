
#pragma once
#include <smacc/smacc_client_behavior.h>
#include <ros_publisher_client/cl_ros_publisher.h>

namespace cl_ros_publisher
{
template <typename RosMsgType>
class CbPublishOnce : public smacc::SmaccClientBehavior
{
private:
    std::function<void()> deferedPublishFn;
    ClRosPublisher *client_;

public:
    CbPublishOnce()
        : deferedPublishFn(nullptr)
    {
    }

    template <typename TMessage>
    CbPublishOnce(const TMessage &data)
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

        if (deferedPublishFn != nullptr)
            deferedPublishFn();
    }

    virtual void onExit() override
    {
    }
};
} // namespace cl_ros_publisher