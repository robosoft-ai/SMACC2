#pragma once

#include <smacc/smacc.h>
#include <boost/signals2.hpp>
#include <optional>
#include <chrono>

namespace cl_ros_timer
{
template <typename TSource, typename TOrthogonal>
struct EvTimer : sc::event<EvTimer<TSource, TOrthogonal>>
{
    /*
    ClRosTimer *sender;
    ros::TimerEvent timedata;

    EvTimer(ClRosTimer *sender, const ros::TimerEvent &timedata)
    {
        this->sender = sender;
        this->timedata = timedata;
    }
    */
};

class ClRosTimer : public smacc::ISmaccClient
{
public:
    ClRosTimer(std::chrono::seconds duration, bool oneshot = false);

    virtual ~ClRosTimer();

    virtual void initialize();

    template <typename T>
    boost::signals2::connection onTimerTick(void (T::*callback)(), T *object)
    {
        return this->getStateMachine()->createSignalConnection(onTimerTick_, callback, object);
    }

    template <typename TOrthogonal, typename TSourceObject>
    void onOrthogonalAllocation()
    {
        this->postTimerEvent_ = [=]() {
            this->postEvent<EvTimer<TSourceObject, TOrthogonal>>();
        };
    }

protected:

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::seconds duration_;
    bool oneshot_;

    void timerCallback();
    std::function<void()> postTimerEvent_;
    smacc::SmaccSignal<void()> onTimerTick_;
};
} // namespace cl_ros_timer
