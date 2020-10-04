#pragma once

#include <smacc/smacc.h>
#include <ros_timer_client/cl_ros_timer.h>

namespace cl_ros_timer
{
class CbTimerCountdownOnce : public smacc::SmaccClientBehavior
{
public:
    CbTimerCountdownOnce(unsigned long triggerTickCount);

    virtual void onEntry() override;
    virtual void onExit() override;

    template <typename TOrthogonal, typename TSourceObject>
    void onOrthogonalAllocation()
    {
        this->postCountDownEvent_ = [=]() {
            this->template postEvent<EvTimer<TSourceObject, TOrthogonal>>();
        };
    }

    template <typename T>
    boost::signals2::connection onTimerTick(void (T::*callback)(), T *object)
    {
        return this->getStateMachine()->createSignalConnection(onTimerTick_, callback, object);
    }

private:
    unsigned long tickCounter_;
    unsigned long tickTriggerCount_;

    ClRosTimer *timerClient_;
    std::function<void()> postCountDownEvent_;
    smacc::SmaccSignal<void()> onTimerTick_;
    void onClientTimerTickCallback();
};
} // namespace cl_ros_timer