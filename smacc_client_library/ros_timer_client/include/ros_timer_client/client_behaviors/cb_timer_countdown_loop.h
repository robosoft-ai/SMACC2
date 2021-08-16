#pragma once

#include <ros_timer_client/cl_ros_timer.h>
#include <smacc/smacc.h>

namespace cl_ros_timer
{
class CbTimerCountdownLoop : public smacc::SmaccClientBehavior
{
public:
  CbTimerCountdownLoop(unsigned long triggerTickCount);

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
  boost::signals2::connection onTimerTick(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onTimerTick_, callback, object);
  }

private:
  unsigned long tickTriggerCount_;
  unsigned long tickCounter_;

  ClRosTimer * timerClient_;
  std::function<void()> postCountDownEvent_;
  smacc::SmaccSignal<void()> onTimerTick_;
  void onClientTimerTickCallback();
};
}  // namespace cl_ros_timer
