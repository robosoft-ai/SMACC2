#pragma once

#include <smacc/smacc.h>
#include <ros_timer_client/cl_ros_timer.h>

namespace cl_ros_timer
{
class CbTimer : public smacc::SmaccClientBehavior
{
public:
  virtual void onEntry() override;
  virtual void onExit() override;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->postTimerEvent_ = [=]() {
      this->template postEvent<EvTimer<TSourceObject, TOrthogonal>>();
    };
  }

  void onClientTimerTickCallback();

private:
  ClRosTimer *timerClient_;
  std::function<void()> postTimerEvent_;
  boost::signals2::scoped_connection c_;
};
} // namespace cl_ros_timer
