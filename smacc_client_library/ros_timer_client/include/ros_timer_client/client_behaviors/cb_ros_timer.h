#pragma once

#include <ros_timer_client/cl_ros_timer.h>
#include <smacc/smacc.h>

namespace cl_ros_timer
{
class CbTimer : public smacc::SmaccClientBehavior
{
public:
  void onEntry() override;
  void onExit() override;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->postTimerEvent_ = [=]() {
      this->template postEvent<EvTimer<TSourceObject, TOrthogonal>>();
    };
  }

  void onClientTimerTickCallback();

private:
  ClRosTimer * timerClient_;
  std::function<void()> postTimerEvent_;
  boost::signals2::scoped_connection c_;
};
}  // namespace cl_ros_timer
