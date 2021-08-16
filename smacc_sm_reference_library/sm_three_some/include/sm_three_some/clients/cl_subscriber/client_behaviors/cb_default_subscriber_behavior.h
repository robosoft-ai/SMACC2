
#pragma once
#include <sm_three_some/clients/cl_subscriber/cl_subscriber.h>

namespace sm_three_some
{
namespace cl_subscriber
{
class CbDefaultSubscriberBehavior : public smacc::SmaccClientBehavior
{
public:
  typedef std_msgs::msg::UInt16 TMessageType;
  //-------------------------------------------------------------------------------
  void onEntry() {}
};
}  // namespace cl_subscriber
}  // namespace sm_three_some