
#pragma once
#include <sm_respira_1/clients/cl_subscriber/cl_subscriber.h>

namespace sm_respira_1
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
}  // namespace sm_respira_1
