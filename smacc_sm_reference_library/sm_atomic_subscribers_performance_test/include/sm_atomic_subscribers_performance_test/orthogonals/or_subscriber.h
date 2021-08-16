#include <smacc/client_bases/smacc_subscriber_client.h>
#include <smacc/smacc.h>

#include <std_msgs/msg/int16.hpp>

namespace sm_atomic_subscribers_performance_test
{
using namespace smacc::client_bases;

class OrSubscriber : public smacc::Orthogonal<OrSubscriber>
{
public:
  virtual void onInitialize() override
  {
    this->createClient<SmaccSubscriberClient<std_msgs::msg::Int16>>("temperature");
  }
};
}  // namespace sm_atomic_subscribers_performance_test
