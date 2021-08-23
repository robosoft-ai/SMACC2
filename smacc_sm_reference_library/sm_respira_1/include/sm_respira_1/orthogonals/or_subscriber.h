#pragma once

#include <sm_respira_1/clients/cl_subscriber/cl_subscriber.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_respira_1
{
using namespace sm_respira_1::cl_subscriber;

class OrSubscriber : public smacc::Orthogonal<OrSubscriber>
{
public:
  virtual void onInitialize() override
  {
    auto subscriber_client = this->createClient<ClSubscriber>();
  }
};
}  // namespace sm_respira_1
