#pragma once

#include <smacc/client_bases/smacc_service_client.h>
#include <std_srvs/srv/set_bool.hpp>
namespace sm_dance_bot_strikes_back
{
namespace cl_service3
{
class ClService3 : public smacc::client_bases::SmaccServiceClient<std_srvs::srv::SetBool>
{
public:
  ClService3()
  {
  }
};
} // namespace cl_service3
} // namespace sm_dance_bot
