#pragma once
#include <smacc/client_bases/smacc_action_client_base.h>
#include <smacc/smacc_asynchronous_client_behavior.h>

namespace smacc
{
namespace client_behaviors
{
using namespace smacc::client_bases;

// waits the action server is available in the current orthogonal
class CbWaitActionServer : public smacc::SmaccAsyncClientBehavior
{
public:
  CbWaitActionServer(std::chrono::milliseconds timeout);
  virtual ~CbWaitActionServer();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
    this->requiresClient(client_);
  }

  void executeOnEntry() override;

private:
  ISmaccActionClient * client_;
  std::chrono::milliseconds timeout_;
};
}  // namespace client_behaviors
}  // namespace smacc
