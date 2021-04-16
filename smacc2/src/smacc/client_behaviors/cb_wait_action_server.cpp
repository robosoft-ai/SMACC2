#include <smacc/client_behaviors/cb_wait_action_server.h>

namespace smacc
{
namespace client_behaviors
{
CbWaitActionServer::CbWaitActionServer(std::chrono::milliseconds timeout) : timeout_(timeout)
{
}

CbWaitActionServer::~CbWaitActionServer()
{
}

void CbWaitActionServer::executeOnEntry()
{
  if (client_ != nullptr)
  {
    std::shared_ptr<rclcpp_action::ClientBase> client_base = client_->getClientBase();
    RCLCPP_INFO(getLogger(), "[CbWaitActionServer] waiting action server..");
    bool found = client_base->wait_for_action_server(timeout_);

    if (found)
    {
      RCLCPP_INFO(getLogger(), "[CbWaitActionServer] action server already avaliable");
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_INFO(getLogger(), "[CbWaitActionServer] action server not found, timeout");
      this->postFailureEvent();
    }
  }
  else
  {
    RCLCPP_INFO(getLogger(), "[CbWaitActionServer] there is no action client in this orthogonal");
    this->postFailureEvent();
  }
}
}  // namespace client_behaviors
}  // namespace smacc