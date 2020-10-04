#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.h>

namespace cl_ros_timer
{

CbTimerCountdownLoop::CbTimerCountdownLoop(unsigned long triggerTickCount)
    : tickTriggerCount_(triggerTickCount),
      tickCounter_(0)
{
}

void CbTimerCountdownLoop::onClientTimerTickCallback()
{
    tickCounter_++;

    if (tickCounter_ % tickTriggerCount_ == 0)
    {
        onTimerTick_();
        postCountDownEvent_();
    }
}

void CbTimerCountdownLoop::onEntry()
{
    this->requiresClient(timerClient_);
    timerClient_->onTimerTick(&CbTimerCountdownLoop::onClientTimerTickCallback, this);
}

void CbTimerCountdownLoop::onExit()
{
}
} // namespace cl_ros_timer