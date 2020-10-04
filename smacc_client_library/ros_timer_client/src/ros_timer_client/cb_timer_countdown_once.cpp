#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>

namespace cl_ros_timer
{

CbTimerCountdownOnce::CbTimerCountdownOnce(unsigned long triggerTickCount)
    : tickCounter_(0)
      , tickTriggerCount_(triggerTickCount)
{
}

void CbTimerCountdownOnce::onClientTimerTickCallback()
{
    tickCounter_++;

    if (tickCounter_ % tickTriggerCount_ == 0)
    {
        onTimerTick_();
        postCountDownEvent_();
    }
}

void CbTimerCountdownOnce::onEntry()
{
    this->requiresClient(timerClient_);
    timerClient_->onTimerTick(&CbTimerCountdownOnce::onClientTimerTickCallback, this);
}

void CbTimerCountdownOnce::onExit()
{
}
} // namespace cl_ros_timer