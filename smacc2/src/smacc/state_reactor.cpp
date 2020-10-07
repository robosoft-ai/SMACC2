#include <smacc/smacc_state_reactor.h>
#include <smacc/smacc_state.h>
#include "rclcpp/rclcpp.hpp"

namespace smacc
{

StateReactor::StateReactor()
{
}

void StateReactor::initialize(smacc::ISmaccState *ownerState)
{
    this->ownerState = ownerState;
    this->onInitialized();
}

void StateReactor::onInitialized()
{
}

void StateReactor::onEventNotified(const std::type_info */*eventType*/)
{
}

void StateReactor::onEntry()
{

}

void StateReactor::onExit()
{

}

void StateReactor::update()
{
    if (this->triggers())
    {
        RCLCPP_INFO(getNode()->get_logger(),"State reactor base REALLY TRIGGERS!!");
        this->postEventFn();
    }
}

rclcpp::Node::SharedPtr StateReactor::getNode()
{
    return ownerState->getNode();
}

namespace introspection
{
void StateReactorHandler::configureStateReactor(std::shared_ptr<smacc::StateReactor> sb)
{
    for (auto callback : this->callbacks_)
    {
        callback.fn(sb);
    }
}

void EventGeneratorHandler::configureEventGenerator(std::shared_ptr<smacc::SmaccEventGenerator> eg)
{
    for (auto callback : this->callbacks_)
    {
        callback.fn(eg);
    }
}
} // namespace introspection
} // namespace smacc
