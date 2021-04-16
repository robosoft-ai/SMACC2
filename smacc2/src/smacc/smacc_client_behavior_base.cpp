#include <smacc/smacc_client_behavior.h>

namespace smacc
{
    ISmaccClientBehavior::ISmaccClientBehavior()
    {
        stateMachine_ = nullptr;
        currentState = nullptr;
    }

    ISmaccClientBehavior::~ISmaccClientBehavior()
    {
        RCLCPP_WARN(getNode()->get_logger(),"Client behavior deallocated.");
    }

    std::string ISmaccClientBehavior::getName() const
    {
        return demangleSymbol(typeid(*this).name());
    }

    rclcpp::Node::SharedPtr ISmaccClientBehavior::getNode()
    {
        return this->stateMachine_->getNode();
    }

    rclcpp::Logger ISmaccClientBehavior::getLogger()
    {
        auto nh = this->getNode();
        if(nh != nullptr)
        {
            return nh->get_logger();
        }
        else
        {
            return rclcpp::get_logger("SMACC");
        }
    }

    void ISmaccClientBehavior::runtimeConfigure()
    {
        RCLCPP_DEBUG(getNode()->get_logger(),"[%s] Default empty SmaccClientBehavior runtimeConfigure", this->getName().c_str());
    }

    void ISmaccClientBehavior::executeOnEntry()
    {
        RCLCPP_DEBUG(getNode()->get_logger(),"[%s] Default empty SmaccClientBehavior onEntry", this->getName().c_str());
        this->onEntry();
    }

    void ISmaccClientBehavior::executeOnExit()
    {
        RCLCPP_DEBUG(getNode()->get_logger(),"[%s] Default empty SmaccClientBehavior onExit", this->getName().c_str());
        this->onExit();
    }

    void ISmaccClientBehavior::dispose()
    {

    }
    
} // namespace smacc