#include <smacc/impl/smacc_state_machine_impl.h>
#include <smacc/smacc_client_behavior.h>
#include <smacc/smacc_orthogonal.h>

namespace smacc
{
    void ISmaccOrthogonal::setStateMachine(ISmaccStateMachine *value)
    {
        this->stateMachine_ = value;
        this->onInitialize();
    }

    rclcpp::Node::SharedPtr ISmaccOrthogonal::getNode()
    {
        return this->stateMachine_->getNode();
    }

    void ISmaccOrthogonal::addClientBehavior(std::shared_ptr<smacc::ISmaccClientBehavior> clBehavior)
    {
        if (clBehavior != nullptr)
        {
            RCLCPP_INFO(getNode()->get_logger(),"[Orthogonal %s] adding client behavior: %s", this->getName().c_str(), clBehavior->getName().c_str());
            clBehavior->stateMachine_ = this->getStateMachine();
            clBehavior->currentOrthogonal = this;

            clientBehaviors_.push_back(clBehavior);
        }
        else
        {
            RCLCPP_INFO(getNode()->get_logger(),"[orthogonal %s] no client behaviors in this state", this->getName().c_str());
        }
    }

    void ISmaccOrthogonal::onInitialize()
    {
    }

    std::string ISmaccOrthogonal::getName() const
    {
        return demangleSymbol(typeid(*this).name());
    }

    void ISmaccOrthogonal::runtimeConfigure()
    {
        for (auto &clBehavior : clientBehaviors_)
        {
            RCLCPP_INFO(getNode()->get_logger(),"[Orthogonal %s] runtimeConfigure, current Behavior: %s", this->getName().c_str(),
                     clBehavior->getName().c_str());

            clBehavior->runtimeConfigure();
        }
    }

    void ISmaccOrthogonal::onEntry()
    {
        if (clientBehaviors_.size() > 0)
        {
            for (auto &clBehavior : clientBehaviors_)
            {
                RCLCPP_INFO(getNode()->get_logger(),"[Orthogonal %s] OnEntry, current Behavior: %s", this->getName().c_str(), clBehavior->getName().c_str());

                try
                {
                    clBehavior->executeOnEntry();
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(getNode()->get_logger(),"[ClientBehavior %s] Exception on Entry - continuing with next client behavior. Exception info: %s",
                              clBehavior->getName().c_str(), e.what());
                }
            }
        }
        else
        {
            RCLCPP_INFO(getNode()->get_logger(),"[Orthogonal %s] OnEntry", this->getName().c_str());
        }
    }

    void ISmaccOrthogonal::onExit()
    {
        if (clientBehaviors_.size() > 0)
        {
            for (auto &clBehavior : clientBehaviors_)
            {
                RCLCPP_INFO(getNode()->get_logger(),"[Orthogonal %s] OnExit, current Behavior: %s", this->getName().c_str(), clBehavior->getName().c_str());
                try
                {
                    clBehavior->executeOnExit();
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(getNode()->get_logger(),"[ClientBehavior %s] Exception onExit - continuing with next client behavior. Exception info: %s", clBehavior->getName().c_str(), e.what());
                }
            }

            int i = 0;
            for (auto &clBehavior : clientBehaviors_)
            {
                clBehavior->dispose();
                clientBehaviors_[i] = nullptr;
            }

            clientBehaviors_.clear();
        }
        else
        {
            RCLCPP_INFO(getNode()->get_logger(),"[Orthogonal %s] OnExit", this->getName().c_str());
        }
    }
} // namespace smacc