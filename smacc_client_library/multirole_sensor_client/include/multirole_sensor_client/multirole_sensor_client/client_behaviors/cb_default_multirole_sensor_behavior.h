#pragma once

#include <smacc/smacc_client_behavior.h>
#include <multirole_sensor_client/cl_multirole_sensor.h>

namespace cl_multirole_sensor
{
template <typename ClientType>
class CbDefaultMultiRoleSensorBehavior : public smacc::SmaccClientBehavior
{
public:
  typedef typename ClientType::TMessageType TMessageType;

  ClientType *sensor_;

  CbDefaultMultiRoleSensorBehavior()
  {
    sensor_ = nullptr;
  }

  static std::string getEventLabel()
  {
    // show ros message type
    return demangleSymbol(typeid(TMessageType).name());
  }

  std::function<void()> deferedEventPropagation;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    deferedEventPropagation = [=]() {
      // just propagate the client events from this client behavior source.
      sensor_->onMessageReceived(&CbDefaultMultiRoleSensorBehavior<ClientType>::propagateEvent<EvTopicMessage<TSourceObject, TOrthogonal>>, this);
      sensor_->onFirstMessageReceived(&CbDefaultMultiRoleSensorBehavior<ClientType>::propagateEvent<EvTopicInitialMessage<TSourceObject, TOrthogonal>>, this);
      sensor_->onMessageTimeout(&CbDefaultMultiRoleSensorBehavior<ClientType>::propagateEvent2<EvTopicMessageTimeout<TSourceObject, TOrthogonal>>, this);
    };
  }

  template <typename EvType>
  void propagateEvent(const TMessageType &msg)
  {
    this->postEvent<EvType>();
  }

  template <typename EvType>
  void propagateEvent2(const ros::TimerEvent &tdata)
  {
    this->postEvent<EvType>();
  }

  virtual void onEntry() override
  {
    RCLCPP_INFO(getNode()->get_logger(), "[CbDefaultMultiRoleSensorBehavior] onEntry. Requires client of type '%s'", demangleSymbol<ClientType>().c_str());

    this->requiresClient(sensor_);

    if (sensor_ == nullptr)
    {
      RCLCPP_FATAL_STREAM(getNode()->get_logger(),"Sensor client behavior needs a client of type: " << demangleSymbol<ClientType>() << " but it is not found.");
    }
    else
    {
      deferedEventPropagation();
      RCLCPP_INFO(getNode()->get_logger(),"[CbDefaultMultiRoleSensorBehavior] onEntry. sensor initialize");
      sensor_->initialize();
    }
  }

  void onExit()
  {
  }

  virtual void onMessageCallback(const TMessageType &msg)
  {
    // empty to fill by sensor customization based on inheritance
  }
};
} // namespace smacc
