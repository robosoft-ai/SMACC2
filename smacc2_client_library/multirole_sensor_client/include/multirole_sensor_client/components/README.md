# Multirole Sensor Client Components

This directory contains reusable components for the multirole sensor client library.

## Components

### CpMessageTimeout

A component that monitors ROS topic message reception and posts timeout events when messages are not received within a specified duration.

#### Features
- Automatic timeout watchdog functionality
- Configurable timeout duration
- Integrates with `CpTopicSubscriber` for message monitoring
- Posts `EvTopicMessageTimeout` events
- Emits `onMessageTimeout_` signal

#### Usage

The `CpMessageTimeout` component is automatically created by `ClMultiroleSensor` when a timeout duration is configured.

**Example 1: Client with timeout**
```cpp
// In your client constructor or initialization
ClMultiroleSensor<std_msgs::msg::String> sensor("/my_topic", rclcpp::Duration(5, 0));
```

**Example 2: Configuring timeout after construction**
```cpp
ClMultiroleSensor<sensor_msgs::msg::LaserScan> lidarClient;
lidarClient.topicName_ = "/scan";
lidarClient.timeout_ = rclcpp::Duration(2, 0);  // 2 second timeout
```

**Example 3: No timeout (optional)**
```cpp
// Timeout component will not be created
ClMultiroleSensor<std_msgs::msg::Int32> sensor("/counter");
// No timeout configured - component gracefully not created
```

#### Event Handling

The timeout component posts `EvTopicMessageTimeout` events that can be used in state transitions:

```cpp
// In your state transition table
typedef mpl::list<
  Transition<EvTopicMessage<ClSensor, OrSensor>, StProcessing>,
  Transition<EvTopicMessageTimeout<ClSensor, OrSensor>, StError>
> reactions;
```

#### Architecture

`CpMessageTimeout` follows the SMACC2 component pattern:
- Inherits from `ISmaccComponent`
- Requires `CpTopicSubscriber<MessageType>` component
- Subscribes to message reception signals to reset timer
- Posts events through the state machine event queue
- Automatic lifecycle management (created/destroyed with client)

#### Design Rationale

The separation of timeout functionality into a component provides:
- **Single Responsibility**: Each component has one clear purpose
- **Reusability**: Can be used with any client that needs timeout monitoring
- **Optional**: Only created when needed, no overhead if not used
- **Composability**: Works seamlessly with other components

## See Also

- `CpTopicSubscriber` - Core SMACC2 component for topic subscription
- `ClMultiroleSensor` - Main client that uses these components
- `CbDefaultMultiRoleSensorBehavior` - Behavior that propagates component events
