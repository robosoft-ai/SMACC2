# Generic Sensor Client

A SMACC2 client library for subscribing to ROS topics with optional timeout watchdog functionality. Built using a pure component-based architecture following SMACC2 best practices.

## Features

- **Component-based architecture** - Uses `CpTopicSubscriber` and `CpMessageTimeout` components
- **Optional timeout watchdog** - Configurable message timeout monitoring
- **Type-safe** - Template-based for any ROS message type
- **Event-driven** - Posts SMACC2 events for message reception and timeouts
- **Flexible configuration** - Configure topic and timeout at construction or runtime

## Architecture

The multirole sensor client follows the **ClKeyboard pattern** where all functionality is implemented through composable components:

```
ClGenericSensor
    ├── CpTopicSubscriber (from smacc2 core)
    │   ├── Subscribes to ROS topic
    │   ├── Posts EvTopicMessage events
    │   └── Posts EvTopicInitialMessage events
    │
    └── CpMessageTimeout (optional)
        ├── Monitors message reception
        ├── Posts EvTopicMessageTimeout events
        └── Emits onMessageTimeout_ signal
```

### Events Posted

- `EvTopicMessage<TSource, TOrthogonal, TMessageType>` - Posted on every message
- `EvTopicInitialMessage<TSource, TOrthogonal, TMessageType>` - Posted on first message only
- `EvTopicMessageTimeout<TSource, TOrthogonal>` - Posted when timeout occurs (if configured)

## Usage

### Basic Usage (No Timeout)

```cpp
#include <cl_generic_sensor/cl_generic_sensor.hpp>

namespace my_sm
{
// Create a client for sensor_msgs::LaserScan
class ClLidar : public cl_generic_sensor::ClGenericSensor<sensor_msgs::msg::LaserScan>
{
public:
  ClLidar()
  {
    this->topicName_ = "/scan";
    // No timeout configured - watchdog disabled
  }
};
}
```

### Usage with Timeout Watchdog

```cpp
#include <cl_generic_sensor/cl_generic_sensor.hpp>

namespace my_sm
{
// Create a client with 5-second timeout
class ClGps : public cl_generic_sensor::ClGenericSensor<sensor_msgs::msg::NavSatFix>
{
public:
  ClGps()
  {
    this->topicName_ = "/gps/fix";
    this->timeout_ = rclcpp::Duration(5, 0);  // 5 second timeout
  }
};
}
```

### Constructor-based Configuration

```cpp
// Direct construction with topic and timeout
ClGenericSensor<std_msgs::msg::String> sensor("/topic_name", rclcpp::Duration(3, 0));
```

### Using in Orthogonals

```cpp
class OrSensor : public smacc2::Orthogonal<OrSensor>
{
public:
  void onInitialize() override
  {
    auto sensor_client = this->createClient<ClLidar>();
  }
};
```

### State Transitions with Events

```cpp
struct StMonitoring : smacc2::SmaccState<StMonitoring, SmMyStateMachine>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    // Transition on message received
    Transition<EvTopicMessage<ClLidar, OrSensor>, StProcessing>,

    // Transition on first message (initialization)
    Transition<EvTopicInitialMessage<ClLidar, OrSensor>, StInitialized>,

    // Transition on timeout (if configured)
    Transition<EvTopicMessageTimeout<ClLidar, OrSensor>, StError>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrSensor, CbDefaultGenericSensorBehavior<ClLidar>>();
  }
};
```

### Using the Default Behavior

The `CbDefaultGenericSensorBehavior` automatically propagates events from components:

```cpp
#include <cl_generic_sensor/client_behaviors/cb_default_generic_sensor_behavior.hpp>

// Use directly
configure_orthogonal<OrSensor, cl_generic_sensor::CbDefaultGenericSensorBehavior<ClLidar>>();

// Or create custom behavior
class CbCustomSensorBehavior
  : public cl_generic_sensor::CbDefaultGenericSensorBehavior<ClLidar>
{
public:
  void onMessageCallback(const sensor_msgs::msg::LaserScan& msg) override
  {
    // Custom processing
    RCLCPP_INFO(getLogger(), "Received scan with %zu points", msg.ranges.size());
  }
};
```

## Components

See [components/README.md](include/cl_generic_sensor/components/README.md) for detailed component documentation.

## Examples

### Example 1: Temperature Sensor with Timeout

```cpp
class ClTemperature
  : public cl_generic_sensor::ClGenericSensor<sensor_msgs::msg::Temperature>
{
public:
  ClTemperature()
  {
    this->topicName_ = "/temperature";
    this->timeout_ = rclcpp::Duration(10, 0);  // 10 second timeout
  }
};

// In your state
typedef mpl::list<
  Transition<EvTopicMessage<ClTemperature, OrSensors>, StProcessTemp>,
  Transition<EvTopicMessageTimeout<ClTemperature, OrSensors>, StSensorFault>
> reactions;
```

### Example 2: Image Stream (No Timeout)

```cpp
class ClCamera
  : public cl_generic_sensor::ClGenericSensor<sensor_msgs::msg::Image>
{
public:
  ClCamera()
  {
    this->topicName_ = "/camera/image_raw";
    // No timeout - high-frequency stream
  }
};
```

## Migration from Legacy Version

If you have code using the old `SmaccSubscriberClient`-based implementation:

**Old:**
```cpp
class ClOldSensor : public smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>
{
  // Timeout logic embedded in client
};
```

**New:**
```cpp
class ClNewSensor : public cl_generic_sensor::ClGenericSensor<std_msgs::msg::String>
{
public:
  ClNewSensor()
  {
    this->topicName_ = "/topic";
    this->timeout_ = rclcpp::Duration(5, 0);  // Optional
  }
};
```

The event types remain the same, so state transition tables don't need changes.

## Benefits of Component-Based Design

1. **Separation of Concerns** - Subscription and timeout are separate components
2. **Reusability** - Components can be used across different clients
3. **Testability** - Each component can be tested independently
4. **Flexibility** - Timeout is optional, no overhead if not needed
5. **Maintainability** - Clear boundaries between functionality
6. **Extensibility** - Easy to add new components without modifying client

## Design Pattern

This client follows the **ClKeyboard pattern** established in SMACC2:
- Client inherits from `ISmaccClient` (not `SmaccSubscriberClient`)
- Functionality provided through components via `onComponentInitialization()`
- Components are created using `createComponent<>()`
- Components communicate via signals and SMACC2 events

## See Also

- [SMACC2 Documentation](https://smacc2.robosoft.ai/)
- [ClKeyboard Client](../../cl_keyboard/) - Reference implementation of component-based pattern
- [SMACC2 Client Library Guide](../../CLAUDE.md)

## License

Apache License 2.0
