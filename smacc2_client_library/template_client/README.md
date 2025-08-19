# SMACC2 Template Client

A comprehensive template for creating new SMACC2 clients with proper architecture patterns, behaviors, and components.

## Overview

This template provides a complete example of how to structure and implement SMACC2 clients following the established patterns found in the SMACC2 client library. It includes:

- **Client implementations** for different ROS2 communication patterns
- **Client behaviors** with various operational modes  
- **Reusable components** for common functionality
- **Proper CMake and package configuration**

## Architecture

### Client Types Supported

1. **Topic-Based Client** (`ClTemplate`)
   - Publisher/Subscriber pattern
   - Signal-based event communication
   - Thread-safe state management

2. **Action-Based Client** (commented example)
   - Action server integration
   - Goal/Result handling
   - Cancellation support

3. **Service-Based Client** (commented example)
   - Synchronous/Asynchronous service calls
   - Response handling

### Behavior Types

1. **CbTemplateBehavior** - Basic behavior with:
   - Entry/Exit lifecycle
   - Event posting
   - Timeout handling
   - Retry logic

2. **CbTemplateAsyncBehavior** - Asynchronous operations:
   - Threading support
   - Cancellation handling
   - Async completion callbacks

3. **CbTemplatePeriodicBehavior** - Periodic operations:
   - Timer-based execution
   - Pause/Resume capability
   - Configurable periods

4. **CbTemplateConditionalBehavior** - Condition-based execution:
   - Custom condition functions
   - Timeout handling
   - Success/Failure events

### Component Types

1. **CpTemplateDataStorage** - Data management:
   - Key-value storage
   - Time-series buffers
   - Thread-safe access

2. **CpTemplateStateTracker** - State management:
   - State transitions
   - History tracking  
   - Change callbacks

3. **CpTemplateTransformManager** - Transform handling:
   - Pose tracking
   - Distance calculations
   - Frame management

4. **CpTemplateConfigManager** - Configuration:
   - Parameter management
   - YAML load/save
   - ROS parameter integration

## Usage Instructions

### 1. Copy and Rename

```bash
cp -r template_client my_custom_client
cd my_custom_client
```

### 2. Update Package Configuration

**package.xml:**
- Change `<name>template_client</name>` to your client name
- Update maintainer, author, description
- Modify dependencies as needed

**CMakeLists.txt:**
- Change `project(template_client)` to your project name
- Update source file references if renamed

### 3. Rename Source Files

```bash
# Rename main client files
mv include/template_client include/my_custom_client
mv src/template_client src/my_custom_client

# Update include paths in all files
find . -name "*.hpp" -o -name "*.cpp" | xargs sed -i 's/template_client/my_custom_client/g'
```

### 4. Update Code

1. **Namespace**: Change `cl_template` to your namespace
2. **Class names**: Rename `ClTemplate` to your client class name
3. **Include guards**: Update header guards
4. **Events**: Rename event types as appropriate

### 5. Customize Implementation

- Modify client functionality in `cl_*.cpp`
- Add/remove behaviors as needed
- Customize components for your use case
- Update ROS2 message/action types

## Example Usage in State Machine

```cpp
#include <my_custom_client/cl_my_custom.hpp>
#include <my_custom_client/client_behaviors/cb_my_custom_behavior.hpp>

namespace my_state_machine
{
  // State using the custom client
  struct MyState : smacc2::SmaccState<MyState, MyStateMachine>
  {
    using SmaccState::SmaccState;
    
    typedef mpl::list<
      // Client behaviors
      smacc2::Transition<cl_my_custom::EvSuccess<CbMyCustomBehavior, OrthogonalClient>, NextState>,
      smacc2::Transition<cl_my_custom::EvFailure<CbMyCustomBehavior, OrthogonalClient>, ErrorState>
    > reactions;
    
    static void staticConfigure()
    {
      // Configure client behavior
      configure_orthogonal<OrthogonalClient, CbMyCustomBehavior>();
    }
  };
}
```

## Building

```bash
cd /path/to/your/workspace
colcon build --packages-select my_custom_client
source install/setup.bash
```

## Testing

The template includes test stubs in CMakeLists.txt. Uncomment and implement tests:

```bash
colcon test --packages-select my_custom_client
```

## Dependencies

### Required
- `smacc2` - Core SMACC2 framework
- `rclcpp` - ROS2 C++ client library
- `std_msgs` - Standard ROS2 messages
- `geometry_msgs` - Geometry messages
- `yaml-cpp` - YAML parsing library

### Optional
- `rclcpp_action` - For action-based clients
- `nav2_msgs` - For navigation clients
- `moveit_msgs` - For manipulation clients

## Best Practices

1. **Follow SMACC2 patterns** established in existing clients
2. **Use proper event posting** for state machine communication
3. **Implement thread-safe** operations where needed
4. **Provide configuration options** via behavior option structs
5. **Add comprehensive logging** for debugging
6. **Write unit tests** for critical functionality
7. **Document public APIs** with Doxygen comments

## Common Patterns

### Event Posting
```cpp
template <typename TOrthogonal, typename TSourceObject>
void onOrthogonalAllocation()
{
  postSuccessEvent_ = [this]() {
    this->template postEvent<EvSuccess<TSourceObject, TOrthogonal>>();
  };
}
```

### Signal Connections
```cpp
data_connection_ = client_->onDataReceived_.connect(
  std::bind(&MyBehavior::onDataReceived, this));
```

### Component Usage
```cpp
void onEntry() override
{
  requiresComponent(component_);
  component_->configure(options);
}
```

## Troubleshooting

- **Build errors**: Check dependencies and CMakeLists.txt
- **Runtime errors**: Verify client initialization and connections
- **Event issues**: Ensure proper event posting and template parameters
- **Component errors**: Check component initialization and requirements

## Contributing

When contributing improvements to this template:

1. Maintain compatibility with existing patterns
2. Add comprehensive examples
3. Update documentation
4. Test with multiple client types
5. Follow SMACC2 coding standards

## License

This template client is licensed under the Apache-2.0 license.