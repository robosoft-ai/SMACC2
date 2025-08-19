# SMACC2 Client Library Development Guide

This document provides comprehensive guidance for developing SMACC2 clients, based on the established patterns and architecture found in this client library.

## 📋 Table of Contents

- [Overview](#overview)
- [Architecture Patterns](#architecture-patterns)
- [Client Types](#client-types)
- [Development Workflow](#development-workflow)
- [Best Practices](#best-practices)
- [Template Usage](#template-usage)
- [Testing Guidelines](#testing-guidelines)
- [Common Patterns](#common-patterns)
- [Troubleshooting](#troubleshooting)

## 🎯 Overview

The SMACC2 Client Library provides modular, reusable clients for robot behaviors within the SMACC2 state machine framework. Each client encapsulates specific functionality and can be used across different state machines.

### Current Clients

| Client | Purpose | Communication Pattern |
|--------|---------|----------------------|
| `nav2z_client` | Navigation with Nav2 | Action-based |
| `moveit2z_client` | Manipulation with MoveIt2 | Direct API calls |
| `keyboard_client` | Keyboard input handling | Subscriber-based |
| `ros_timer_client` | Timer-based behaviors | Timer callbacks |
| `ros_publisher_client` | Publishing messages | Publisher-based |
| `http_client` | HTTP requests | Custom protocol |
| `multirole_sensor_client` | Sensor data management | Subscriber-based |
| `lifecyclenode_client` | ROS2 lifecycle management | Service calls |

## 🏗️ Architecture Patterns

### Core Components

Every SMACC2 client follows a three-layer architecture:

1. **Client Layer** (`cl_*.hpp/cpp`)
   - Inherits from SMACC2 base classes
   - Manages ROS2 communication
   - Provides public API

2. **Behavior Layer** (`client_behaviors/cb_*.hpp/cpp`)
   - Implements specific actions/behaviors
   - Handles state transitions via events
   - Manages behavior lifecycle

3. **Component Layer** (`components/cp_*.hpp/cpp`)
   - Provides reusable functionality
   - Manages internal state and data
   - Offers utility services

### Inheritance Hierarchy

```cpp
// For action-based clients
class MyClient : public smacc2::client_bases::SmaccActionClientBase<ActionType>

// For general clients
class MyClient : public smacc2::ISmaccClient

// For subscriber clients  
class MyClient : public smacc2::client_bases::SmaccSubscriberClient<MessageType>

// For behaviors
class MyBehavior : public smacc2::SmaccClientBehavior

// For components
class MyComponent : public smacc2::ISmaccComponent
```

## 🔧 Client Types

### 1. Action-Based Clients

**Used for:** Long-running operations with feedback (navigation, manipulation)

**Example:** `nav2z_client` using `NavigateToPose` action

```cpp
class ClNav2Z : public smacc2::client_bases::SmaccActionClientBase<nav2_msgs::action::NavigateToPose>
{
public:
  using smacc2::client_bases::SmaccActionClientBase<nav2_msgs::action::NavigateToPose>::GoalHandle;
  typedef smacc2::SmaccSignal<void(const WrappedResult &)> SmaccNavigateResultSignal;
  
  ClNav2Z(std::string actionName = "/navigate_to_pose");
  virtual ~ClNav2Z();
};
```

### 2. Topic-Based Clients

**Used for:** Publish/subscribe communication patterns

**Example:** `keyboard_client` subscribing to key events

```cpp
class ClKeyboard : public smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::UInt16>
{
public:
  ClKeyboard();
  virtual ~ClKeyboard();
  
  virtual void onInitialize() override;
  
private:
  void onKeyPress(const std_msgs::msg::UInt16::SharedPtr msg);
};
```

### 3. Service-Based Clients

**Used for:** Request/response interactions

**Example:** `lifecyclenode_client` calling lifecycle services

```cpp
class ClLifecycleNode : public smacc2::ISmaccClient
{
public:
  ClLifecycleNode(std::string serviceName);
  
  bool changeState(std::uint8_t transition);
  
private:
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
};
```

### 4. Timer-Based Clients

**Used for:** Periodic operations and delays

**Example:** `ros_timer_client` for timed behaviors

```cpp
class ClRosTimer : public smacc2::ISmaccClient
{
public:
  ClRosTimer(rclcpp::Duration duration, bool oneshot = true);
  
  virtual void onInitialize() override;
  
  smacc2::SmaccSignal<void()> onTimerTick_;
  
private:
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;
};
```

## 🚀 Development Workflow

### 1. Planning Phase

Before creating a new client, determine:
- **Communication pattern** (action, topic, service, timer)
- **Dependencies** (ROS2 packages, external libraries)
- **Behaviors needed** (what actions the client will perform)
- **Components required** (utility classes, state management)

### 2. Setup Phase

```bash
# 1. Copy template client
cp -r template_client my_new_client
cd my_new_client

# 2. Update package files
# - Update package.xml (name, description, dependencies)
# - Update CMakeLists.txt (project name, source files)

# 3. Rename files
mv include/template_client include/my_new_client
mv src/template_client src/my_new_client

# 4. Update include paths and namespaces
find . -name "*.hpp" -o -name "*.cpp" | xargs sed -i 's/template_client/my_new_client/g'
find . -name "*.hpp" -o -name "*.cpp" | xargs sed -i 's/cl_template/cl_my_new/g'
```

### 3. Implementation Phase

1. **Client Implementation:**
   ```cpp
   // In include/my_new_client/cl_my_new.hpp
   namespace cl_my_new
   {
     class ClMyNew : public smacc2::ISmaccClient
     {
     public:
       ClMyNew(/* parameters */);
       virtual ~ClMyNew();
       
       virtual void onInitialize() override;
       
       // Public API methods
       void startOperation();
       void stopOperation();
       
       // Signals for event communication
       smacc2::SmaccSignal<void()> onOperationComplete_;
       
     private:
       // ROS2 communication objects
       // Internal state variables
     };
   }
   ```

2. **Behavior Implementation:**
   ```cpp
   // In include/my_new_client/client_behaviors/cb_my_behavior.hpp
   class CbMyBehavior : public smacc2::SmaccClientBehavior
   {
   public:
     virtual void onEntry() override;
     virtual void onExit() override;
     
     template <typename TOrthogonal, typename TSourceObject>
     void onOrthogonalAllocation()
     {
       postSuccessEvent_ = [this]() {
         this->template postEvent<EvSuccess<TSourceObject, TOrthogonal>>();
       };
     }
     
   private:
     ClMyNew* client_;
     std::function<void()> postSuccessEvent_;
   };
   ```

3. **Component Implementation:**
   ```cpp
   // In include/my_new_client/components/cp_my_component.hpp
   class CpMyComponent : public smacc2::ISmaccComponent
   {
   public:
     CpMyComponent();
     virtual ~CpMyComponent();
     
     virtual void onInitialize() override;
     
     // Component API methods
     void configure(/* parameters */);
     bool isReady() const;
     
   private:
     // Component state and utilities
   };
   ```

### 4. Testing Phase

```bash
# Build the client
colcon build --packages-select my_new_client

# Run tests (if implemented)
colcon test --packages-select my_new_client

# Integration test with a simple state machine
```

## ✨ Best Practices

### Naming Conventions

- **Packages:** `snake_case` (e.g., `nav2z_client`)
- **Classes:** `PascalCase` with prefixes:
  - Clients: `Cl` prefix (e.g., `ClNav2Z`)
  - Behaviors: `Cb` prefix (e.g., `CbNavigateForward`)  
  - Components: `Cp` prefix (e.g., `CpOdomTracker`)
- **Namespaces:** Match package name (e.g., `cl_nav2z`)
- **Events:** `Ev` prefix (e.g., `EvNavigationSuccess`)

### Code Organization

```
my_client/
├── include/my_client/
│   ├── cl_my_client.hpp              # Main client
│   ├── client_behaviors.hpp          # Behavior includes (optional)
│   ├── client_behaviors/
│   │   ├── cb_behavior1.hpp
│   │   └── cb_behavior2.hpp
│   └── components/
│       ├── cp_component1.hpp
│       └── cp_component2.hpp
├── src/my_client/
│   ├── cl_my_client.cpp
│   ├── client_behaviors/
│   │   ├── cb_behavior1.cpp
│   │   └── cb_behavior2.cpp
│   └── components/
│       ├── cp_component1.cpp
│       └── cp_component2.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

### Error Handling

```cpp
// Always validate in onEntry()
void CbMyBehavior::onEntry()
{
  requiresClient(client_);
  if (!client_)
  {
    RCLCPP_ERROR(getLogger(), "Failed to get client");
    if (postFailureEvent_)
      postFailureEvent_();
    return;
  }
  
  // Continue with behavior logic...
}
```

### Thread Safety

```cpp
class ClMyClient : public smacc2::ISmaccClient
{
private:
  std::mutex state_mutex_;
  std::atomic<bool> is_active_;
  
public:
  void updateState()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Modify shared state safely
  }
};
```

### Lifecycle Management

```cpp
// Proper cleanup in destructors
ClMyClient::~ClMyClient()
{
  if (timer_)
    timer_->cancel();
    
  if (subscriber_)
    subscriber_.reset();
    
  RCLCPP_DEBUG(getLogger(), "Client destroyed");
}
```

## 📝 Template Usage

The `template_client` provides ready-to-use examples for:

### Quick Start

1. **Copy template:**
   ```bash
   cp -r template_client my_custom_client
   ```

2. **Update package.xml:**
   ```xml
   <name>my_custom_client</name>
   <description>My custom SMACC2 client</description>
   <maintainer email="you@example.com">Your Name</maintainer>
   ```

3. **Update CMakeLists.txt:**
   ```cmake
   project(my_custom_client)
   ```

4. **Rename and customize** source files as needed

### Template Features

- **Multiple client patterns** (action, topic, service)
- **4 behavior types** (basic, async, periodic, conditional)
- **4 component types** (data storage, state tracking, transforms, config)
- **Complete build configuration**
- **Comprehensive documentation**

## 🧪 Testing Guidelines

### Unit Tests

```cpp
// test/test_my_client.cpp
#include <gtest/gtest.h>
#include <my_client/cl_my_client.hpp>

class MyClientTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    client_ = std::make_shared<cl_my_client::ClMyClient>();
  }
  
  std::shared_ptr<cl_my_client::ClMyClient> client_;
};

TEST_F(MyClientTest, TestInitialization)
{
  EXPECT_NO_THROW(client_->onInitialize());
}
```

### Integration Tests

Create simple state machines to test client behaviors:

```cpp
// test/integration_test.cpp
struct TestState : smacc2::SmaccState<TestState, TestStateMachine>
{
  typedef mpl::list<
    smacc2::Transition<cl_my_client::EvSuccess<CbMyBehavior, OrthogonalClient>, TestState2>
  > reactions;
  
  static void staticConfigure()
  {
    configure_orthogonal<OrthogonalClient, CbMyBehavior>();
  }
};
```

## 🔍 Common Patterns

### Event-Driven Communication

```cpp
// In behavior header
template <typename TOrthogonal, typename TSourceObject>
void onOrthogonalAllocation()
{
  postSuccessEvent_ = [this]() {
    this->template postEvent<EvSuccess<TSourceObject, TOrthogonal>>();
  };
}

// In behavior implementation
void CbMyBehavior::onSomeCondition()
{
  if (postSuccessEvent_)
    postSuccessEvent_();
}
```

### Signal/Slot Pattern

```cpp
// In client
class ClMyClient : public smacc2::ISmaccClient
{
public:
  smacc2::SmaccSignal<void(int)> onDataReceived_;
  
private:
  void handleData(int data)
  {
    onDataReceived_(data);
  }
};

// In behavior
void CbMyBehavior::onEntry()
{
  connection_ = client_->onDataReceived_.connect(
    std::bind(&CbMyBehavior::onDataReceived, this, std::placeholders::_1));
}
```

### Component Dependency

```cpp
void CbMyBehavior::onEntry()
{
  // Get required client
  requiresClient(client_);
  
  // Get optional component
  auto component = client_->getComponent<CpMyComponent>();
  if (component)
  {
    component->configure(options_);
  }
}
```

### Configuration Pattern

```cpp
struct CbMyBehaviorOptions
{
  std::optional<double> timeout;
  std::optional<int> retries;
  std::optional<std::string> mode;
};

class CbMyBehavior : public smacc2::SmaccClientBehavior
{
public:
  CbMyBehaviorOptions options;
  
  CbMyBehavior(const CbMyBehaviorOptions& opts = {})
  : options(opts) {}
};
```

## 🛠️ Troubleshooting

### Build Issues

**Problem:** "Client does not name a type"
```bash
# Solution: Check include paths and forward declarations
# Ensure proper header guards and namespace usage
```

**Problem:** Template compilation errors
```bash
# Solution: Check template parameter matching
# Verify event types match between behavior and state machine
```

### Runtime Issues  

**Problem:** Client not initialized
```cpp
// Solution: Verify client allocation in orthogonal
configure_orthogonal<OrthogonalClient, ClMyClient>();
```

**Problem:** Events not firing
```cpp
// Solution: Check event template parameters
this->template postEvent<EvSuccess<TSourceObject, TOrthogonal>>();
```

### Performance Issues

**Problem:** High CPU usage
```cpp
// Solution: Check timer frequencies and callback efficiency
// Use appropriate thread-safety mechanisms
```

## 📚 References

### SMACC2 Documentation
- [SMACC2 GitHub Repository](https://github.com/robosoft-ai/SMACC2)
- [State Machine Concepts](https://github.com/robosoft-ai/SMACC2/blob/main/docs/)

### Example Clients
- **nav2z_client** - Complex action-based client with custom planners
- **moveit2z_client** - MoveIt2 integration with trajectory planning
- **keyboard_client** - Simple subscriber-based input handling
- **ros_timer_client** - Timer-based behavior coordination

### ROS2 Integration
- [ROS2 Client Libraries](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Client-Libraries.html)
- [ROS2 Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [ROS2 Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

---

## 🤝 Contributing

When contributing new clients or improvements:

1. **Follow established patterns** from existing clients
2. **Add comprehensive documentation** and examples
3. **Include unit and integration tests**
4. **Update this CLAUDE.md** with new patterns or lessons learned
5. **Follow SMACC2 coding standards** and review guidelines

## 📄 License

This documentation and associated code templates are licensed under the Apache-2.0 license, consistent with the SMACC2 project.