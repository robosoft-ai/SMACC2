
//////////////////////////////////////////////////////////////////////////////

# Code style guidelines

### Naming Conventions

- **Packages:** `snake_case` (e.g., `cl_nav2z`)
- **Classes:** `PascalCase` with prefixes:
  - Clients: `Cl` prefix (e.g., `ClNav2Z`)
  - Client Behaviors: `Cb` prefix (e.g., `CbNavigateForward`)  
  - Components: `Cp` prefix (e.g., `CpOdomTracker`)
- **Namespaces:** Match package name (e.g., `cl_nav2z`)
- **Events:** `Ev` prefix (e.g., `EvNavigationSuccess`)


### Code Organization

```
cl_example/
├── include/cl_example/
│   ├── cl_example.hpp              # Main client
│   ├── client_behaviors.hpp          # Behavior includes (optional)
│   ├── client_behaviors/
│   │   ├── cb_behavior_1.hpp
│   │   └── cb_behavior_2.hpp
│   └── components/
│       ├── cp_component_1.hpp
│       └── cp_component_2.hpp
├── src/cl_example/
│   ├── cl_example.cpp
│   ├── client_behaviors/
│   │   ├── cb_behavior_1.cpp
│   │   └── cb_behavior_2.cpp
│   └── components/
│       ├── cp_component_1.cpp
│       └── cp_component_2.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```


//////////////////////////////////////////////////////////////////////////////

# Client Library Binary representation
so, header files, cpp, state machine.
 
 - A client library is a ros package that generates a library (.so file)
 - .so files cannot be executed directly, they are just linked to other executables and loaded at runtime (startup)
 - The .so file could be a combination of one or multiple cpp files compiled together (but there is no main function in the cpp files)
 - The code body of the clients, components and client behaviors such as onEntry or onExit are compiled into the .so file
 - The onEntry and onExit function code body is not shown in the header file, we only see the declaration (ended in ;)
 - The binary implementation of a function depends on which cpp the body of the function is defined. 
 - If the body of the function is defined in the header file, it will be compiled into the cpp that includes the header file, not in the .so file.
 - We could generate the .so file with one single cpp file that includes all the cpp files of the package. However, the key point is not how many cpp files we have but where the body of the functions are defined. If they are defined in the header file, they will be compiled into the executable that includes the header file, not in the .so file.
 - If we define the body of the functions in the hpp files, the body of the functions will be compiled both in the .so file and in the executable that includes the header file. This is not a problem because the linker will take care of it. However, it increases the compilation time because every time we change a header file, all the cpp files that include the header file need to be recompiled.


//////////////////////////////////////////////////////////////////////////////

# Overview

The SMACC2 Client Library provides modular, reusable clients for robot behaviors within the SMACC2 state machine framework. Each client encapsulates specific functionality and can be used across different state machines. The preferred style for clients is a **pure component-based architecture** where clients act as orchestrators that compose reusable components.

### Current Clients

| Client | Purpose | Core Components |
|--------|---------|-----------------|
| `cl_nav2z` | Navigation with Nav2 | CpActionClient, CpNav2ActionInterface |
| `cl_moveit2z` | Manipulation with MoveIt2 | CpMoveItInterface |
| `cl_keyboard` | Keyboard input handling | CpTopicSubscriber, CpKeyboardListener1 |
| `cl_ros2_timer` | Timer-based behaviors | CpRos2Timer, CpTimerListener1 |
| `cl_http` | HTTP requests | CpHttpConnectionManager, CpHttpRequestExecutor |
| `cl_lifecycle_node` | ROS2 lifecycle management | CpLifecycleEventMonitor |
| `cl_generic_sensor` | Generic sensor input | CpTopicSubscriber, CpMessageTimeout |

## Architecture Patterns

### Core Components

Every SMACC2 client follows an architecture with 3 object types:

1. **Client Objects** 
   - Usually, a Client is an entity that is a counterpart of a remote node, in a form of client server relationship. But this is not always the case.

2. **Client Behavior Objects**
   - Implements specific actions/behaviors

3. **Component Objects**
   - Provides reusable functionality
   - Manages internal state and data
   - Offers utility services


//////////////////////////////////////////////////////////////////////////////

# Pure Component-Based Architecture

All SMACC2 clients follow a **pure orchestrator pattern**:

1. **Clients are orchestrators only** - They create and configure components but implement no business logic
2. **Components implement functionality** - All actual functionality lives in reusable components
3. **Behaviors consume components** - Behaviors access components via `requiresComponent()`


### Inheritance Hierarchy

```cpp
// Client: Pure orchestrator - creates components only
class ClExample : public smacc2::ISmaccClient

// Component: Implements actual functionality
class CpExample : public smacc2::ISmaccComponent

// Synchronous behavior: Accesses components via requiresComponent()
class CbSyncBehavior : public smacc2::SmaccClientBehavior

// Asynchronous behavior: For long-running operations
class CbAsyncBehavior : public smacc2::SmaccAsyncClientBehavior
```


### Key Template Methods

| Method | Called When | Purpose |
|--------|-------------|---------|
| `onComponentInitialization<TOrthogonal, TClient>()` | Orthogonal init | Create components with type context |
| `onStateOrthogonalAllocation<TOrthogonal, TSourceObject>()` | State entry | Set up event posting lambdas |
| `onInitialize()` | Component creation | Component setup, require dependencies |
| `onEntry()` / `onExit()` | State transitions | Behavior execution lifecycle |


//////////////////////////////////////////////////////////////////////////////

# Framework Core Components

Located in `smacc2/include/smacc2/client_core_components/`

### CpActionClient<ActionType>
Generic ROS2 action client with signals for result handling.
- **Signals:** `onActionSucceeded_`, `onActionAborted_`, `onActionCancelled_`, `onActionFeedback_`
- **Methods:** `sendGoal()`, `cancelGoal()`, `waitForServer()`
- [cp_action_client.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2/include/smacc2/client_core_components/cp_action_client.hpp)
- `src/SMACC2/smacc2/include/smacc2/client_core_components/cp_action_client.hpp`

### CpTopicSubscriber<MessageType>
Generic ROS2 topic subscription with event posting.
- **Signals:** `onFirstMessageReceived_`, `onMessageReceived_`
- **Events:** `EvTopicMessage`, `EvTopicInitialMessage`
- [cp_topic_subscriber.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2/include/smacc2/client_core_components/cp_topic_subscriber.hpp)
- `src/SMACC2/smacc2/include/smacc2/client_core_components/cp_topic_subscriber.hpp`

### CpTopicPublisher<MessageType>
Generic ROS2 topic publisher with QoS configuration.
- **Methods:** `publish(const MessageType & msg)`
- [cp_topic_publisher.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2/include/smacc2/client_core_components/cp_topic_publisher.hpp)
- `src/SMACC2/smacc2/include/smacc2/client_core_components/cp_topic_publisher.hpp`

### CpServiceClient<ServiceType>
Generic ROS2 service client with async/sync support.
- **Signals:** `onServiceResponse_`, `onServiceFailure_`
- **Methods:** `sendRequest()`, `sendRequestSync()`
- [cp_service_client.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2/include/smacc2/client_core_components/cp_service_client.hpp)
- `src/SMACC2/smacc2/include/smacc2/client_core_components/cp_service_client.hpp`

### CpRos2Timer
Timer component for periodic or one-shot execution.
- **Signal:** `onTimerTick_`
- **Methods:** `startTimer()`, `stopTimer()`
- [cp_ros2_timer.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2/include/smacc2/client_core_components/cp_ros2_timer.hpp)
- `src/SMACC2/smacc2/include/smacc2/client_core_components/cp_ros2_timer.hpp`


//////////////////////////////////////////////////////////////////////////////

# Client Packages Reference

| Client | Purpose | Main Header |
|--------|---------|-------------|
| cl_nav2z | Navigation with Nav2 | [cl_nav2z.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/cl_nav2z.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/cl_nav2z.hpp` |
| cl_moveit2z | Manipulation with MoveIt2 | [cl_moveit2z.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/cl_moveit2z.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/cl_moveit2z.hpp` |
| cl_keyboard | Keyboard input handling | [cl_keyboard.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_keyboard/include/cl_keyboard/cl_keyboard.hpp) / `src/SMACC2/smacc2_client_library/cl_keyboard/include/cl_keyboard/cl_keyboard.hpp` |
| cl_ros2_timer | Timer-based behaviors | [cl_ros_timer.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/cl_ros_timer.hpp) / `src/SMACC2/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/cl_ros_timer.hpp` |
| cl_http | HTTP requests | [cl_http_client.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_http/include/cl_http/cl_http_client.hpp) / `src/SMACC2/smacc2_client_library/cl_http/include/cl_http/cl_http_client.hpp` |
| cl_lifecycle_node | ROS2 lifecycle management | [cl_lifecycle_node.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/cl_lifecycle_node.hpp) / `src/SMACC2/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/cl_lifecycle_node.hpp` |
| cl_mission_tracker | Mission state management | [cl_mission_tracker.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_mission_tracker/include/cl_mission_tracker/cl_mission_tracker.hpp) / `src/SMACC2/smacc2_client_library/cl_mission_tracker/include/cl_mission_tracker/cl_mission_tracker.hpp` |
| cl_isaac_apriltag | AprilTag detection | [cl_isaac_apriltag.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_isaac_apriltag/include/cl_isaac_apriltag/cl_isaac_apriltag.hpp) / `src/SMACC2/smacc2_client_library/cl_isaac_apriltag/include/cl_isaac_apriltag/cl_isaac_apriltag.hpp` |
| cl_foundation_pose | Object pose tracking | [cl_foundation_pose.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_foundation_pose/include/cl_foundation_pose/cl_foundation_pose.hpp) / `src/SMACC2/smacc2_client_library/cl_foundation_pose/include/cl_foundation_pose/cl_foundation_pose.hpp` |



//////////////////////////////////////////////////////////////////////////////

# Client Behavior Patterns

Behaviors access components via `requiresComponent()` and connect to component signals.
   
## 1. Action-Based Behaviors

**Purpose:** Long-running, goal-oriented operations with feedback (navigation, manipulation).

**Base Class:** `SmaccAsyncClientBehavior`

**Key Characteristics:**
- Inherits from `SmaccAsyncClientBehavior` for non-blocking execution
- Connects to action client signals (`onSucceeded_`, `onAborted_`)
- Posts `EvCbSuccess` / `EvCbFailure` events on completion

**Best Reference:**
- [cb_navigate_global_position.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_navigate_global_position.hpp)
- `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_navigate_global_position.hpp`

**Additional Examples:**

| Behavior | Description | Links |
|----------|-------------|-------|
| cb_navigate_forward | Forward navigation to goal | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_navigate_forward.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_navigate_forward.hpp` |
| cb_navigate_backwards | Backward navigation | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_navigate_backwards.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_navigate_backwards.hpp` |
| cb_navigate_named_waypoint | Navigate to predefined waypoint | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_navigate_named_waypoint.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_navigate_named_waypoint.hpp` |
| cb_navigate_next_waypoint | Sequential waypoint navigation | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_navigate_next_waypoint.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_navigate_next_waypoint.hpp` |
| cb_abort_navigation | Cancel active navigation | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_abort_navigation.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_abort_navigation.hpp` |
| cb_move_end_effector | Move arm end-effector to pose | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_move_end_effector.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_move_end_effector.hpp` |
| cb_move_joints | Direct joint control | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_move_joints.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_move_joints.hpp` |
| cb_move_known_state | Move to predefined configuration | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_move_known_state.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_move_known_state.hpp` |
| cb_move_cartesian_relative2 | Relative Cartesian movement | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_move_cartesian_relative2.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_move_cartesian_relative2.hpp` |

---

## 2. Event-Driven Behaviors

**Purpose:** React to external stimuli (keyboard input, timer ticks, sensor messages).

**Base Class:** `SmaccClientBehavior` or `SmaccAsyncClientBehavior`

**Key Characteristics:**
- Uses `onStateOrthogonalAllocation<TOrthogonal, TSourceObject>()` for type-safe event posting
- Connects to component signals in `onEntry()`
- Event posting via stored lambda functions

**Best Reference:**
- [cb_default_keyboard_behavior.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_keyboard/include/cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp)
- `src/SMACC2/smacc2_client_library/cl_keyboard/include/cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp`

**Additional Examples:**

| Behavior | Description | Links |
|----------|-------------|-------|
| cb_timer_countdown_once | Single timer expiration | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp) / `src/SMACC2/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp` |
| cb_timer_countdown_loop | Recurring timer events | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/client_behaviors/cb_timer_countdown_loop.hpp) / `src/SMACC2/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/client_behaviors/cb_timer_countdown_loop.hpp` |
| cb_ros2_timer | Generic timer behavior | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/client_behaviors/cb_ros2_timer.hpp) / `src/SMACC2/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/client_behaviors/cb_ros2_timer.hpp` |
| cb_detect_apriltag | AprilTag detection events | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_isaac_apriltag/include/cl_isaac_apriltag/client_behaviors/cb_detect_apriltag.hpp) / `src/SMACC2/smacc2_client_library/cl_isaac_apriltag/include/cl_isaac_apriltag/client_behaviors/cb_detect_apriltag.hpp` |

---

## 3. Continuous Update Behaviors

**Purpose:** Persistent monitoring, tracking, or periodic calculations during state execution.

**Base Class:** `SmaccClientBehavior` + `ISmaccUpdatable` (multiple inheritance)

**Key Characteristics:**
- Inherits from both `SmaccClientBehavior` and `ISmaccUpdatable`
- Implements `update()` method called at ~20Hz by SignalDetector
- Enable/disable tracking in `onEntry()` / `onExit()`

**Best Reference:**
- [cb_track_object_pose.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_foundation_pose/include/cl_foundation_pose/client_behaviors/cb_track_object_pose.hpp)
- `src/SMACC2/smacc2_client_library/cl_foundation_pose/include/cl_foundation_pose/client_behaviors/cb_track_object_pose.hpp`

**Additional Examples:**

| Behavior | Description | Links |
|----------|-------------|-------|
| cb_track_path_odometry | Odometry-based path tracking | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_track_path_odometry.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_track_path_odometry.hpp` |
| cb_track_path_slam | SLAM-based path tracking | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_track_path_slam.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_track_path_slam.hpp` |
| cb_position_control_free_space | Continuous position control | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_position_control_free_space.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_position_control_free_space.hpp` |
| cb_battery_decission | Battery monitoring for mission control | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_mission_tracker/include/cl_mission_tracker/client_behaviors/cb_battery_decission.hpp) / `src/SMACC2/smacc2_client_library/cl_mission_tracker/include/cl_mission_tracker/client_behaviors/cb_battery_decission.hpp` |

---

## 4. Configuration/Lifecycle Behaviors

**Purpose:** System state management, ROS2 lifecycle transitions, waiting for readiness.

**Base Class:** `SmaccAsyncClientBehavior`

**Key Characteristics:**
- Connects to lifecycle transition signals
- Posts success/failure events on transition completion
- Often used in initialization states

**Best Reference:**
- [cb_activate.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/client_behaviors/cb_activate.hpp)
- `src/SMACC2/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/client_behaviors/cb_activate.hpp`

**Additional Examples:**

| Behavior | Description | Links |
|----------|-------------|-------|
| cb_configure | Configure lifecycle node | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/client_behaviors/cb_configure.hpp) / `src/SMACC2/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/client_behaviors/cb_configure.hpp` |
| cb_deactivate | Deactivate lifecycle node | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/client_behaviors/cb_deactivate.hpp) / `src/SMACC2/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/client_behaviors/cb_deactivate.hpp` |
| cb_cleanup | Cleanup lifecycle node | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/client_behaviors/cb_cleanup.hpp) / `src/SMACC2/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/client_behaviors/cb_cleanup.hpp` |
| cb_pause_slam | Pause SLAM operations | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_pause_slam.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_pause_slam.hpp` |
| cb_resume_slam | Resume SLAM operations | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_resume_slam.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_resume_slam.hpp` |
| cb_wait_nav2_nodes | Wait for Nav2 system readiness | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_wait_nav2_nodes.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_wait_nav2_nodes.hpp` |
| cb_wait_pose | Wait for pose availability | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_wait_pose.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_wait_pose.hpp` |
| cb_wait_transform | Wait for TF transforms | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_wait_transform.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_wait_transform.hpp` |

---

## 5. Motion Control Behaviors

**Purpose:** Direct robot movement control (rotation, spinning, specialized motion patterns).

**Base Class:** `SmaccAsyncClientBehavior` or client-specific base (e.g., `CbNav2ZClientBehaviorBase`)

**Key Characteristics:**
- Motion parameters as member variables (angles, distances, speeds)
- Optional planner/controller selection via `std::optional<>`
- TF buffer access for coordinate calculations

**Best Reference:**
- [cb_rotate.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_rotate.hpp)
- `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_rotate.hpp`

**Additional Examples:**

| Behavior | Description | Links |
|----------|-------------|-------|
| cb_absolute_rotate | Rotate to absolute heading | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_absolute_rotate.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_absolute_rotate.hpp` |
| cb_rotate_look_at | Rotate to face target | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_rotate_look_at.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_rotate_look_at.hpp` |
| cb_pure_spinning | Continuous spinning motion | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_pure_spinning.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_pure_spinning.hpp` |
| cb_spiral_motion | Spiral search patterns | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_spiral_motion.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_spiral_motion.hpp` |
| cb_undo_path_backwards | Reverse path following | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_undo_path_backwards.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_undo_path_backwards.hpp` |
| cb_active_stop | Emergency stop behavior | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_active_stop.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_active_stop.hpp` |
| cb_end_effector_rotate | End-effector rotation | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_end_effector_rotate.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_end_effector_rotate.hpp` |
| cb_circular_pivot_motion | Circular motion patterns | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_circular_pivot_motion.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_circular_pivot_motion.hpp` |

---

## 6. Communication Behaviors

**Purpose:** Data exchange via HTTP, object manipulation in planning scenes.

**Base Class:** `SmaccAsyncClientBehavior`

**Key Characteristics:**
- Deferred operation lambdas for template-safe execution
- Response callback virtual methods
- Request method configuration

**Best Reference:**
- [cb_http_get_request.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_http/include/cl_http/client_behaviors/cb_http_get_request.hpp)
- `src/SMACC2/smacc2_client_library/cl_http/include/cl_http/client_behaviors/cb_http_get_request.hpp`

**Additional Examples:**

| Behavior | Description | Links |
|----------|-------------|-------|
| cb_http_post_request | HTTP POST operations | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_http/include/cl_http/client_behaviors/cb_http_post_request.hpp) / `src/SMACC2/smacc2_client_library/cl_http/include/cl_http/client_behaviors/cb_http_post_request.hpp` |
| cb_http_request | Generic HTTP request base | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_http/include/cl_http/client_behaviors/cb_http_request.hpp) / `src/SMACC2/smacc2_client_library/cl_http/include/cl_http/client_behaviors/cb_http_request.hpp` |
| cb_attach_object | Attach objects in planning scene | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_attach_object.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_attach_object.hpp` |
| cb_detach_object | Detach objects from planning scene | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_detach_object.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_detach_object.hpp` |

---

## 7. Utility/Helper Behaviors

**Purpose:** Support functions, file loading, waypoint management, retry logic.

**Base Class:** `SmaccClientBehavior` or `SmaccAsyncClientBehavior`

**Key Characteristics:**
- Minimal state, often just configuration parameters
- Simple `onEntry()` operations
- Building blocks for complex behaviors

**Best Reference:**
- [cb_load_waypoints_file.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_load_waypoints_file.hpp)
- `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_load_waypoints_file.hpp`

**Additional Examples:**

| Behavior | Description | Links |
|----------|-------------|-------|
| cb_seek_waypoint | Search for specific waypoints | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_seek_waypoint.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_seek_waypoint.hpp` |
| cb_retry_behavior | Retry failed operations | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_retry_behavior.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_retry_behavior.hpp` |
| cb_undo_last_trajectory | Reverse previous motion | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_undo_last_trajectory.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/client_behaviors/cb_undo_last_trajectory.hpp` |
| cb_save_slam_map | Save current SLAM map | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_save_slam_map.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_save_slam_map.hpp` |
| cb_pause_object_tracking | Pause tracking operations | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_foundation_pose/include/cl_foundation_pose/client_behaviors/cb_pause_object_tracking.hpp) / `src/SMACC2/smacc2_client_library/cl_foundation_pose/include/cl_foundation_pose/client_behaviors/cb_pause_object_tracking.hpp` |


//////////////////////////////////////////////////////////////////////////////

# Component Patterns

## 1. Action Interface Components

**Purpose:** Wrap generic action clients with domain-specific interfaces.

**Key Characteristics:**
- Creates domain-specific signals from generic action client
- Provides high-level methods (e.g., `sendNavigationGoal()`)
- Uses `requiresComponent()` to access underlying `CpActionClient`

**Best Reference:**
- [cp_nav2_action_interface.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/nav2_action_interface/cp_nav2_action_interface.hpp)
- `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/nav2_action_interface/cp_nav2_action_interface.hpp`

---

## 2. Event Listener Components

**Purpose:** Connect to ROS topics/timers and emit signals for event posting.

**Key Characteristics:**
- Uses `onStateOrthogonalAllocation()` to set up event posting lambdas
- Connects to `CpTopicSubscriber` or `CpRos2Timer` in `onInitialize()`
- Emits signals that behaviors connect to

**Best Reference:**
- [cp_keyboard_listener_1.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_keyboard/include/cl_keyboard/components/cp_keyboard_listener_1.hpp)
- `src/SMACC2/smacc2_client_library/cl_keyboard/include/cl_keyboard/components/cp_keyboard_listener_1.hpp`

**Additional Examples:**

| Component | Description | Links |
|-----------|-------------|-------|
| cp_timer_listener_1 | Timer tick event generator | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/components/cp_timer_listener_1.hpp) / `src/SMACC2/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/components/cp_timer_listener_1.hpp` |
| cp_lifecycle_event_monitor | Lifecycle transition event monitoring | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/components/cp_lifecycle_event_monitor.hpp) / `src/SMACC2/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/components/cp_lifecycle_event_monitor.hpp` |

---

## 3. State Tracking Components

**Purpose:** Maintain internal state and provide state query interfaces.

**Key Characteristics:**
- Uses `enum class` for type-safe state representation
- Inline getter methods for state access
- Dedicated methods for state transitions

**Best Reference:**
- [cp_slam_toolbox.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/slam_toolbox/cp_slam_toolbox.hpp)
- `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/slam_toolbox/cp_slam_toolbox.hpp`

**Additional Examples:**

| Component | Description | Links |
|-----------|-------------|-------|
| cp_grasping_objects | Grasping object state and collision management | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/components/cp_grasping_objects.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/components/cp_grasping_objects.hpp` |
| cp_apriltag_mission_state | AprilTag mission state management | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_isaac_apriltag/include/cl_isaac_apriltag/components/cp_apriltag_mission_state.hpp) / `src/SMACC2/smacc2_client_library/cl_isaac_apriltag/include/cl_isaac_apriltag/components/cp_apriltag_mission_state.hpp` |
| cp_decision_manager | Decision counter and state management | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_mission_tracker/include/cl_mission_tracker/components/cp_decision_manager.hpp) / `src/SMACC2/smacc2_client_library/cl_mission_tracker/include/cl_mission_tracker/components/cp_decision_manager.hpp` |

---

## 4. Configuration Management Components

**Purpose:** Runtime parameter switching and configuration publishing.

**Key Characteristics:**
- Publisher-based configuration broadcasting
- Preset configuration methods (e.g., `setBackwardPlanner()`)
- Deferred execution via `commit` parameter

**Best Reference:**
- [cp_planner_switcher.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/planner_switcher/cp_planner_switcher.hpp)
- `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/planner_switcher/cp_planner_switcher.hpp`

**Additional Examples:**

| Component | Description | Links |
|-----------|-------------|-------|
| cp_goal_checker_switcher | Goal checker algorithm selection | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/goal_checker_switcher/cp_goal_checker_switcher.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/goal_checker_switcher/cp_goal_checker_switcher.hpp` |
| cp_costmap_switch | Costmap layer enable/disable | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/costmap_switch/cp_costmap_switch.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/costmap_switch/cp_costmap_switch.hpp` |

---

## 5. Data Buffer Components

**Purpose:** Store and retrieve sequential or historical data.

**Key Characteristics:**
- Vector-based storage with metadata structs
- Stack-like push/pop semantics
- Thread-safe operations with `std::mutex`
- Index-based retrieval with bounds checking

**Best Reference:**
- [cp_odom_tracker.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/odom_tracker/cp_odom_tracker.hpp)
- `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/odom_tracker/cp_odom_tracker.hpp`

**Additional Examples:**

| Component | Description | Links |
|-----------|-------------|-------|
| cp_trajectory_history | Trajectory execution history | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/components/cp_trajectory_history.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/components/cp_trajectory_history.hpp` |

---

## 6. Transform Management Components

**Purpose:** Handle spatial coordinate systems and pose tracking.

**Key Characteristics:**
- Static resource sharing for expensive TF resources
- `ISmaccUpdatable` for real-time pose updates
- Thread-safe access with mutex protection
- `std::optional<>` returns for potentially failing operations

**Best Reference:**
- [cp_pose.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/pose/cp_pose.hpp)
- `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/pose/cp_pose.hpp`

**Additional Examples:**

| Component | Description | Links |
|-----------|-------------|-------|
| cp_tf_listener | Transform listener wrapper | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/components/cp_tf_listener.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/components/cp_tf_listener.hpp` |
| cp_object_tracker_tf | TF-based object pose tracking | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_foundation_pose/include/cl_foundation_pose/components/cp_object_tracker_tf.hpp) / `src/SMACC2/smacc2_client_library/cl_foundation_pose/include/cl_foundation_pose/components/cp_object_tracker_tf.hpp` |
| cp_object_tracker_1 | Vision-based object detection tracking | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_foundation_pose/include/cl_foundation_pose/components/cp_object_tracker_1.hpp) / `src/SMACC2/smacc2_client_library/cl_foundation_pose/include/cl_foundation_pose/components/cp_object_tracker_1.hpp` |

---

## 7. Waypoint/Mission Control Components

**Purpose:** Orchestrate multi-waypoint mission execution.

**Key Characteristics:**
- Abstract base class with template method pattern
- Signal-based event coordination for waypoint completion
- Waypoint list management with rewind capability

**Best Reference:**
- [cp_waypoints_navigator.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/waypoints_navigator/cp_waypoints_navigator.hpp)
- `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/waypoints_navigator/cp_waypoints_navigator.hpp`

**Additional Examples:**

| Component | Description | Links |
|-----------|-------------|-------|
| cp_waypoints_navigator_base | Abstract waypoint navigation foundation | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/waypoints_navigator/cp_waypoints_navigator_base.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/waypoints_navigator/cp_waypoints_navigator_base.hpp` |
| cp_waypoints_event_dispatcher | Waypoint navigation event coordination | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/waypoints_navigator/cp_waypoints_event_dispatcher.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/waypoints_navigator/cp_waypoints_event_dispatcher.hpp` |
| cp_waypoints_visualizer | Waypoint visualization | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/waypoints_navigator/cp_waypoints_visualizer.hpp) / `src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/components/waypoints_navigator/cp_waypoints_visualizer.hpp` |

---

## 8. HTTP/Protocol Components

**Purpose:** Manage HTTP connections, sessions, and request execution.

**Key Characteristics:**
- Separation of concerns: connection, session, execution
- Component dependencies via `setDependencies()`
- Async request execution with result signals

**Best Reference:**
- [cp_http_request_executor.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_http/include/cl_http/components/cp_http_request_executor.hpp)
- `src/SMACC2/smacc2_client_library/cl_http/include/cl_http/components/cp_http_request_executor.hpp`

**Additional Examples:**

| Component | Description | Links |
|-----------|-------------|-------|
| cp_http_session_manager | HTTP session state management | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_http/include/cl_http/components/cp_http_session_manager.hpp) / `src/SMACC2/smacc2_client_library/cl_http/include/cl_http/components/cp_http_session_manager.hpp` |
| cp_http_connection_manager | HTTP connection lifecycle management | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_http/include/cl_http/components/cp_http_connection_manager.hpp) / `src/SMACC2/smacc2_client_library/cl_http/include/cl_http/components/cp_http_connection_manager.hpp` |

---

## 9. Motion Planning Components (cl_moveit2z)

**Purpose:** MoveIt2 motion planning and trajectory execution.

**Best Reference:**
- [cp_motion_planner.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/components/cp_motion_planner.hpp)
- `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/components/cp_motion_planner.hpp`

**Additional Examples:**

| Component | Description | Links |
|-----------|-------------|-------|
| cp_trajectory_executor | Trajectory execution controller | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/components/cp_trajectory_executor.hpp) / `src/SMACC2/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/components/cp_trajectory_executor.hpp` |

---

## 10. Perception/Tracking Components

**Purpose:** AprilTag and object detection/tracking.

**Best Reference:**
- [cp_apriltag_tracker.hpp](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_isaac_apriltag/include/cl_isaac_apriltag/components/cp_apriltag_tracker.hpp)
- `src/SMACC2/smacc2_client_library/cl_isaac_apriltag/include/cl_isaac_apriltag/components/cp_apriltag_tracker.hpp`

**Additional Examples:**

| Component | Description | Links |
|-----------|-------------|-------|
| cp_april_visualization | AprilTag visualization | [GitHub](https://github.com/robosoft-ai/SMACC2/blob/jazzy/smacc2_client_library/cl_isaac_apriltag/include/cl_isaac_apriltag/components/cp_april_visualization.hpp) / `src/SMACC2/smacc2_client_library/cl_isaac_apriltag/include/cl_isaac_apriltag/components/cp_april_visualization.hpp` |

//////////////////////////////////////////////////////////////////////////////

## Common Component Patterns

1. **Base Class:** All inherit from `smacc2::ISmaccComponent`
2. **Dependencies:** Use `requiresComponent()` to access other components
3. **Signals:** Use `smacc2::SmaccSignal<>` for decoupled communication
4. **Lifecycle:** Implement `onInitialize()` for setup
5. **Event Posting:** Use `onStateOrthogonalAllocation()` for templated events
6. **Thread Safety:** Use `std::mutex` for shared data
7. **Real-Time:** Inherit from `ISmaccUpdatable` for periodic updates


//////////////////////////////////////////////////////////////////////////////

# Common Architectural Patterns

These patterns appear consistently across all behavior and component types:

1. **Base Class Inheritance**
   - Behaviors: `SmaccClientBehavior` (sync) or `SmaccAsyncClientBehavior` (async)
   - Components: `ISmaccComponent`
   - For periodic updates: Add `ISmaccUpdatable` via multiple inheritance

2. **Template-Based Orthogonal Allocation**
   - `onStateOrthogonalAllocation<TOrthogonal, TSourceObject>()` captures type info
   - Enables type-safe event posting with correct orthogonal context

3. **Dependency Declaration**
   - `requiresClient(client_)` - get reference to owning client
   - `requiresComponent(component_)` - get reference to sibling component

4. **Signal-Callback Architecture**
   - Components expose `SmaccSignal<>` members
   - Behaviors connect via `signal.connect(&Class::method, this)`
   - Automatic lifecycle management prevents dangling callbacks

5. **Optional Configuration**
   - `std::optional<>` for flexible parameterization
   - Allows defaults while enabling explicit overrides

6. **Thread Safety**
   - `std::mutex` for shared data protection
   - Lock guards in accessor methods


//////////////////////////////////////////////////////////////////////////////

# Architecture Benefits

1. **Pure Separation of Concerns**
   - Clients orchestrate only - no business logic
   - Components implement all functionality
   - Behaviors consume components via `requiresComponent()`

2. **Composability**
   - Components can be mixed and matched
   - New clients compose existing components
   - Framework provides reusable core components

3. **Testability**
   - Each component can be tested independently
   - Clear interfaces between components

4. **Type Safety**
   - Template parameters ensure correct event typing
   - Compile-time orthogonal resolution

5. **Signal-Based Communication**
   - Loose coupling via SmaccSignal connections
   - Automatic lifecycle management


//////////////////////////////////////////////////////////////////////////////

# References

## Repo
https://github.com/robosoft-ai/SMACC2

## Usage Repo
https://github.com/robosoft-ai/nova_carter_sm_library

## Doxygen
https://robosoft-ai.github.io/smacc2_doxygen/jazzy/html/index.html

## Documentation
https://smacc2.robosoft.ai/
