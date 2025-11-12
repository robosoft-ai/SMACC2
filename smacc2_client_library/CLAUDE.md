
//////////////////////////////////////////////////////////////////////////////

# Code style guidelines

### Naming Conventions

- **Packages:** `snake_case` (e.g., `nav2z_client`)
- **Classes:** `PascalCase` with prefixes:
  - Clients: `Cl` prefix (e.g., `ClNav2Z`)
  - Client Behaviors: `Cb` prefix (e.g., `CbNavigateForward`)  
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
│   │   ├── cb_behavior_1.hpp
│   │   └── cb_behavior_2.hpp
│   └── components/
│       ├── cp_component_1.hpp
│       └── cp_component_2.hpp
├── src/my_client/
│   ├── cl_my_client.cpp
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

The SMACC2 Client Library provides modular, reusable clients for robot behaviors within the SMACC2 state machine framework. Each client encapsulates specific functionality and can be used across different state machines.

### Current Clients

| Client | Purpose | Communication Pattern |
|--------|---------|----------------------|
| `cl_nav2z` | Navigation with Nav2 | Action-based |
| `cl_moveitz` | Manipulation with MoveIt2 | Direct API calls |
| `cl_keyboard` | Keyboard input handling | Subscriber-based |
| `cl_ros2_timer` | Timer-based behaviors | Timer callbacks |
| `http_client` | HTTP requests | Custom protocol |
| `lifecyclenode_client` | ROS2 lifecycle management | Service calls |

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

### Inheritance Hierarchy

```cpp
// For general clients
class ClExample : public smacc2::ISmaccClient

// For synchronous client behaviors
class CbSyncBehavior1 : public smacc2::SmaccClientBehavior

// For asynchronous client behaviors
class CbAsyncBehavior1 : public smacc2::SmaccAsyncClientBehavior

// For components
class CpComponent1 : public smacc2::ISmaccComponent
```

//////////////////////////////////////////////////////////////////////////////

# Architecture

//////////////////////////////////////////////////////////////////////////////

## Known Client Types

### 1. ACTION-BASED CLIENTS

**Used for:** Long-running operations with feedback (navigation, manipulation)
**Features:**    
   - Action server integration
   - Goal/Result handling
   - Cancellation support
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

### 2. TOPIC-BASED CLIENTS

**Used for:** Publish/subscribe communication patterns
**Features:**  
   - Publisher/Subscriber pattern
   - Signal-based event communication
   - Thread-safe state management

Publishing is easy. Subscribing to topics, and throwing events based on that subscription is the main challenge.

#### Creating a subscriber client using the CpTopicSubscriber

**Example:** `keyboard_client` subscribing to key events

The key point of this approach is using the CpTopicSubscriber that essentially is a SMACC2 Component that wraps a ROS2 subscriber and provides a signal-based callback mechanism and also posts SMACC2 events.


```cpp
class ClKeyboard : public smacc2::ISmaccClient
{
public:
  ClKeyboard();
  virtual ~ClKeyboard();

  virtual void onInitialize() override;

  template <typename TOrthogonal>
  void onComponentInitialization()
  {
    // Create the subscriber component during orthogonal initialization
    subscriberComponent_ = this->createComponent<
      smacc2::components::CpTopicSubscriber<std_msgs::msg::UInt16>, TOrthogonal, ClKeyboard>(
      "/keyboard_unicode");
    subscriberComponent_->onMessageReceived(&ClKeyboard::onKeyboardMessage, this);

    // ...
  }

private:
  void onKeyPress(const std_msgs::msg::UInt16::SharedPtr msg);

  smacc2::components::CpTopicSubscriber<std_msgs::msg::UInt16> * subscriberComponent_;
};
```

**Examples:** 

- [ClAprilTagDetector/CpAprilVisualization](https://github.com/robosoft-ai/nova_carter_sm_library/blob/main/sm_nav2_test_7/include/sm_nav2_test_7/clients/cl_april_tag_detector/components/cp_april_visualization.hpp)

- [ClNav2Z/CpWaypointsVisualizer](https://github.com/robosoft-ai/SMACC2/blob/humble/smacc2_client_library/nav2z_client/nav2z_client/include/nav2z_client/components/waypoints_navigator/cp_waypoints_visualizer.hpp)


### 3. SERVICE-BASED CLIENTS

**Used for:** Request/response interactions
   - Synchronous/Asynchronous service calls
   - Response handling
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
### 4. TIMER-BASED CLIENTS

**Used for:** Periodic operations and delays
**Example:** `cl_ros2_timer` for timed behaviors

```cpp
class ClRos2Timer : public smacc2::ISmaccClient
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

### 5. API-BASED CLIENTS

**Used for:** Wrapping an API
**Example:** `cl_moveit2z` 

- [ClMoveit2Z](https://github.com/robosoft-ai/SMACC2/blob/humble/smacc2_client_library/cl_moveit2z/include/cl_moveit2z/cl_moveit2z.hpp)



//////////////////////////////////////////////////////////////////////////////

## Known Behavior Types
   
###   1. ACTION-BASED PATTERN BEHAVIORS (Goal-oriented with feedback)
 
#### Examples:
   cl_nav2z Client Behaviors:
  - cb_navigate_forward - Forward navigation with goal feedback
  - cb_navigate_backwards - Backward navigation with goal feedback
  - cb_navigate_global_position - Navigate to absolute position
  - cb_navigate_named_waypoint - Navigate to predefined waypoint
  - cb_navigate_next_waypoint - Sequential waypoint navigation
  - cb_navigate_next_waypoint_free - Free-form waypoint navigation
  - cb_navigate_next_waypoint_until_reached - Persistent waypoint pursuit
  - cb_abort_navigation - Cancel active navigation goal
  - cb_stop_navigation - Stop current navigation

  cl_moveit2z Client Behaviors:
  - cb_move_end_effector - Move robot arm end effector to pose
  - cb_move_cartesian_relative - Relative Cartesian movements
  - cb_move_cartesian_relative2 - Enhanced relative movements
  - cb_move_joints - Direct joint control
  - cb_move_joints_relative - Relative joint movements
  - cb_move_known_state - Move to predefined configuration
  - cb_move_named_target - Move to named pose targets
  - cb_execute_last_trajectory - Re-execute previous trajectory
  - cb_undo_last_trajectory - Reverse last movement

  Nova Carter Specific:
  - cb_spiral_motion - Execute spiral search patterns

#### Common C++ Structure:

```cpp
  class CbActionBehavior : public CbNav2ZClientBehaviorBase  // or SmaccAsyncClientBehavior
  {
  public:
    // Configuration options struct
    SomeOptions options;

    // Goal parameters (pose, distance, joints, etc.)
    geometry_msgs::msg::PoseStamped targetPose;  // or equivalent

    // Constructor with parameters
    CbActionBehavior(ParamType param);

    virtual void onEntry() override;  // Sends goal to action server
    virtual void onExit() override;   // Clean up

  protected:
    // Action client reference
    ClActionClient* actionClient_;

    // Result handling methods
    virtual void onActionResult(const Result&);
    virtual void onActionSuccess(const Result&);
    virtual void onActionAbort(const Result&);
  };
  ```

  Key Commonalities:
  - Inherit from SmaccAsyncClientBehavior or specialized base classes like CbNav2ZClientBehaviorBase
  <sup>file:///src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_nav2z_client_behavior_base.hpp#L28</sup>
  - Configuration options structs (e.g., CbNavigateForwardOptions
  <sup>file:///src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_navigate_forward.hpp#L33)</sup>

  - Goal parameter members (poses, distances, joint targets)
  - Action client pointers with typed results
  - Async result handling via callbacks


###  2. EVENT-DRIVEN PATTERN BEHAVIORS (Reactive to external stimuli)

#### Examples:

  Keyboard Client Behaviors:
  - cb_default_keyboard_behavior - React to keyboard input (A-Z keys)

  Sensor/Perception Client Behaviors:
  - cb_detect_apriltag - AprilTag detection events
  - cb_lidar_sensor - LiDAR data processing events
  - cb_forward_obstacle_detector - Obstacle detection events

  Timer Client Behaviors:
  - cb_timer_countdown_once - Single timer expiration event
  - cb_timer_countdown_loop - Recurring timer events
  - cb_ros_timer - Generic timer-based events

####  Common C++ Structure:
```cpp
  class CbEventBehavior : public smacc2::SmaccClientBehavior
  {
  public:
    template <typename TOrthogonal, typename TSourceObject>
    void onOrthogonalAllocation() {
      // Event posting lambda setup
      postEventFunction_ = [=]() {
        this->template postEvent<EvSomeEvent<TSourceObject, TOrthogonal>>();
      };
    }

    void onEntry() override {
      this->requiresClient(client_);
      client_->onSomeCallback(&CbEventBehavior::onCallbackMethod, this);
    }

  private:
    std::function<void()> postEventFunction_;  // Event posting mechanism
    SomeClient* client_;                       // Client for external data

    void onCallbackMethod(const CallbackData& data);  // Process external input
  };
  ```

  Key Commonalities:
  - Template-based onOrthogonalAllocation() for type-safe event posting
  <sup>file:///src/SMACC2/smacc2_client_library/cl_keyboard/include/cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp#L32</sup>
  - Event posting lambda functions stored as members
  <sup>file:///src/SMACC2/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/client_behaviors/cb_timer_countdown_loop.hpp#L49</sup>
  - Callback registration in onEntry()
  <sup>file:///src/nova_carter_sm_library/sm_nav2_test_7/include/sm_nav2_test_7/clients/cl_april_tag_detector/client_behaviors/cb_detect_apriltag.hpp#L47</sup>
  - Counter/state tracking variables (tick counters, detection flags)
  
  
###  3. CONTINUOUS UPDATE PATTERN BEHAVIORS (Persistent monitoring/tracking)

#### Examples:

  Tracking Behaviors:
  - cb_track_object_pose - Continuous object pose tracking
  - cb_track_path_odometry - Odometry-based path tracking
  - cb_track_path_slam - SLAM-based path tracking
  - cb_position_control_free_space - Continuous position control

  Sensor Monitoring:
  - cb_default_multirole_sensor_behavior - Multi-sensor data streaming
  - cb_battery_decision - Battery monitoring for mission control

#### Common C++ Structure:
 
 ```cpp
  class CbContinuousBehavior : public smacc2::SmaccClientBehavior,
                              public smacc2::ISmaccUpdatable
  {
  public:
    virtual void onEntry() override {
      requiresClient(client_);
      // or requiresComponent(component_, true);
      enableContinuousOperation();
    }

    virtual void onExit() override {
      disableContinuousOperation();
    }

    virtual void update() override {  // Called continuously by SMACC2 framework
      performContinuousTask();
    }

  private:
    SomeClient* client_;             // or Component* component_;
    ConfigParams params_;            // Tracking parameters
  };
```

  Key Commonalities:
  - Multiple inheritance: SmaccClientBehavior + ISmaccUpdatable
  - Required update() method for continuous operation
  <sup>file:///src/nova_carter_sm_library/sm_nav2_test_7/include/sm_nav2_test_7/clients/cl_foundationpose/client_behaviors/cb_track_object_pose.hpp#L64</sup>

  - Enable/disable state management in onEntry()/onExit()
  - Deferred operation functions (lambdas for templated operations)
  
###  4. CONFIGURATION/LIFECYCLE PATTERN BEHAVIORS (System state management)

#### Examples:

  Lifecycle Node Behaviors:
  - cb_activate - Activate lifecycle node
  - cb_deactivate - Deactivate lifecycle node
  - cb_configure - Configure lifecycle node
  - cb_cleanup - Clean up lifecycle node
  - cb_shutdown - Shutdown lifecycle node
  - cb_destroy - Destroy lifecycle node
  - cb_deactivate_on_exit - Auto-deactivate on state exit

  System Configuration:
  - cb_wait_nav2_nodes - Wait for Nav2 system readiness
  - cb_wait_pose - Wait for pose availability
  - cb_wait_transform - Wait for TF transforms
  - cb_pause_slam - Pause SLAM operations
  - cb_resume_slam - Resume SLAM operations
  - cb_save_slam_map - Save current SLAM map
  
 #### Common C++ Structure:
  
  ```cpp
  class CbLifecycleBehavior : public smacc2::SmaccAsyncClientBehavior
  {
  public:
    template <typename TOrthogonal, typename TSourceObject>
    void onOrthogonalAllocation() {
      SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
      this->requiresClient(lifecycleClient_);

      // Connect to lifecycle transition signals
      lifecycleClient_->onTransitionSuccess_.connect([this]() {
        this->postSuccessEvent();
      });
      lifecycleClient_->onTransitionFailure_.connect([this]() {
        this->postFailureEvent();
      });
    }

    virtual void onEntry() override {
      lifecycleClient_->performLifecycleTransition();
    }

  private:
    ClLifecycleNode* lifecycleClient_;
  };
  ```

  Key Commonalities:
  - Async behavior inheritance for non-blocking operations
  <sup>file:///src/SMACC2/smacc2_client_library/lifecyclenode_client/include/lifecyclenode_client/client_behaviors/cb_activate.hpp#L28</sup>
  - Signal-based result handling (success/failure events)
  <sup>file:///src/SMACC2/smacc2_client_library/lifecyclenode_client/include/lifecyclenode_client/client_behaviors/cb_activate.hpp#L41</sup>
  - Lifecycle client references
  - Standard postSuccessEvent()/postFailureEvent() methods
  

###  5. MOTION CONTROL PATTERN BEHAVIORS (Direct robot control)

####  Examples: 

  Rotation Behaviors:
  - cb_rotate - Basic rotation control
  - cb_absolute_rotate - Rotate to absolute heading
  - cb_rotate_look_at - Rotate to face target
  - cb_pure_spinning - Continuous spinning motion
  - cb_end_effector_rotate - Manipulator end-effector rotation

  Complex Motion:
  - cb_circular_pivot_motion - Circular motion patterns
  - cb_pouring_motion - Specialized pouring movements
  - cb_undo_path_backwards - Reverse path following
  - cb_active_stop - Emergency stop behavior
  
  
####  Common C++ Structure:
  
  ```cpp
  class CbMotionBehavior : public CbNav2ZClientBehaviorBase  // or SmaccAsyncClientBehavior
  {
  public:
    // Motion parameters
    float motionParameter_;                    // angle, distance, etc.
    std::optional<std::string> goalChecker_;  // Navigation goal checker
    std::optional<PlannerType> plannerType_;  // Motion planner selection

    CbMotionBehavior(float param, PlannerType planner = default);

    void onEntry() override;  // Execute motion command

  private:
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;  // Transform data
    // or MoveitClient* moveitClient_;
  };
```

  Key Commonalities:
  - Motion parameter members (angles, distances, speeds)
  <sup>file:///src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_rotate.hpp#L31</sup>
  - Optional planner/controller selection <sup>file:///src/SMACC2/smacc2_client_library/cl_nav2z/include/cl_nav2z/client_behaviors/cb_rotate.hpp#L35</sup>

  - Transform buffer access for coordinate calculations
  - Goal checker configuration options
  - Immediate motion execution in onEntry()


###   6. COMMUNICATION PATTERN BEHAVIORS (Data exchange)

#### Examples:
  
  HTTP Client Behaviors:
  - cb_http_get_request - HTTP GET operations
  - cb_http_post_request - HTTP POST operations
  - cb_http_request - Generic HTTP requests

  Publisher Client Behaviors:
  - cb_default_publish_loop - Continuous message publishing
  - cb_publish_once - Single message publishing
  - cb_muted_behavior - Silent/disabled publishing

  Object Interaction:
  - cb_attach_object - Attach objects in planning scene
  - cb_detach_object - Detach objects from planning scene
    
 
 #### Common C++ Structure:
  
  ```cpp
  class CbCommunicationBehavior : public smacc2::SmaccClientBehavior
  {
  public:
    template <typename TMessage>
    CbCommunicationBehavior(const TMessage& data) {
      setMessage(data);
    }

    template <typename TMessage>
    void setMessage(const TMessage& data) {
      deferredOperation_ = [this, data]() {
        this->client_->performOperation(data);
      };
    }

    void onEntry() override {
      this->requiresClient(client_);
      if (deferredOperation_) deferredOperation_();
    }

    virtual void onResponseReceived(const Response& response) {}

  private:
    std::function<void()> deferredOperation_;  // Template-safe operation storage
    CommunicationClient* client_;
  };
```

  Key Commonalities:
  - Template-based message handling
  - Deferred operation lambdas for type erasure
  - Response callback virtual methods <sup>file:///src/SMACC2/smacc2_client_library/http_client/include/http_client/client_behaviors/cb_http_request.hpp#L49</sup>

  - Communication client references
  - Request method enumeration/configuration

###  7. UTILITY/HELPER PATTERN BEHAVIORS (Support functions)

#### Examples: 

  File/Data Management:
  - cb_load_waypoints_file - Load waypoint configurations
  - cb_seek_waypoint - Search for specific waypoints
  - cb_pause_object_tracking - Pause tracking operations
  - cb_retry_behavior - Retry failed operations

  Trajectory Management:
  - cb_move_last_trajectory_initial_state - Reset to trajectory start
  - cb_move_end_effector_trajectory - Execute complex trajectories
  
#### Key C++ Commonalities:

  - Inherit from appropriate base behavior classes
  - Minimal state - often just configuration parameters
  - Simple onEntry() operations (file loading, state reset, retry logic)
  - Optional result/status reporting
  - Often used as building blocks for complex behaviors
   

### Common Architectural Patterns Across All Types:

  1. Base Classes: Most behaviors inherit from either SmaccClientBehavior(synchronous) or SmaccAsyncClientBehavior (asynchronous)
  2. Template-based Orthogonal Allocation: All behaviors use onOrthogonalAllocation<TOrthogonal, TSourceObject>() for type-safe integration
  3. Client/Component Dependencies: Use requiresClient() or requiresComponent() pattern
  4. Standard Lifecycle: onEntry()/onExit() methods for state management
  5. Event Integration: Type-safe event posting via templates
  6. Signal/Callback Architecture: Boost.signals2 or lambda-based callback systems
  7. Optional Configuration: Extensive use of std::optional<> for flexible parameterization


//////////////////////////////////////////////////////////////////////////////

## Known Component Types

###  1. PUBLISHER-SUBSCRIBER PATTERN COMPONENTS

  Navigation Publishers

  - CpAmcl (cl_nav2z)
    - AMCL initial pose publisher
    - Features: Initial pose setting for localization
  - CpWaypointsVisualizer (cl_nav2z)
    - Waypoint visualization publisher
    - Features: RViz marker publishing for waypoint display

  Perception Publishers

  - CpAprilTagVisualization (nova_carter/cl_april_tag_detector)
    - AprilTag detection visualization publisher
    - Features: MarkerArray publishing, aggregated tag position calculation, real-time marker updates
    - Update Pattern: Implements ISmaccUpdatable
    - Thread Safety: Mutex-protected transform data

####  Core C++ Design Patterns of Publisher-Subscriber Components: 
  
  Core Communication Infrastructure

  - CpTopicSubscriber<MessageType> (smacc2/core)
    - Generic ROS topic subscription with event generation
    - Features: Configurable QoS, first message tracking, signal-slot integration
  - CpTopicPublisher<MessageType> (smacc2/core)
    - ROS topic publishing infrastructure
    - Features: Template-based message publishing, periodic publishing support


###  2. STATE TRACKING PATTERN COMPONENTS

#####  Navigation State Trackers

  - CpSlamToolbox (cl_nav2z)
    - SLAM toolbox state tracking (Resumed/Paused)
    - Features: Blind state tracking, toggle operations
    - Pattern: Internal state enum with getter methods

#####  Manipulation State Trackers

  - CpGraspingComponent (cl_moveit2z)
    - Object manipulation state and collision object management
    - Features: Attached object tracking, gripper state, finger tip management
    - State: Current attached object name, collision object database
	
####  Core C++ Design Patterns of State Tracking Components:

#####  Enumeration State Management:
  <sup>cl_nav2z/include/cl_nav2z/components/slam_toolbox/cp_slam_toolbox.hpp:37-43</sup>
  
  ```cpp
  class CpSlamToolbox : public smacc2::ISmaccComponent
  {
  public:
    enum class SlamToolboxState { Resumed, Paused };

    inline SlamToolboxState getState() { return state_; }
    void toggleState();  // State transition logic

  private:
    SlamToolboxState state_;
  };
 ```
  <sup>cl_nav2z/include/cl_nav2z/components/slam_toolbox/cp_slam_toolbox.hpp:37-43</sup>

##### Database-Style State Tracking:
   ```cpp
  class CpGraspingComponent : public smacc2::ISmaccComponent
  {
  private:
    std::map<std::string, moveit_msgs::msg::CollisionObject> graspingObjects;

  public:
    std::optional<std::string> currentAttachedObjectName;
    bool getGraspingObject(std::string name, ObjectType& object);
  };
```
  <sup> cl_moveit2z/include/cl_moveit2z/components/cp_grasping_objects.hpp:30-41</sup>

#####  Common C++ Patterns:
  - Strongly Typed Enums: enum class for type-safe state representation
  - State Query Interface: Inline getter methods for state access
  - State Transition Methods: Dedicated methods for state changes (toggleState())
  - Map-Based Databases: std::map<std::string, ObjectType> for object tracking
  - Optional State Fields: std::optional<> for nullable state information
  - RAII State Initialization: Constructor-based state initialization
  - Const-Correct Getters: const methods for read-only state access

###  3. OBJECT TRACKING COMPONENTS

  - CpObjectTracker1 (nova_carter/cl_foundationpose)
    - 3D object detection state management
    - Features: Detection database with ID-based tracking, pose extraction
    - Data Structure: Map of detected objects with vision_msgs integration
    - Event Generation: EvObjectDetected on new detections

###  4. CONFIGURATION MANAGEMENT PATTERN COMPONENTS

#####  Navigation Configuration Managers

  - CpPlannerSwitcher (cl_nav2z)
    - Navigation2 planner/controller runtime switching
    - Features: Global planner switching, local controller switching, goal checker selection
    - Publishers: planner_selector, controller_selector, goal_checker_selector
  - CpGoalCheckerSwitcher (cl_nav2z)
    - Navigation goal verification switching
    - Features: Runtime goal checker algorithm selection
    - Publisher: goal_checker_selector topic
  - CpCostmapSwitch (cl_nav2z)
    - Costmap layer enable/disable control
    - Includes: CpCostmapProxy helper class for dynamic reconfigure
	
#### Core C++ Design Patterns of Configuration Management Components:

#####  Publisher-Based Configuration Broadcasting:

  ```cpp
  class CpPlannerSwitcher : public smacc2::ISmaccComponent
  {
  private:
    std::string desired_planner_;
    std::string desired_controller_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_selector_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_selector_pub_;

  public:
    void setDesiredGlobalPlanner(std::string plannerName);
    void commitPublish();  // Deferred execution pattern
  };
 ```
  <sup>cl_nav2z/include/cl_nav2z/components/planner_switcher/cp_planner_switcher.hpp:57-67</sup>

##### Preset Configuration Pattern:
 
   ```cpp
  void CpPlannerSwitcher::setBackwardPlanner(bool commit = true)
  {
    desired_planner_ = "BackwardGlobalPlanner";
    desired_controller_ = "BackwardLocalPlanner";

    if (commit) commitPublish();
  }
  ```
   <sup>cl_nav2z/src/cl_nav2z/components/planner_switcher/cp_planner_switcher.cpp:60-68</sup>
   
 ##### Common C++ Patterns:
  - Deferred Execution: commit parameter for batched configuration updates (TODO)

###  5. DATA BUFFER PATTERN COMPONENTS

  Motion History Buffers

  - CpOdomTracker (cl_nav2z)
    - Comprehensive odometry and path tracking system
    - Features: Path stack management, multi-mode operation (RECORD/CLEAR/IDLE)
    - Data Structures: Path stack, aggregated stack path, goal tracking
    - Thread Safety: Full mutex protection
  - CpTrajectoryHistory (cl_moveit2z)
    - MoveIt2 trajectory execution history
    - Features: Named trajectory storage, execution result tracking
    - Data Structure: Vector of TrajectoryHistoryEntry with metadata
    - Retrieval: By index, last trajectory access

  Sensor Data Buffers

  - CpForwardObstacleDetector (nova_carter/cl_lidar)
    - LiDAR-based forward obstacle distance tracking
    - Features: Security distance calculation, ray filtering, real-time distance updates
    - Data Storage: Latest scan message, selected forward distance
    - Safety: Configurable security distance threshold (0.4m default)

#### Core C++ Design Patterns of Data Buffer Components:

##### Vector-Based Historical Storage:

  ```cpp
  class CpTrajectoryHistory : public smacc2::ISmaccComponent
  {
  private:
    std::vector<TrajectoryHistoryEntry> trajectoryHistory_;

  public:
    bool getLastTrajectory(int backIndex, TrajectoryType& trajectory);
    void pushTrajectory(std::string name, const TrajectoryType& trajectory, ResultType result);
  };

  struct TrajectoryHistoryEntry 
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::MoveItErrorCodes result;
    std::string name;
  };
  ```
  <sup>Reference: cl_moveit2z/include/cl_moveit2z/components/cp_trajectory_history.hpp:30-49</sup>

##### Complex Multi-Mode Buffer Management:
  ```cpp
  class CpOdomTracker : public smacc2::ISmaccComponent
  {
  private:
    std::vector<nav_msgs::msg::Path> pathStack_;
    std::vector<PathInfo> pathInfos_;
    std::mutex m_mutex_;

  public:
    void pushPath(std::string pathname);
    void popPath(int pathCount = 1, bool keepPreviousPath = false);
  };
  ```
  <sup>cl_nav2z/include/cl_nav2z/components/odom_tracker/cp_odom_tracker.hpp:175-194</sup>

#####  Common C++ Patterns:
  - Vector-Based Storage: std::vector<> for sequential data storage
  - Struct-Based Metadata: Custom structs combining data with metadata
  - Index-Based Access: Negative indexing from end (size() - 1 - backIndex)
  - Thread-Safe Operations: std::mutex protection for concurrent access
  - RAII Resource Management: Automatic memory management via containers
  - Stack-Like Operations: Push/pop semantics for buffer management
  - Bounds Checking: Explicit size validation before access
  - Named Storage: String-based identification for stored elements
  
###  6. TRANSFORM MANAGEMENT PATTERN COMPONENTS

#####  Spatial Coordinate Systems

  - Pose (cl_nav2z)
    - Real-time pose tracking and transform management
    - Features: TF2 integration, frame conversion, pose freezing
    - Update Pattern: Implements ISmaccUpdatable
    - Thread Safety: Mutex-protected pose data
    - Reference Frames: Configurable (map, odom, base_link)
  - CpTfListener (cl_moveit2z)
    - Transform listener wrapper (referenced but minimal implementation)
    - Features: TF tree monitoring, transform buffering

 ##### Advanced Transform Management

  - CpObjectTrackerTf (nova_carter/cl_foundationpose)
    - TF-based object pose tracking with temporal filtering
    - Features:
        - Multi-object tracking via TF frames
      - Historical pose buffering (512 sample max)
      - Median filtering for pose stabilization
      - Enable/disable state management
      - Reset capabilities
    - Update Pattern: Implements ISmaccUpdatable
    - Thread Safety: Full mutex protection
    - Event Generation: EvObjectDetected on successful tracking
	
####  Core C++ Design Patterns of Transform Management Components:

#####  Static Resource Sharing:
 
  ```cpp
  class Pose : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
  {
  private:
    static std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    static std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    static std::mutex listenerMutex_;

    geometry_msgs::msg::PoseStamped pose_;
    std::mutex m_mutex_;
  };
 ```
  <sup>cl_nav2z/include/cl_nav2z/components/pose/cp_pose.hpp:96-104</sup>
 
#####  Real-Time Update Pattern:
  
   ```cpp
  class CpObjectTrackerTf : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
  {
  private:
    std::map<std::string, DetectedObject> detectedObjects;
    std::mutex m_mutex_;

  public:
    void update() override;  // Real-time processing
    std::optional<geometry_msgs::msg::PoseStamped> updateAndGetGlobalPose(const std::string& frame_id);
  };
```
<sup>nova_carter_sm_library/sm_nav2_test_7/include/sm_nav2_test_7/clients/cl_foundationpose/components/cp_object_tracker_tf.hpp:31-44</sup>

#####  Common C++ Patterns:
  - Static Resource Sharing: Singleton-like pattern for expensive TF resources
  - Multiple Inheritance: ISmaccComponent + ISmaccUpdatable for real-time components
  - Thread-Safe Access: Mutex protection for concurrent transform access
  - Optional Return Types: std::optional<> for potentially failing operations
  - Template Method Pattern: update() override for real-time processing
  - RAII TF Management: Automatic TF buffer/listener lifecycle management
  - Temporal Filtering: Historical data for stability (median filtering)
  - Frame-Based Indexing: String-based frame identification for transform lookups

###  7. WAYPOINT (MISSION) CONTROL PATTERN COMPONENTS

  Waypoint Navigation Orchestration

  - CpWaypointNavigatorBase (nav2z_client)
    - Abstract waypoint navigation foundation
    - Features: Base class for waypoint management systems
    - Pattern: Template method pattern for waypoint operations
  - CpWaypointNavigator (nav2z_client)
    - Sequential waypoint navigation controller
    - Features: Waypoint list management, goal sending with options
    - State Management: Success-based progression
    - Integration: Navigation2 action client integration
  - CpWaypointsEventDispatcher (nav2z_client)
    - Waypoint navigation event coordination
    - Features: Event routing for waypoint-based state machines
    - Pattern: Event dispatcher for multi-waypoint missions

#### Core C++ Design Patterns of Waypoint (Mission) Control Components:

##### Abstract Base Template Method:

```cpp
  class CpWaypointNavigatorBase : public smacc2::ISmaccComponent
  {
  protected:
    int currentWaypoint_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;

  public:
    virtual void onInitialize() = 0;  // Template method pattern
    void rewind(int count);  // Common behavior
  };
```
  <sup>cl_nav2z/include/cl_nav2z/components/waypoints_navigator/cp_waypoints_navigator_base.hpp</sup>

##### Signal-Based Event Coordination:
 
  ```cpp
  class CpWaypointNavigator : public CpWaypointNavigatorBase
  {
  public:
    smacc2::SmaccSignal<void()> onNavigationRequestSucceded;
    smacc2::SmaccSignal<void()> onNavigationRequestAborted;
    smacc2::SmaccSignal<void()> onNavigationRequestCancelled;

  private:
    void onGoalReached(const ClNav2Z::WrappedResult & res);
    void onGoalCancelled(const ClNav2Z::WrappedResult & /*res*/);
    void onGoalAborted(const ClNav2Z::WrappedResult & /*res*/);

    boost::signals2::connection succeddedNav2ZClientConnection_;
    boost::signals2::connection abortedNav2ZClientConnection_;
    boost::signals2::connection cancelledNav2ZClientConnection_;
  };
```
 <sup>cl_nav2z/include/cl_nav2z/components/waypoints_navigator/cp_waypoints_navigator.hpp:70-83</sup>


 ##### Common C++ Patterns:
  - Template Method Pattern: Abstract base with concrete implementations
  - Signal Aggregation: Multiple SmaccSignal<> for different event types
  - Connection Management: boost::signals2::connection for callback lifecycle
  - Index-Based State: Integer waypoint progression tracking
  - Event Dispatcher Pattern: Separate dispatcher for event routing
  - Polymorphic Callbacks: Virtual methods for customizable behavior
  - State Progression Logic: Automatic increment on successful completion
  - RAII Connection Management: Automatic signal disconnection on destruction
  
  ### Common C++ Design Patterns Across All Component Types

  ##### Universal Patterns:

  1. Base Class Hierarchy: All inherit from smacc2::ISmaccComponent
  2. Lifecycle Management: Recommended onInitialize() override pattern
  3. RAII Resource Management: Automatic cleanup via destructors and smart pointers
  4. Thread Safety: std::mutex protection for shared data
  5. Optional Configuration: std::optional<> for configurable parameters
  6. Signal-Slot Communication: smacc2::SmaccSignal<> for decoupled messaging
  7. String-Based Identification: Consistent naming and identification patterns
  8. Template-Based Generics: Type-safe generic programming where applicable

  ##### Specialized Patterns by Category:

  - Publisher-Subscriber: Template specialization, QoS configuration
  - State Tracking: Enum classes, map-based databases
  - Configuration: Builder pattern, deferred execution
  - Data Buffers: Vector storage, index-based access, bounds checking
  - Transform Management: Static resource sharing, real-time updates
  - Mission Control: Abstract base classes, event aggregation
  ---


//////////////////////////////////////////////////////////////////////////////

# References

## Repo
https://github.com/robosoft-ai/SMACC2

## Usage Repo
https://github.com/robosoft-ai/nova_carter_sm_library

## Doxygen
https://robosoft-ai.github.io/smacc2_doxygen/humble/html/index.html

## Documentation
https://smacc2.robosoft.ai/
