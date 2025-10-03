//////////////////////////////////////////////////////////////////////////////

# SMACC2 Object Lifetimes
SMACC2 runtime objects can be categorized according to object lifetimes. 
State machine-scoped objects persist throughout the entire state machine
execution, while state-scoped objects are dynamically created and destroyed with
state transitions.
  
  ### State Scoped Objects (Lifetime tied to individual states)
  - States (St)
  - Client Behaviors (Cb)
  - State Reactors (Sr)
  - Event Generators (Eg)

  ### State Machine Scoped Objects (Lifetime tied to the State Machine)
  - State Machines (Sm) 
  - Orthogonals (Or)
  - Clients (Cl)
  - Components (Cp)

//////////////////////////////////////////////////////////////////////////////

# SMACC Signals
SmaccSignal is a communication mechanism (template wrapper around boost::signals2).

  ### Runtime architecture
  The actual runtime object is the SignalDetector (State Machine Scoped)
  - No prefix - singleton per state machine
  - Lifetime: State machine lifetime
  - Purpose: Polls and processes all signals in the system
  - Key responsibilities:
    - Manages signal processing loop
    - Finds and updates ISmaccUpdatable objects
    - Coordinates signal connections/disconnections
    - Handles execution model (single-threaded vs multi-threaded)

  ### Signal Connection Lifetime Management
  Signal connections are managed through the state machine's
  createSignalConnection() which:
  1. Creates boost::signals2 connections between signal sources and
  callbacks
  2. Tracks connections by object pointer
  3. Automatically disconnects when objects are destroyed via
  disconnectSmaccSignalObject()
  4. Ensures proper cleanup when states exit (disconnecting state-scoped
  object signals)

  ### Important Core Files
  - smacc2/include/smacc2/common.hpp
  - smacc2/include/smacc2/smacc_signal.hpp
  - smacc2/include/smacc2/smacc_state_machine.hpp
  - smacc2/include/smacc2/impl/smacc_state_machine_impl.hpp
  - smacc2/include/smacc2/smacc_client.hpp
  - smacc2/include/smacc2/impl/smacc_client_impl.hpp
  - smacc2/include/smacc2/callback_counter_semaphore.hpp
  - smacc2/src/smacc2/orthogonal.cpp

  ### Important Client Library Related Files
  - smacc2/include/smacc2/smacc_asynchronous_client_behavior.hpp
  - smacc2/include/smacc2/client_core_components/cp_topic_subscriber.hpp
  - smacc2/include/smacc2/impl/smacc_asynchronous_client_behavior_impl.hpp
  - smacc2/include/smacc2/client_bases/smacc_action_client_base.hpp
  - smacc2/include/smacc2/client_bases/smacc_service_server_client.hpp

  ### Important Classes
  - SmaccSignal

  ### Usage in SMACC Client Libraries
  - Clients have signals: onSucceeded_, onAborted_, onMessageReceived_
  - Components have signals: onMessageTimeout_, onResponseReceived_
  - ClientBehaviors have signals: onSuccess_, onFailure_, onFinished_

  ### Example usage in Clients:
  - smacc2_client_library/lifecyclenode_client/include/lifecyclenode_client/lifecyclenode_client.hpp
  - smacc2_client_library/ros_timer_client/include/ros_timer_client/cl_ros_timer.hpp
  - smacc2_client_library/moveit2z_client/include/moveit2z_client/cl_moveit2z.hpp
  - smacc2_client_library/nav2z_client/nav2z_client/include/nav2z_client/nav2z_client.hpp
  - smacc2_client_library/http_client/include/http_client/cl_http_client.hpp
  - smacc2_client_library/keyboard_client/include/keyboard_client/cl_keyboard.hpp

  ### Example usage in Client Behaviors:
  - smacc2_client_library/ros_timer_client/include/ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp

  ### Example usage in Components:
  - smacc2_client_library/nav2z_client/nav2z_client/include/nav2z_client/components/waypoints_navigator/cp_waypoints_navigator.hpp

  ### Boost Signals2 Documentation
  https://www.boost.org/doc/libs/1_89_0/doc/html/signals2.html

  ### Boost Signals2 Source Code
  https://github.com/boostorg/signals2

  ## Problems related to incorrectly using boost signal raw instead of the SmaccSignal approach.
  After analyzing the SMACC signal architecture, using raw boost::signals2
  would cause catastrophic failures:

  ### Memory Leaks from Orphaned Connections
  Client behaviors create signal connections in onEntry() but without disconnectSmaccSignalObject() being called automatically at state exit, these connections would persist indefinitely. 
  Each state transition would accumulate more dead connections, eventually exhausting memory.

  ### Segmentation Faults from Dangling Callbacks
  Most critical for client behaviors: When a CbNavigateForward subscribes to ClNav2Z::onSucceeded_ signal using raw boost::signals2, the connection would outlive the behavior. 
  When navigation completes after the state has exited, the signal would invoke a callback on a destroyed behavior object → immediate segfault.

  ### State Machine Corruption
  Without EventLifeTime::CURRENT_STATE enforcement, client behaviors could post events after their state exits. 
  Example: An async CbTimer using raw signals could fire EvTimer<TsFinished> into the wrong state, triggering invalid transitions and breaking the entire state flow.

  ### Race Conditions in Multi-threaded Execution
  The SignalDetector synchronizes signal processing through its polling loop and scheduler. 
  Raw boost::signals2 connections would fire callbacks directly from arbitrary threads (ROS executor threads, timer threads), causing:
  - Concurrent state modifications
  - Non-deterministic event ordering
  - Data races in state machine internals

  ### Invisible Signal Flows
  createSignalConnection() registers connections with the state machine for introspection. 
  Raw connections would be invisible to:
  - State machine visualization tools
  - Runtime debugging
  - Signal flow analysis
  - Connection leak detection

  ### Async Client Behavior Breakdown
  SmaccAsyncClientBehavior relies on signals (onSuccess_, onFailure_, onFinished_) managed by the framework. 
  Raw signal usage would:
  - Break the async behavior protocol
  - Prevent proper requestForceFinish() handling
  - Cause behaviors to "hang" indefinitely
  - Make waitOnEntryThread_ synchronization fail

  ### Connection Lifecycle Chaos
  Without centralized management through signalConnectionsManager_, developers would need to manually track every connection and ensure disconnection at the exact right moment - an error-prone nightmare in complex state machines with dozens of concurrent behaviors.

  The SmaccSignal approach isn't just convenience - it's essential for correctness. The automatic lifecycle management prevents an entire class of runtime failures that would be nearly impossible to debug in production.


//////////////////////////////////////////////////////////////////////////////

# SMACC2 Threading & Asynchronous I/O
SMACC2's threading architectureuses a sophisticated hybrid concurrency model that solves a fundamental robotics challenge: maintaining deterministic state machine behavior while handling inherently asynchronous real-world I/O.

SMACC2 uses what can be referred to as event-queue-mediated concurrency: async operations run in separate threads but communicate exclusively through a synchronized event queue, never directly touching state machine internals.

  ## Thread Hierarchy & Roles

  ### 1. State Machine Main Thread (Deterministic Core)
  - Executes all state logic, transitions, and onEntry()/onExit() calls
  - Only thread allowed to modify state machine state
  - Processes events sequentially from SmaccFifoScheduler queue
  - Guaranteed deterministic behavior regardless of event timing

  ### 2. SignalDetector Coordination Thread
  - Bridges async world to synchronous state machine
  - Runs at configurable frequency (default 20Hz) via pollingLoop()
  - Critical responsibilities:
    - Calls executeUpdate() on all ISmaccUpdatable objects
    - Processes ROS2 callbacks via rclcpp::spin_some()
    - Manages updatable object lifecycle (finds new ones, cleans up
  destroyed ones)
    - Enforces state transition barriers (no updates during transitions)

  ### 3. Async Behavior Worker Threads
  - Spawned via std::async(std::launch::async) for each
  SmaccAsyncClientBehavior
  - Run completely independent operations (navigation, manipulation, long
  computations)
  - Controlled termination: requestForceFinish_ flag monitored by behavior
  implementations
  - Lifecycle synchronization: State exit blocks until all async behaviors
  complete via waitOnEntryThread()

  ### 4. ROS2 Executor Threads
  - Handle ROS2 subscriptions, timers, service calls
  - Never directly modify state machine - only post events through signal
  connections
  - Two modes: SINGLE_THREAD_SPINNER or MULTI_THREAD_SPINNER for callback
  concurrency

  ## Async I/O Patterns & Use Cases

  ### Pattern 1: Long-Running Operations (SmaccAsyncClientBehavior)
  ```cpp
  // Navigation behavior runs in separate thread
  class CbNavigateForward : public SmaccAsyncClientBehavior {
    void onEntry() override {
      // This runs in dedicated thread, doesn't block state machine
      navigationClient_->sendGoal(target);
      // Behavior posts success/failure events when complete
    }
  };
  ```
  - Use case: Navigation, manipulation, file I/O, network requests
  - Lifecycle: Thread spawned on state entry, must complete before state
  exit
  - Communication: Posts EvCbSuccess/EvCbFailure events to queue

  ### Pattern 2: Periodic Monitoring (ISmaccUpdatable)
  ```cpp
  // Component that monitors sensor data
  class CpLidarMonitor : public ISmaccComponent, public ISmaccUpdatable {
    void update() override {
      // Called every 50ms by SignalDetector
      if (checkObstacleDetected()) {
        postEvent<EvObstacleDetected>();
      }
    }
  };
  ```
  - Use case: Sensor monitoring, watchdogs, periodic calculations
  - Lifecycle: Updated while component/behavior is active
  - Frequency: Controlled by SignalDetector loop rate

  ### Pattern 3: Event-Driven I/O (ROS2 Callbacks)
  ```cpp
  // Timer fires in ROS executor thread
  timer_->registerCallback([this]() {
    // This runs in ROS thread, posts event to state machine
    onTimerTick_.emit();  // Signal connection handles thread-safe event 
  posting
  });
  ```
  - Use case: ROS2 subscriptions, timers, service responses
  - Lifecycle: Managed by ROS2 executor
  - Thread safety: Signals automatically queue events to main thread

 ## Synchronization Mechanisms

  ### 1. Event Queue as Synchronization Primitive
  - All async→sync communication flows through SmaccFifoScheduler event
  queue
  - Thread-safe event posting from any thread
  - Sequential processing in main thread ensures determinism

  ### 2. State Transition Barriers
  - Updates suspended during STATE_CONFIGURING, STATE_ENTERING,
  STATE_EXITING
  - Prevents race conditions during state machine topology changes
  - Async behaviors must complete before state exit proceeds

  ### 3. Automatic Resource Management
  - Signal connections track object lifetimes
  - disconnectSmaccSignalObject() prevents callbacks to destroyed objects
  - State-scoped objects automatically cleaned up on state exit

  ## Design Strengths

  - Determinism Without Blocking: State machine logic remains predictable
  while I/O operations run concurrently.

  - Deadlock Prevention: Multiple escape mechanisms (requestForceFinish_,
  timeouts, force disconnections) prevent system hangs.

  - Memory Safety: Automatic lifecycle management prevents dangling pointers
  and callback-after-destruction bugs.

  - Flexible Concurrency: Three distinct patterns handle different async I/O
  requirements without forcing a one-size-fits-all solution.

  - Debugging Simplicity: Single-threaded state logic is much easier to debug
   than fully concurrent state machines.

  The architecture achieves controlled chaos - allowing the inherently
  chaotic async world to coexist with the deterministic requirements of
  state machine logic through careful interface boundaries and
  synchronization primitives.


//////////////////////////////////////////////////////////////////////////////

# Other SMACC2 CLAUDE.md File Locations

  ### Client Library  
  SMACC2/smacc2_client_library/CLAUDE.md

  ### Reference Library
  SMACC2/smacc2_sm_reference_library/CLAUDE.md

  ### Project Settings
  SMACC2/.claude/settings.json

//////////////////////////////////////////////////////////////////////////////

## Token Conservation
- Summarize lengthy outputs instead of showing full logs
- Group similar error messages
- Skip routine background process messages
- Prioritize actionable information over verbose logging

//////////////////////////////////////////////////////////////////////////////

# Other References

  ### Boost Statechart Documentation
  https://www.boost.org/doc/libs/1_80_0/libs/statechart/doc/index.html

  ### Boost Statechart Source Code
  https://github.com/boostorg/statechart

  ### Boost ASIO Documentation
  https://www.boost.org/doc/libs/1_89_0/doc/html/boost_asio.html

  ### Boost ASIO Source Code
  https://github.com/boostorg/asio
