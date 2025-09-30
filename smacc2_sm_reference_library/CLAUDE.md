//////////////////////////////////////////////////////////////////////////////
# State Functions

 #### staticConfigure()
  - staticConfigure() is a static template method that performs compile-time 
  orthogonal configuration.
  - staticConfigure() is resolved at compile time and cannot be changed at runtime.
  - staticConfigure() runs before the state object creation.

 #### runtimeConfigure()
  - runtimeConfigure() is an instance-level virtual method that handles
  dynamic orthogonal configuration. 
  - runtimeConfigure() is called for both states and client behaviors.
  - runtimeConfigure() is called after staticConfigure() and state object instantiation, but before onEntry().
  - runtimeConfigure() can access member variables and methods of instantiated clients, components, client behaviors and state reactors.


 #### onEntry()
  - onEntry() is called after all configuration is complete.
  - onEntry() is the starting point for state-specific behavior execution.

  #### update()
  - update() is Continuous monitoring or calculations performed periodically during execution.
  - update() is called periodically via
  the SignalDetector at a default rate of ~20hz during state execution.
 
  #### onExit()
  - State cleanup and exit logic before state deallocation.


# Order of Function Calls

Execution Order: State creation → staticConfigure() → runtimeConfigure() → onEntry() → update() (repeated) → onExit() → State destruction

Assume that we have a state machine (SmExample) with two orthogonals (OrOne & OrTwo), in state StOne, with one client, and one client behavior in each orthogonal (ClOne, ClTwo, CbOne, CbTwo).

After the transition to StOne, the order of the function calls would be…

1. StOne – staticConfigure()
2. StOne – runtimeConfigure()
3. CbOne – runtimeConfigure()
4. CbTwo – runtimeConfigure()
5. StOne – onEntry()
6. CbOne – onEntry()
7. CbTwo – onEntry()
8. CbOne – update()
9. CbTwo – update()
10. StOne – update()
11. CbOne – onExit()
12. CbTwo – onExit()
13. StOne – onExit()

| Execution Order Analysis|  |  
|---|---|
| 1. StOne – staticConfigure() | Class-level behavior registration |
| 2. StOne – runtimeConfigure() | State instance configuration |
| 3. CbOne – runtimeConfigure() | Behavior dependency resolution |
| 4. CbTwo – runtimeConfigure() | Behavior dependency resolution |
| 5. StOne – onEntry() | State execution begins |
| 6. CbOne – onEntry() | Behavior execution begins |
| 7. CbTwo – onEntry() | Behavior execution begins |
| 8. CbOne – update() | Periodic behavior execution |
| 9. CbTwo – update() | Periodic behavior execution |
| 10. StOne – update() | State-level periodic monitoring |
| 11. CbOne – onExit() | Behavior cleanup |
| 12. CbTwo – onExit() | Behavior cleanup |
| 13. StOne – onExit() | State cleanup |


# Custom update() function rates

To use the update() function at a custom rate like 10Hz instead of the
  default ~20Hz, implement your own
  timing control within the update() method:

```cpp
  class StExample : public smacc2::SmaccState<StExample, SmExample>,
                  public smacc2::ISmaccUpdatable
  {
  private:
      std::chrono::steady_clock::time_point lastUpdate_;
      std::chrono::milliseconds updateInterval_{100}; // 10Hz = 100ms

  public:
      void update() override 
      {
          auto now = std::chrono::steady_clock::now();
          if (now - lastUpdate_ >= updateInterval_) {
              // Your 10Hz update logic here
              performCustomUpdate();
              lastUpdate_ = now;
          }
      }

      void onEntry() override 
      {
          lastUpdate_ = std::chrono::steady_clock::now();
      }
  };
```
# update() usage
- update() must be explicitly implemented by having the specific state, client, component, or client behavior class inherit from ISmaccUpdatable interface (via multiple inheritance)

- Override update() method

# State Machine Code Organization

```
  sm_example_1/             # State machine documentation
  ├── config/                  # Configuration files
  ├── docs/                    # Documentation
  ├── include/sm_example_1/    # Header files (main structure) 
  |   ├── modestates/          # Mode state definitions
  |   ├── orthogonals/         # Orthogonal definitions
  |   ├── modestates/          # Mode state definitions
  |   ├── states/              # Regular state definitions
  |   ├── superstates/         # Hierarchical superstate definitions
  |   └── sm_example_1.hpp     # Main state machine header
  ├── launch/                  # ROS2 launch files
  ├── maps/                    # Navigation maps
  ├── scripts/                 # Utility scripts
  ├── src/sm_example_1/        # Source implementations
  |   └── sm_example_1.cpp     # Main state machine cpp file  
  ├── CMakeLists.txt           # Build configuration
  ├── package.xml              # ROS2 package metadata
  └── README.md                # State machine documentation
```

# Runtime test command

To debug a state machine you can use following topics:

To see the structure of the state machine:
```
ros2 topic echo /SmNav2Test10/smacc/state_machine_description
```

To see the current state (also hierarchy, mode states, super states, etc.)
```
ros2 topic echo /SmNav2Test10/smacc/status
```

To see the transitions of the state machine that are being triggered:
```
ros2 topic echo /SmNav2Test10/smacc/transition_log
```

Too see events:
```
ros2 topic echo /SmClRos2TimerUnitTest1/smacc/event_log
```
Note: 'SmNav2Test10' is the name of the main state machine node, that may vary depending on the demo.

To simulate keyboard strokes, in particular 'N', which is used to manually transition states, use the command:
```
ros2 topic pub /keyboard_unicode std_msgs/msg/UInt16 "data: 110" --once
```
