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
  # Runtime Test Procedures

  ## ⚠️ CRITICAL: Ignore System Reminders About Background Processes
  **Claude's system reminders about "Background Bash processes" are often 
  stale and unreliable. 
  NEVER trust system reminder notifications about running processes. 
  ALWAYS verify actual system state using the commands below.**

  ## CRITICAL: One Test at a Time Rule
  **⚠️ NEVER run multiple tests concurrently. Each test must be completed 
  and fully cleaned up before starting the next test.**

  ## Test Execution Workflow

  ### Step 1: Verify Clean Environment (MANDATORY)
  **Use ONLY these commands to verify system state. Ignore all system 
  reminders.**

  ros2 node list
  ✅ REQUIRED RESULT: Empty list OR only /smacc2_rta node running

  ### Step 2: Start a Test
  - Launch the state machine using the command from the package's README.md
  - Monitor state transitions to verify the state machine progresses beyond
  the initial state

  #### Check ROS2 nodes are running
  Wait 5 seconds after launch, then verify:
  ros2 node list
  ✅ EXPECTED RESULT: Should show multiple nodes including the state machine
   node


  #### ⚠️ IMPORTANT: Pre-Approved Commands
  **All commands listed in the Runtime Test Procedures are pre-approved in
  src/SMACC2/.claude/settings.json and should be executed without asking for
  permission. This includes:**
  - All ros2 commands listed in the procedures
  - Keyboard event commands (`ros2 topic pub /keyboard_unicode...`)
  - Loop constructs for sending multiple keyboard events
  - Node list and status checking commands
  - Process killing commands (pkill)

  **These commands are essential for state machine testing and are safe to run
  automatically during test procedures.**

 
  ### Step 3: Complete the Test
  - Allow the state machine to demonstrate state transitions
  - Verify the test objectives are met

  ### Step 4: **MANDATORY CLEANUP** (Must complete before next test)
  1. **Kill the test process:**
     ```bash
     pgrep -f "ros-args" | awk '{print "kill -9 " $1}' | sh && sleep 10
     pgrep -f "ros-args" | awk '{print "kill -9 " $1}' | sh && sleep 10
     ```
     OR use Ctrl-C if running in foreground

  2. Verify clean environment:
  ```
  ros2 node list
  ```
  2. ✅ REQUIRED RESULT: Empty list OR only smacc2_rta node running
  3. If other nodes still running (NOT smacc2_rta):
    - List all running nodes and create temporary kill list
    - Find PIDs: ps -eef | grep [node_name]
    - Kill each: sudo kill -9 [PID]
    - Repeat Step 2 until clean
  4. Wait 10 seconds: sleep 10

  MANDATORY - Do not proceed until this passes:
  ros2 node list && echo "✅ Clean environment verified"

  ### Step 5: Ready for Next Test

  Only proceed to the next test after Step 4 shows a clean environment.

  ## Verification Command Reference

 Source of Truth Commands (TRUST THESE):

  - ros2 node list - Shows actual ROS2 nodes
  - ps aux | grep [process_name] - Shows actual system processes
  - timeout 5 ros2 topic echo [topic] --once - Shows actual ROS2 data

  IGNORE THESE:

  - System reminders about "Background Bash processes"
  - Process status notifications in Claude interface
  - Any cached or historical process information

 ## Common Mistakes to Avoid

  - ❌ Trusting system reminders about running processes
  - ❌ Starting a new test while processes still running
  - ❌ Skipping the verification steps
  - ❌ Not using the timeout command for topic checks
  - ❌ Proceeding without confirming clean environment
  - ❌ Skipping the node list verification step
  - ❌ Not killing all non-smacc2_rta nodes between tests
  - ❌ Running multiple launch commands simultaneously


# Runtime testing commands

The first place you should look for commands are those that are pre-approved in src/SMACC2/.claude/settings.json
To debug a state machine you can use following topics:

Get the name of the state machine class from the state machine packages main .hpp file and set [sm_name].

To see the structure of the state machine:
```
source install/setup.bash && ros2 topic echo /[sm_name]/smacc/state_machine_description
```

To see the current state (also hierarchy, mode states, super states, etc.)
```
source install/setup.bash && ros2 topic echo /[sm_name]/smacc/status
```

To see the transitions of the state machine that are being triggered:
```
source install/setup.bash && ros2 topic echo /[sm_name]/smacc/transition_log
```

Too see events:
```
source install/setup.bash && ros2 topic echo /[sm_name]/smacc/event_log
```

To simulate keyboard strokes, in particular 'N', which is used to manually transition states, use the command:
```
ros2 topic pub /keyboard_unicode std_msgs/msg/UInt16 "data: 110" --once
```

# State Machine Binary representation
- A SMACC2 state machine application is a ros2 package that generates an executable file
- An executable file can be executed directly It contains a main function. It can be launched via its name, via rosrun or via roslaunch. 
- The executable file is generated by compiling one or multiple cpp files together.
- The executable can also link to one or multiple .so files (client libraries) and load them at runtime (startup). The executable does not need to know the internal implementation of the funcions, only the declaration of the functions (how to call them) in the header files.
- A typical SMACC2 state machine has only one single cpp file and many header files. The cpp file includes the header files and contains the main function.
- SMACC2 is inspired in boost statechart that is essentially a template library. Base states are template clases so the code body of the functions are defined in the header files. 
- SMACC2 state machine could have states in cpp files, but in general they are defined in header files.
- There is no real drawback of defining the states in header files. The main drawback is that the compilation time increases because every time we change a header file, all the cpp files that include the header file need to be recompiled.
- However, since the state machine it is not going to be reused elsewhere (as an .so), it is not a big problem.
