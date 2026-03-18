### Loading Key Files into Context
```
Load the following files into context:
src/SMACC2/.claude/settings.json
src/SMACC2/CLAUDE.md
src/SMACC2/smacc2_client_library/CLAUDE.md
src/SMACC2/smacc2_sm_reference_library/CLAUDE.md
```
### Client & Initial State Machine Creation: PX4
```
Ultrathink and make a plan, but don't code yet, to create a new client: cl_px4_mr
What should the component architecture look like? What components will be necessary achieve the Client behaviors listed?

And a simple state machine application to serve as a unit test...

	Named: sm_cl_phx_mr_test_1
	Located: src/SMACC2/smacc2_sm_reference_library/
 
The state machine should have the following architecture: 
MsDisarmedOnGround | MsArmedOnGround | MsTakeoff | MsInFlight| MsLanding | MsLanded |

And should proceed in a linear pattern. 

MsDisarmedOnGround:StArmPX4:CbArmPX4  | MsArmedOnGround:StPrepareForTakeoff:CbWait | MsTakeoff:StTakeoff:CbTakeOff | MsInFlight:StGoToWaypoint1:CbGoToLocation, MsInFlight:StOrbitLocation:CbOrbitLocation, MsInFlight:StReturnToBase:CbGoToLocation | MsLanding:StLand:CbLand | MsLanded:StLanded:CbDisarmPX4 | 

With the following client behaviors: 

cbGoToLocation
CbOrbitLocation
CbArmPX4
CbDisarmPX4
CbTakeOff
CbLand

Our initial test will be connecting to the instance of...
make px4_sitl gz_x500

All runtime testing will be performed by me (the human). Let me know when to conduct a test and Ill let you know the results.
```
### State Machine Creation: sm_nav2_unit_test_1
```
Ultrathink and make a plan, but don't code yet, to create a new package in the reference library, sm_nav2_unit_test_1.

The packages launch file should follow the pattern of the other launch files in the sm_reference_library, but it should also launch the 'nav2_bringup tb3_simulation_launch.py headless:=False' launch file.

For the State Machine, lets create four states, with the specified behaviors in each state.
StAllSensorsGo:CbTimerCountdownOnce(10), CbDefaultKeyboardBehavior -> StNavigateToWaypoint1:CbNavigateGlobalPosition,CbDefaultKeyboardBehavior -> StRotate:CbPureSpinning -> StNavigateToWaypoint2:CbNavigateGlobalPosition ->StFinalState

And 3 orthogonals 
OrNavigation:cl_nav2z
OrTimer:cl_ros2_timer
OrKeyboard: cl_keyboard

Organize the phases of the plan so that each phase can be successfully compiled and tested at runtime
using the sm_panda_cl_moveit2z_cb_inventory package.

All testing will be performed by me (the human). Let me know when to conduct a test and Ill let you know
the results.
```
### State Machine Debugging
```
lets run and debug the sm_panda_moveit2z_cb_inventory package. Use the launch command ros2 launch sm_panda_moveit2z_cb_inventory sm_panda_moveit2z_cb_inventory.launch.py and monitor the topic using the command: 
source install/setup.bash && ros2 topic echo /SmPandaMoveit2zCbInventory/smacc/transition_log

The following states aren't working in the sense that they show no movement:
StMoveCartesianRelative2
StMoveCartesianRelative
StPouringMotion
StCircularPivotMotion

 look in the log files (~/.ros/log) for the most recent state machine run.
  Analyse the move_group, state machine, and other logs and creat a plan to
  fix the error messages that are occuring in StMoveCartesianRelative2
```
### Client Refactoring
```
Think hard and make a plan, but don't code yet, to refactor the cl_ros2_timer package so that its style follows the component based architecture shown in the cl_keyboard package. The plan should create two new components:
 smacc2::client_core_components::CpRos2Timer
 cl_ros2_timer::components::CpTimerListener1
    
where cl_ros2_timer::components::CpTimerListener1 depends on smacc2::client_core_components::CpRos2Timer.
The logic found in ClRos2Timer::onInitialize() should be moved to smacc2::client_core_components::CpRos2Timer. The logic found in ClRos2Timer::timerCallback() should be moved to cl_ros2_timer::components::CpTimerListener1

Use the cl_keyboard as the example, and compile and test frequently using the sm_cl_ros2_timer_unit_test_1 package and the run command:ros2 launch
 sm_cl_ros2_timer_unit_test_1 
 sm_cl_ros2_timer_unit_test_1.launch
```
```
Think hard and make a plan, but don't code yet, to refactor the cl_moveit2z package so that its style follows a pure component based architecture. 

Organize the phases of the so that each phase can be successfully compiled and tested at runtime using the sm_panda_cl_moveit2z_cb_inventory package.

CRITICAL: Do not modify the sm_panda_cl_moveit2z_cb_inventory package for any reason.

All testing will be performed by me (the human). Let me know when to conduct a test and Ill let you know the results. 
```
```
Ultrathink and make a plan, but don't code yet, to refactor the cl_moveit2z package so that all of the client behaviors are header-only.

Organize the phases of the plan so that each phase can be successfully compiled and tested at runtime using the sm_panda_cl_moveit2z_cb_inventory package.

CRITICAL: Do not modify the sm_panda_cl_moveit2z_cb_inventory package for any reason.
```
### Let's stop this from happening again
```
yes, but first please suggest some edits to the sm_reference_library CLAUDE.md file Runtime Test Procedures section so that this never happens again.
```
```
add the bash command i approved to the src/SMACC2/.claude/settings.json file
```
### Critical Requirements Example
```
Perform a test running the sm_panda_moveit2z_cb_inventory package, following
  the Runtime Test Procedures described in the smacc2_sm_reference_library/CLAUDE.md
  file. Perform the test 3 times in a row, in all tests the state machine
  should at least transition out of StKnownState1, before closing it Check this by looking at the transition_log topic. 
  
   #### ⚠️ CRITICAL: Test Completion Requirements
  **When instructed to perform multiple tests (e.g., "3 tests"), Claude MUST
  complete ALL tests as specified. No exceptions.**

  **Enforcement rules:**
  - Use TodoWrite tool to track all test completions and enforce accountability

  Follow the test procedures exactly. Take no shortcuts. 
  Focus only on test results and errors. Ignore routine ROS node startup messages, background process outputs, and standard system logs unless they indicate failures.
```

All testing will be performed by me (the human). Let me know when to conduct a test and Ill let you know the results. 
```
