# Implementation Plan: sm_cl_gcalcli_test_1

## Overview

Create a unit test state machine for the `cl_gcalcli` client library. The state machine will exercise all behaviors and components to verify proper functionality of the Google Calendar integration via gcalcli.

## Test Strategy

### Test Scope

The test state machine will verify:
1. **Connection behaviors** - CbWaitConnection, CbMonitorConnection
2. **Status behaviors** - CbStatus, CbRefreshAgenda
3. **Event detection** - CbEventDetect with time-based triggering
4. **Event addition** - CbQuickAdd
5. **Event transitions** - All cl_gcalcli events (EvConnectionLost, EvConnectionRestored, etc.)

### Testing Approach

Since gcalcli requires actual Google Calendar authentication and network access, the test will:
1. Use a timer-based fallback for state transitions when gcalcli is not available
2. Log all behavior entry/exit points for verification
3. Test event handling patterns regardless of actual calendar data

## Architecture

```
SmClGcalcliTest1
├── OrCalendar (gcalcli client orthogonal)
│   └── ClGcalcli (with default config)
└── OrTimer (fallback transition timer)
    └── ClRos2Timer

States:
┌─────────────────┐     ┌──────────────────┐     ┌────────────────────┐
│   StInit        │────▶│ StWaitConnection │────▶│ StTestRefresh      │
│ (Entry point)   │     │ (CbWaitConnection│     │ (CbRefreshAgenda)  │
└─────────────────┘     │  CbStatus)       │     └────────┬───────────┘
                        └──────────────────┘              │
                                                          ▼
┌─────────────────┐     ┌──────────────────┐     ┌────────────────────┐
│   StDone        │◀────│ StTestQuickAdd   │◀────│ StTestEventDetect  │
│ (Final state)   │     │ (CbQuickAdd)     │     │ (CbEventDetect,    │
└─────────────────┘     └──────────────────┘     │  CbMonitorConn)    │
                                                  └────────────────────┘
```

## File Structure

```
smacc2_sm_reference_library/sm_cl_gcalcli_test_1/
├── include/sm_cl_gcalcli_test_1/
│   ├── sm_cl_gcalcli_test_1.hpp       # Main SM header
│   ├── orthogonals/
│   │   ├── or_calendar.hpp            # Calendar orthogonal (ClGcalcli)
│   │   └── or_timer.hpp               # Timer orthogonal (fallback)
│   └── states/
│       ├── st_init.hpp                # Initial state
│       ├── st_wait_connection.hpp     # Test CbWaitConnection + CbStatus
│       ├── st_test_refresh.hpp        # Test CbRefreshAgenda
│       ├── st_test_event_detect.hpp   # Test CbEventDetect + CbMonitorConnection
│       ├── st_test_quick_add.hpp      # Test CbQuickAdd
│       └── st_done.hpp                # Final state (loop back or exit)
├── src/sm_cl_gcalcli_test_1/
│   └── sm_cl_gcalcli_test_1_node.cpp  # Main entry point
├── launch/
│   └── sm_cl_gcalcli_test_1.py        # Launch file
├── config/
│   └── sm_cl_gcalcli_test_1.yaml      # Config (optional)
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Detailed State Definitions

### StInit (Initial State)
- **Purpose**: Entry point, immediate transition to testing
- **Behaviors**: None
- **Transitions**:
  - Timer → StWaitConnection (after 1 second)

### StWaitConnection
- **Purpose**: Test connection waiting and status behaviors
- **Behaviors**:
  - `CbWaitConnection` - Async wait for gcalcli connection
  - `CbStatus` - Sync status query
- **Transitions**:
  - `EvCbSuccess<CbWaitConnection>` → StTestRefresh (connection established)
  - `EvCbFailure<CbWaitConnection>` → StDone (connection failed, skip remaining tests)
  - Timer (10s timeout) → StTestRefresh (fallback for testing without gcalcli)

### StTestRefresh
- **Purpose**: Test agenda refresh behavior
- **Behaviors**:
  - `CbRefreshAgenda` - Sync agenda refresh
- **Transitions**:
  - Timer (3s) → StTestEventDetect (proceed after refresh completes)

### StTestEventDetect
- **Purpose**: Test event detection and connection monitoring
- **Behaviors**:
  - `CbEventDetect("TestEvent", /*regex=*/false, /*minutes_before=*/5)` - Watch for test events
  - `CbMonitorConnection` - Continuous connection monitoring
- **Transitions**:
  - `EvCalendarEventStarted` → StTestQuickAdd (event detected)
  - `EvConnectionLost` → StWaitConnection (reconnection needed)
  - Timer (10s) → StTestQuickAdd (fallback for testing)

### StTestQuickAdd
- **Purpose**: Test quick event addition
- **Behaviors**:
  - `CbQuickAdd("Test Event from sm_cl_gcalcli_test_1 tomorrow 2pm")` - Add test event
- **Transitions**:
  - `EvCbSuccess<CbQuickAdd>` → StDone (success)
  - `EvCbFailure<CbQuickAdd>` → StDone (failure logged)
  - Timer (10s) → StDone (timeout fallback)

### StDone
- **Purpose**: Final state, log test completion
- **Behaviors**: None
- **Transitions**:
  - Timer (5s) → StInit (loop for continuous testing)

## Events to Test

| Event | Source | Test State |
|-------|--------|------------|
| `EvConnectionLost<CpGcalcliConnection, OrCalendar>` | CpGcalcliConnection | StTestEventDetect |
| `EvConnectionRestored<CpGcalcliConnection, OrCalendar>` | CpGcalcliConnection | StTestEventDetect |
| `EvAuthenticationRequired<CpGcalcliConnection, OrCalendar>` | CpGcalcliConnection | (any state) |
| `EvCalendarEventDetected<CpCalendarEventListener, OrCalendar>` | CpCalendarEventListener | StTestEventDetect |
| `EvCalendarEventStarted<CpCalendarEventListener, OrCalendar>` | CpCalendarEventListener | StTestEventDetect |
| `EvCalendarEventEnded<CpCalendarEventListener, OrCalendar>` | CpCalendarEventListener | StTestEventDetect |
| `EvAgendaUpdated<CpCalendarPoller, OrCalendar>` | CpCalendarPoller | StTestRefresh |
| `EvCbSuccess<CbWaitConnection, OrCalendar>` | CbWaitConnection | StWaitConnection |
| `EvCbFailure<CbWaitConnection, OrCalendar>` | CbWaitConnection | StWaitConnection |
| `EvCbSuccess<CbQuickAdd, OrCalendar>` | CbQuickAdd | StTestQuickAdd |
| `EvCbFailure<CbQuickAdd, OrCalendar>` | CbQuickAdd | StTestQuickAdd |

## Implementation Order

### Phase 1: Package Structure
1. Create package directory structure
2. Create CMakeLists.txt
3. Create package.xml

### Phase 2: Orthogonals
4. Create or_timer.hpp (reuse from sm_cl_keyboard_unit_test_1)
5. Create or_calendar.hpp (new - creates ClGcalcli)

### Phase 3: States
6. Create st_init.hpp
7. Create st_wait_connection.hpp
8. Create st_test_refresh.hpp
9. Create st_test_event_detect.hpp
10. Create st_test_quick_add.hpp
11. Create st_done.hpp

### Phase 4: Main Files
12. Create sm_cl_gcalcli_test_1.hpp
13. Create sm_cl_gcalcli_test_1_node.cpp

### Phase 5: Launch & Config
14. Create launch file
15. Create README.md

### Phase 6: Build & Test
16. Build package
17. Run test (requires gcalcli setup)

## Dependencies

- smacc2
- cl_gcalcli
- cl_ros2_timer

## Configuration

```yaml
# sm_cl_gcalcli_test_1.yaml (optional)
sm_cl_gcalcli_test_1:
  ros__parameters:
    # gcalcli config can be overridden here if needed
    poll_interval: 30.0
    heartbeat_interval: 60.0
    agenda_days: 7
```

## Usage Example

```bash
# Build
colcon build --packages-select sm_cl_gcalcli_test_1

# Run (requires gcalcli to be authenticated)
ros2 launch sm_cl_gcalcli_test_1 sm_cl_gcalcli_test_1.py

# Monitor state transitions
ros2 topic echo /sm_cl_gcalcli_test_1/smacc/status

# Monitor events
ros2 topic echo /sm_cl_gcalcli_test_1/smacc/event_log
```

## Key Implementation Notes

1. **Timer Fallback**: Each state uses timer-based transitions to allow testing even when gcalcli is not configured. This makes the test runnable in CI/CD environments.

2. **Logging**: All states log entry/exit and behavior results for debugging.

3. **Event Handling**: The state machine demonstrates proper event handling patterns that can be used as a reference for real applications.

4. **No External Dependencies**: The test runs without requiring actual calendar events - it tests the framework integration, not the calendar data.

5. **Loop Mode**: The state machine loops from StDone back to StInit for continuous testing/demonstration.
