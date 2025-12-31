# Implementation Plan: cl_gcalcli SMACC2 Client Library

## Overview

Create a new SMACC2 client library (`cl_gcalcli`) that integrates Google Calendar functionality via the `gcalcli` CLI tool, enabling state machines to react to calendar events.

**Key Decisions:**
- CpTerminalManager → Generic `CpSubprocessExecutor` in smacc2 core (reusable)
- gcalcli path → System `gcalcli` from PATH (installed via pip)
- CbEventDetect → Triggers when event START TIME arrives
- Behaviors → Full set implementation

## Architecture

```
ClGcalcli (Orchestrator)
    ├── CpSubprocessExecutor (CORE - generic subprocess execution)
    ├── CpGcalcliConnection (gcalcli-specific connection health)
    ├── CpCalendarPoller (agenda polling + TSV parsing)
    └── CpCalendarEventListener (event matching + dispatch)
```

## Components

### 1. CpSubprocessExecutor (smacc2 core component)
**Location:** `smacc2/include/smacc2/client_core_components/cp_subprocess_executor.hpp`
**Purpose:** Generic subprocess execution for any CLI tool

- Executes shell commands via `popen()` subprocess
- Captures stdout/stderr with configurable timeout
- Thread-safe with mutex protection
- Signals: `onCommandCompleted_(int exit_code, std::string output)`, `onCommandFailed_(std::string error)`

### 2. CpGcalcliConnection (cl_gcalcli component)
**Purpose:** gcalcli-specific connection health monitoring

- Uses CpSubprocessExecutor to run gcalcli commands
- Implements `ISmaccUpdatable` for periodic heartbeat (uses `gcalcli list`)
- Tracks consecutive failures, emits `EvConnectionLost` after 3 failures
- Provides `restartConnection()` method
- Signals: `onConnectionLost_`, `onConnectionRestored_`, `onAuthenticationRequired_`

### 3. CpCalendarPoller
**Purpose:** Poll calendar and parse event data

- Uses CpGcalcliConnection to execute `gcalcli agenda --tsv`
- Parses TSV output into `CalendarEvent` structs
- Maintains cached event list with mutex protection
- Implements `ISmaccUpdatable` for periodic polling
- Signal: `onAgendaUpdated_(std::vector<CalendarEvent>)`

### 4. CpCalendarEventListener
**Purpose:** Match events against patterns and dispatch (time-based triggering)

- Subscribes to CpCalendarPoller's data signal
- Supports regex or exact-match pattern watching
- Tracks triggered events to prevent duplicates
- Detects event start/end times
- Signals: `onEventDetected_`, `onEventStarted_`, `onEventEnded_`

## Events

```cpp
// Connection events
EvConnectionLost<TSource, TOrthogonal>
EvConnectionRestored<TSource, TOrthogonal>
EvAuthenticationRequired<TSource, TOrthogonal>

// Calendar events
EvCalendarEventDetected<TSource, TOrthogonal> { CalendarEvent event; std::string pattern; }
EvCalendarEventStarted<TSource, TOrthogonal> { CalendarEvent event; }
EvCalendarEventEnded<TSource, TOrthogonal> { CalendarEvent event; }
EvAgendaUpdated<TSource, TOrthogonal> { std::vector<CalendarEvent> events; }
```

## Data Structures

```cpp
struct CalendarEvent {
  std::string id, title, calendar_name, location, description;
  std::chrono::system_clock::time_point start_time, end_time;
  bool is_all_day;
  bool isActiveNow() const;
  bool willStartWithinMinutes(int minutes) const;
};

struct GcalcliConfig {
  std::string gcalcli_path = "gcalcli";
  std::vector<std::string> calendars;  // empty = all
  std::chrono::seconds poll_interval{30};
  std::chrono::seconds heartbeat_interval{60};
  int agenda_days{7};
};
```

## Client Behaviors

| Behavior | Type | Purpose |
|----------|------|---------|
| `CbEventDetect` | Async | Wait until matching event's START TIME arrives (regex/exact, minutes before) |
| `CbStatus` | Sync | Get connection state and current events |
| `CbWaitConnection` | Async | Wait for gcalcli connection with timeout |
| `CbMonitorConnection` | Sync+Updatable | Continuous connection monitoring, posts events on loss |
| `CbQuickAdd` | Async | Add event via `gcalcli quick "text"` |
| `CbRefreshAgenda` | Sync | Force immediate agenda refresh |

## File Structure

### smacc2 Core Addition
```
src/SMACC2/smacc2/include/smacc2/client_core_components/
└── cp_subprocess_executor.hpp   # NEW - generic subprocess component

src/SMACC2/smacc2/src/smacc2/
└── cp_subprocess_executor.cpp   # NEW
```

### cl_gcalcli Package
```
src/SMACC2/smacc2_client_library/cl_gcalcli/
├── include/cl_gcalcli/
│   ├── cl_gcalcli.hpp
│   ├── types.hpp
│   ├── events.hpp
│   ├── client_behaviors/
│   │   ├── cb_event_detect.hpp
│   │   ├── cb_status.hpp
│   │   ├── cb_wait_connection.hpp
│   │   ├── cb_monitor_connection.hpp
│   │   ├── cb_quick_add.hpp
│   │   └── cb_refresh_agenda.hpp
│   └── components/
│       ├── cp_gcalcli_connection.hpp
│       ├── cp_calendar_poller.hpp
│       └── cp_calendar_event_listener.hpp
├── src/cl_gcalcli/
│   ├── cl_gcalcli.cpp
│   ├── client_behaviors/*.cpp
│   └── components/*.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Implementation Order

### Phase 1: smacc2 Core Addition
1. Implement CpSubprocessExecutor in smacc2/client_core_components/ (generic subprocess)
2. Update smacc2 CMakeLists.txt to include new component

### Phase 2: cl_gcalcli Package Setup
3. Create cl_gcalcli package structure (CMakeLists.txt, package.xml)
4. Define types.hpp (CalendarEvent, GcalcliConfig, ConnectionState)
5. Define events.hpp (all event templates)

### Phase 3: cl_gcalcli Components
6. Implement CpGcalcliConnection (uses CpSubprocessExecutor, heartbeat monitoring)
7. Implement CpCalendarPoller (TSV parsing + caching)
8. Implement CpCalendarEventListener (pattern matching, time-based triggering)
9. Implement ClGcalcli client (component orchestration)

### Phase 4: Client Behaviors
10. Implement CbEventDetect (triggers when event start time arrives)
11. Implement CbStatus
12. Implement CbWaitConnection
13. Implement CbMonitorConnection
14. Implement CbQuickAdd
15. Implement CbRefreshAgenda

### Phase 5: Documentation
16. Create README.md with usage examples

## Key Implementation Details

### gcalcli Command Execution (via CpSubprocessExecutor)
```cpp
// CpSubprocessExecutor provides generic interface
bool executeCommand(const std::string& command, std::string& stdout, std::string& stderr, int timeout_ms);

// CpGcalcliConnection uses it for calendar commands
std::string output, error;
subprocessExecutor_->executeCommand("gcalcli agenda --tsv --nocolor", output, error, 30000);
```

### TSV Parsing (gcalcli agenda --tsv)
Columns: Start_Date | Start_Time | End_Date | End_Time | Title | Location | Description | Calendar

### Heartbeat Strategy
- Execute `gcalcli list` every 60 seconds
- 3 consecutive failures → EvConnectionLost
- Parse error messages for auth failures

### Polling Recommendations
- Heartbeat: 60 seconds
- Agenda poll: 30 seconds
- Event matching: Every update cycle (~50ms, in-memory)

## Reference Files

- Pattern: `smacc2_client_library/cl_ros2_timer/` (timer client structure)
- Pattern: `smacc2_client_library/cl_keyboard/` (event listener component)
- Pattern: `smacc2_client_library/cl_http/` (async behaviors)
- gcalcli: `src/gcalcli/gcalcli/gcal.py` (command output formats)

## Configuration

- **gcalcli executable:** `gcalcli` (from PATH, installed via pip)
- **Credentials location:** `~/.local/share/gcalcli/oauth` or `~/.config/gcalcli/`
- **Default poll interval:** 30 seconds
- **Default heartbeat interval:** 60 seconds

## Usage Example

```cpp
// Orthogonal setup
class OrCalendar : public smacc2::Orthogonal<OrCalendar> {
  void onInitialize() override {
    cl_gcalcli::GcalcliConfig config;
    config.gcalcli_path = "gcalcli";  // from PATH
    config.poll_interval = std::chrono::seconds{30};
    this->createClient<cl_gcalcli::ClGcalcli>(config);
  }
};

// State that waits for calendar event START TIME to arrive
struct StWaitForMeeting : smacc2::SmaccState<StWaitForMeeting, SmMain> {
  using reactions = mpl::list<
    // Transition when "Standup" event START TIME arrives (5 min before)
    Transition<EvCalendarEventStarted<CpCalendarEventListener, OrCalendar>, StMeetingStarted>,
    // Handle connection loss
    Transition<EvConnectionLost<CpGcalcliConnection, OrCalendar>, StConnectionError>
  >;

  static void staticConfigure() {
    // Wait for "Standup" (regex), trigger 5 minutes before start time
    configure<OrCalendar, CbEventDetect>(".*Standup.*", /*regex=*/true, /*minutes_before=*/5);
    // Monitor connection health
    configure<OrCalendar, CbMonitorConnection>();
  }
};
```
