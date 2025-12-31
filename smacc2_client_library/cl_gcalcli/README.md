# cl_gcalcli - SMACC2 Google Calendar Client

A SMACC2 client library for Google Calendar integration via the `gcalcli` CLI tool. This client enables state machines to react to Google Calendar events.

## Prerequisites

- gcalcli installed and authenticated (`pip install gcalcli`)
- Google Calendar API credentials configured
- ROS2 and SMACC2 installed

## Architecture

```
ClGcalcli (Orchestrator)
    ├── CpSubprocessExecutor (smacc2 core - generic subprocess execution)
    ├── CpGcalcliConnection (connection health monitoring)
    ├── CpCalendarPoller (agenda polling + TSV parsing)
    └── CpCalendarEventListener (pattern matching + event dispatch)
```

## Components

| Component | Purpose |
|-----------|---------|
| `CpSubprocessExecutor` | Generic subprocess execution (smacc2 core) |
| `CpGcalcliConnection` | Connection health monitoring, heartbeat, authentication |
| `CpCalendarPoller` | Periodic agenda fetching and parsing |
| `CpCalendarEventListener` | Event pattern matching and triggering |

## Client Behaviors

| Behavior | Type | Purpose |
|----------|------|---------|
| `CbEventDetect` | Async | Wait until matching event's start time arrives |
| `CbStatus` | Sync | Get connection state and current events |
| `CbWaitConnection` | Async | Wait for gcalcli connection with timeout |
| `CbMonitorConnection` | Sync | Continuous connection monitoring |
| `CbQuickAdd` | Async | Add event via `gcalcli quick` |
| `CbRefreshAgenda` | Sync | Force immediate agenda refresh |

## Events

```cpp
// Connection events
EvConnectionLost<TSource, TOrthogonal>
EvConnectionRestored<TSource, TOrthogonal>
EvAuthenticationRequired<TSource, TOrthogonal>

// Calendar events
EvCalendarEventDetected<TSource, TOrthogonal>  // Contains: event, matched_pattern
EvCalendarEventStarted<TSource, TOrthogonal>   // Contains: event
EvCalendarEventEnded<TSource, TOrthogonal>     // Contains: event
EvAgendaUpdated<TSource, TOrthogonal>          // Contains: events vector
```

## Usage Example

### Orthogonal Setup

```cpp
#include <cl_gcalcli/cl_gcalcli.hpp>
#include <cl_gcalcli/client_behaviors.hpp>

class OrCalendar : public smacc2::Orthogonal<OrCalendar>
{
  void onInitialize() override
  {
    cl_gcalcli::GcalcliConfig config;
    config.gcalcli_path = "gcalcli";  // from PATH
    config.poll_interval = std::chrono::seconds{30};
    config.heartbeat_interval = std::chrono::seconds{60};
    config.agenda_days = 7;

    this->createClient<cl_gcalcli::ClGcalcli>(config);
  }
};
```

### State Machine Example

```cpp
#include <cl_gcalcli/cl_gcalcli.hpp>
#include <cl_gcalcli/client_behaviors.hpp>
#include <cl_gcalcli/events.hpp>

using namespace cl_gcalcli;

// Wait for connection state
struct StWaitConnection : smacc2::SmaccState<StWaitConnection, SmMain>
{
  using reactions = mpl::list<
    Transition<EvCbSuccess<CbWaitConnection, OrCalendar>, StWaitForMeeting>,
    Transition<EvCbFailure<CbWaitConnection, OrCalendar>, StConnectionError>
  >;

  static void staticConfigure()
  {
    configure<OrCalendar, CbWaitConnection>(std::chrono::seconds{30});
  }
};

// Wait for meeting to start
struct StWaitForMeeting : smacc2::SmaccState<StWaitForMeeting, SmMain>
{
  using reactions = mpl::list<
    // Transition when "Standup" event START TIME arrives (5 min before)
    Transition<EvCbSuccess<CbEventDetect, OrCalendar>, StMeetingStarted>,
    // Handle connection loss
    Transition<EvConnectionLost<CpGcalcliConnection, OrCalendar>, StConnectionError>
  >;

  static void staticConfigure()
  {
    // Wait for "Standup" (regex), trigger 5 minutes before start time
    configure<OrCalendar, CbEventDetect>(".*Standup.*", /*regex=*/true, /*minutes_before=*/5);
    // Monitor connection health
    configure<OrCalendar, CbMonitorConnection>();
  }
};

// Meeting started state
struct StMeetingStarted : smacc2::SmaccState<StMeetingStarted, SmMain>
{
  static void staticConfigure()
  {
    configure<OrCalendar, CbStatus>();
  }

  void onEntry()
  {
    // Access detected event from behavior
    CbEventDetect* behavior;
    this->getOrthogonal<OrCalendar>()->getClientBehavior(behavior);
    if (behavior)
    {
      auto event = behavior->getDetectedEvent();
      if (event)
      {
        RCLCPP_INFO(getLogger(), "Meeting: %s", event->title.c_str());
      }
    }
  }
};
```

### Quick Add Example

```cpp
struct StAddEvent : smacc2::SmaccState<StAddEvent, SmMain>
{
  using reactions = mpl::list<
    Transition<EvCbSuccess<CbQuickAdd, OrCalendar>, StDone>,
    Transition<EvCbFailure<CbQuickAdd, OrCalendar>, StError>
  >;

  static void staticConfigure()
  {
    configure<OrCalendar, CbQuickAdd>("Team meeting tomorrow at 3pm for 1 hour");
  }
};
```

## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gcalcli_path` | `"gcalcli"` | Path to gcalcli executable |
| `config_folder` | `std::nullopt` | Optional gcalcli config folder |
| `calendars` | `{}` | Calendars to monitor (empty = all) |
| `poll_interval` | `30s` | How often to poll agenda |
| `heartbeat_interval` | `60s` | How often to check connection |
| `agenda_days` | `7` | Days ahead to fetch |
| `max_consecutive_failures` | `3` | Failures before connection lost |

## Data Structures

### CalendarEvent

```cpp
struct CalendarEvent {
  std::string id;
  std::string title;
  std::string calendar_name;
  std::string location;
  std::string description;
  std::chrono::system_clock::time_point start_time;
  std::chrono::system_clock::time_point end_time;
  bool is_all_day;

  bool isActiveNow() const;
  bool willStartWithinMinutes(int minutes) const;
  bool hasEnded() const;
  int minutesUntilStart() const;
};
```

### EventWatch

```cpp
struct EventWatch {
  std::string pattern;       // Pattern to match event titles
  bool use_regex = false;    // True = regex, False = substring match
  int minutes_before = 0;    // Trigger N minutes before start
  bool trigger_on_start = true;
  bool trigger_on_end = false;
  bool continuous = false;   // Keep watching or one-shot
};
```

## Dependencies

- smacc2
- Boost (thread, regex)
- gcalcli (external CLI tool)

## Getting Started

### Step 1: Install gcalcli

```bash
# Option A: Using apt (Debian/Ubuntu)
sudo apt install gcalcli

# Option B: Using pipx (recommended for latest version)
pipx install gcalcli
```

### Step 2: Set Up Google Calendar API Credentials

The default gcalcli OAuth token is restricted. You need to create your own Google API credentials:

#### 2.1 Create a Google Cloud Project

1. Go to https://console.cloud.google.com/
2. Click "Select a project" → "New Project"
3. Name it something like `gcalcli-personal`
4. Click **Create**

#### 2.2 Enable the Calendar API

1. Go to **APIs & Services** → **Library**
2. Search for "Google Calendar API"
3. Click on it and click **Enable**

#### 2.3 Configure OAuth Consent Screen

1. Go to **APIs & Services** → **OAuth consent screen**
2. Select **External** → **Create**
3. Fill in:
   - App name: `gcalcli`
   - User support email: your email
   - Developer contact: your email
4. Click **Save and Continue**
5. On Scopes page, click **Add or Remove Scopes**
6. Add: `https://www.googleapis.com/auth/calendar`
7. Click **Save and Continue**
8. Add your email as a test user
9. Click **Save and Continue**

#### 2.4 Create OAuth Credentials

1. Go to **APIs & Services** → **Credentials**
2. Click **Create Credentials** → **OAuth client ID**
3. Application type: **Desktop app**
4. Name: `gcalcli`
5. Click **Create**
6. Copy the **Client ID** and **Client Secret**

### Step 3: Initialize gcalcli

```bash
gcalcli --client-id=YOUR_CLIENT_ID.apps.googleusercontent.com init
```

When prompted, enter your Client Secret. A browser window will open for OAuth authentication.

### Step 4: Verify Installation

```bash
# List your calendars
gcalcli list

# View upcoming events
gcalcli agenda
```

## Troubleshooting

### OAuth Redirect Failed

If you see "Unable to start local webserver on port 8080", old gcalcli processes may be blocking the port:

```bash
# Kill any stuck gcalcli processes
pkill -f gcalcli

# Try again
gcalcli --client-id=YOUR_CLIENT_ID.apps.googleusercontent.com init
```

### Creating a Dedicated Robot Calendar

For testing or robot-specific events, create a separate calendar:

1. Go to https://calendar.google.com/
2. Click the **+** next to "Other calendars"
3. Select "Create new calendar"
4. Name it (e.g., "Robot")
5. Click **Create calendar**

Then configure cl_gcalcli to use it:

```cpp
cl_gcalcli::GcalcliConfig config;
config.calendars = {"Robot"};  // Only monitor this calendar
this->createClient<cl_gcalcli::ClGcalcli>(config);
```

### Testing Event Detection

Create a test event and verify detection:

```bash
# Create an event 10 minutes from now
gcalcli --calendar="Robot" quick "TestEvent at 2:30pm today for 15 minutes"

# Verify it appears
gcalcli --calendar="Robot" agenda
```

## Reference

- [gcalcli GitHub](https://github.com/insanum/gcalcli)
- [gcalcli API Auth Documentation](https://github.com/insanum/gcalcli/blob/HEAD/docs/api-auth.md)
