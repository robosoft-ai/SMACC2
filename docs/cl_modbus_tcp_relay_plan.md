# cl_modbus_tcp_relay Implementation Plan

## Overview
Create a SMACC2 client library for controlling an 8-channel Modbus TCP relay (Waveshare POE ETH Relay). The client uses **libmodbus** directly for Modbus TCP communication (mbpoll available separately for CLI debugging) and follows SMACC2's pure component-based architecture.

## Design Decisions
- **Library**: libmodbus C library (direct integration, not subprocess)
- **Location**: `src/SMACC2/smacc2_client_library/cl_modbus_tcp_relay/`
- **Behaviors**: SmaccAsyncClientBehavior for non-blocking relay operations

## Target Hardware
- **Device**: Waveshare 8-Channel POE ETH Relay
- **Protocol**: Modbus TCP
- **Default IP**: 192.168.1.254
- **Default Port**: 502
- **Slave ID**: 0x01
- **Coil Addresses**: 0x0000-0x0007 (channels 1-8), 0x00FF (all channels)

---

## Architecture

### Client: ClModbusTcpRelay
Pure orchestrator that creates and configures components during initialization.

**Constructor Parameters:** None - all configuration loaded from YAML config file.

**YAML Configuration (in state machine's config directory):**
```yaml
# sm_example/config/sm_example_config.yaml
sm_example:
  ros__parameters:
    modbus_relay:
      ip_address: "192.168.1.254"
      port: 502
      slave_id: 1
      heartbeat_interval_ms: 1000
      connect_on_init: true
```

**Default Values (if not specified in config):**
- `ip_address`: "192.168.1.254"
- `port`: 502
- `slave_id`: 1
- `heartbeat_interval_ms`: 1000
- `connect_on_init`: true

### Components

#### 1. CpModbusConnection
Manages libmodbus context lifecycle, TCP connection, and heartbeat monitoring.

**Responsibilities:**
- Create/destroy modbus_t context
- Manage TCP connection state
- Periodic heartbeat via ISmaccUpdatable (reads coil status to verify connectivity)
- Emit connection state change signals
- Thread-safe connection access via mutex

**Signals:**
- `onConnectionLost_` - Emitted when heartbeat fails
- `onConnectionRestored_` - Emitted when reconnection succeeds
- `onConnectionError_` - Emitted on connection errors with error message

**Methods:**
- `connect()` - Establish TCP connection
- `disconnect()` - Close connection gracefully
- `reconnect()` - Close and re-establish connection
- `isConnected()` - Check connection state
- `getContext()` - Get modbus_t* for operations (thread-safe)

**Pattern Reference:**
- `CpHttpConnectionManager` for connection lifecycle
- `CpMessageTimeout` for heartbeat/watchdog pattern
- `ISmaccUpdatable` for periodic monitoring

#### 2. CpModbusRelay
Handles Modbus coil read/write operations for the 8-channel relay.

**Responsibilities:**
- Write single coil (channel on/off)
- Write all coils simultaneously
- Read single coil status
- Read all coil statuses
- Emit operation result signals

**Signals:**
- `onWriteSuccess_` - Emitted on successful write with channel info
- `onWriteFailure_` - Emitted on write failure with error
- `onReadComplete_` - Emitted with current relay states (8-bit mask)

**Methods:**
- `writeCoil(int channel, bool state)` - Write single channel (1-8)
- `writeAllCoils(bool state)` - Write all channels
- `writeAllCoils(uint8_t mask)` - Write all channels with bitmask
- `readCoil(int channel)` - Read single channel state
- `readAllCoils()` - Read all channel states

**Modbus Protocol Mapping:**
| Operation | Function Code | Address |
|-----------|---------------|---------|
| Single Relay ON/OFF | 0x05 | 0x0000-0x0007 |
| All Relays ON/OFF | 0x0F | 0x0000 (8 coils) |
| Read Single Coil | 0x01 | 0x0000-0x0007 |
| Read All Coils | 0x01 | 0x0000 (8 coils) |

---

### Events

```cpp
// Connection events
template <typename TSource, typename TOrthogonal>
struct EvConnectionLost : sc::event<EvConnectionLost<TSource, TOrthogonal>> {};

template <typename TSource, typename TOrthogonal>
struct EvConnectionRestored : sc::event<EvConnectionRestored<TSource, TOrthogonal>> {};

// Relay operation events
template <typename TSource, typename TOrthogonal>
struct EvRelayWriteSuccess : sc::event<EvRelayWriteSuccess<TSource, TOrthogonal>> {};

template <typename TSource, typename TOrthogonal>
struct EvRelayWriteFailure : sc::event<EvRelayWriteFailure<TSource, TOrthogonal>> {};
```

---

### Client Behaviors

#### CbRelayOn
Turn on a specific relay channel.

```cpp
class CbRelayOn : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbRelayOn(int channel);  // channel 1-8

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation();

  void onEntry() override;

private:
  int channel_;
  CpModbusRelay* relayComponent_;
};
```

#### CbRelayOff
Turn off a specific relay channel.

#### CbAllRelaysOn
Turn on all 8 relay channels simultaneously.

#### CbAllRelaysOff
Turn off all 8 relay channels simultaneously.

#### CbRelayStatus
Read the status of relay channels (can be configured for single channel or all).

```cpp
class CbRelayStatus : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbRelayStatus();                    // Read all channels
  CbRelayStatus(int channel);         // Read specific channel

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation();

  void onEntry() override;

  // Callback for read completion
  virtual void onStatusRead(uint8_t channelStates);

private:
  std::optional<int> channel_;
  CpModbusRelay* relayComponent_;
};
```

---

## Package Structure

```
src/SMACC2/smacc2_client_library/cl_modbus_tcp_relay/
├── include/cl_modbus_tcp_relay/
│   ├── cl_modbus_tcp_relay.hpp           # Main client header
│   ├── client_behaviors/
│   │   ├── cb_relay_on.hpp
│   │   ├── cb_relay_off.hpp
│   │   ├── cb_all_relays_on.hpp
│   │   ├── cb_all_relays_off.hpp
│   │   └── cb_relay_status.hpp
│   └── components/
│       ├── cp_modbus_connection.hpp
│       └── cp_modbus_relay.hpp
├── src/cl_modbus_tcp_relay/
│   ├── cl_modbus_tcp_relay.cpp
│   └── components/
│       ├── cp_modbus_connection.cpp
│       └── cp_modbus_relay.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Implementation Details

### CpModbusConnection Implementation

```cpp
class CpModbusConnection : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  CpModbusConnection();  // No constructor params - config from YAML

  void onInitialize() override;
  ~CpModbusConnection();

  // Connection management
  bool connect();
  void disconnect();
  bool reconnect();
  bool isConnected() const;

  // Thread-safe context access
  modbus_t* getContext();
  std::mutex& getMutex();

  // Signals
  smacc2::SmaccSignal<void()> onConnectionLost_;
  smacc2::SmaccSignal<void()> onConnectionRestored_;
  smacc2::SmaccSignal<void(const std::string&)> onConnectionError_;

  // Event posting (set during onStateOrthogonalAllocation)
  template <typename TOrthogonal, typename TClient>
  void onStateOrthogonalAllocation();

protected:
  void update() override;  // Heartbeat check

private:
  // Configuration loaded from YAML in onInitialize()
  std::string ip_address_;
  int port_;
  int slave_id_;
  int heartbeat_interval_ms_;
  bool connect_on_init_;

  modbus_t* ctx_;
  bool connected_;
  mutable std::mutex mutex_;

  std::function<void()> postConnectionLostEvent_;
  std::function<void()> postConnectionRestoredEvent_;

  // Helper for parameter loading
  template <typename T>
  void declareAndLoadParam(const std::string& name, T& value, const T& default_val);
};
```

### Parameter Loading (in onInitialize())

```cpp
void CpModbusConnection::onInitialize()
{
  auto node = getNode();

  // Load configuration from YAML with defaults
  ip_address_ = "192.168.1.254";
  declareAndLoadParam("modbus_relay.ip_address", ip_address_, ip_address_);

  port_ = 502;
  declareAndLoadParam("modbus_relay.port", port_, port_);

  slave_id_ = 1;
  declareAndLoadParam("modbus_relay.slave_id", slave_id_, slave_id_);

  heartbeat_interval_ms_ = 1000;
  declareAndLoadParam("modbus_relay.heartbeat_interval_ms", heartbeat_interval_ms_, heartbeat_interval_ms_);

  connect_on_init_ = true;
  declareAndLoadParam("modbus_relay.connect_on_init", connect_on_init_, connect_on_init_);

  RCLCPP_INFO(getLogger(), "[CpModbusConnection] Config: %s:%d (slave=%d, heartbeat=%dms)",
    ip_address_.c_str(), port_, slave_id_, heartbeat_interval_ms_);

  // Set update period for heartbeat
  this->setUpdatePeriod(rclcpp::Duration::from_seconds(heartbeat_interval_ms_ / 1000.0));

  // Create modbus context
  ctx_ = modbus_new_tcp(ip_address_.c_str(), port_);
  if (ctx_)
  {
    modbus_set_slave(ctx_, slave_id_);
    modbus_set_response_timeout(ctx_, 1, 0);  // 1 second timeout

    if (connect_on_init_)
    {
      connect();
    }
  }
  else
  {
    RCLCPP_ERROR(getLogger(), "[CpModbusConnection] Failed to create modbus context");
  }
}

template <typename T>
void CpModbusConnection::declareAndLoadParam(const std::string& name, T& value, const T& default_val)
{
  auto node = getNode();
  if (!node->has_parameter(name))
  {
    node->declare_parameter(name, default_val);
  }
  node->get_parameter(name, value);
  RCLCPP_INFO(getLogger(), "[CpModbusConnection] %s: %s", name.c_str(), std::to_string(value).c_str());
}
```

### Heartbeat Implementation (in update())

```cpp
void CpModbusConnection::update()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!ctx_) return;

  // Try to read a single coil as heartbeat
  uint8_t status;
  int rc = modbus_read_bits(ctx_, 0x0000, 1, &status);

  if (rc == -1)
  {
    if (connected_)
    {
      connected_ = false;
      RCLCPP_WARN(getLogger(), "Modbus connection lost: %s", modbus_strerror(errno));
      onConnectionLost_();
      if (postConnectionLostEvent_) postConnectionLostEvent_();
    }
  }
  else
  {
    if (!connected_)
    {
      connected_ = true;
      RCLCPP_INFO(getLogger(), "Modbus connection restored");
      onConnectionRestored_();
      if (postConnectionRestoredEvent_) postConnectionRestoredEvent_();
    }
  }
}
```

---

## package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cl_modbus_tcp_relay</name>
  <version>1.0.0</version>
  <description>SMACC2 client for Modbus TCP relay control (8-channel)</description>
  <maintainer email="your-email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>pkg-config</buildtool_depend>

  <depend>smacc2</depend>

  <!-- System dependency: sudo apt install libmodbus-dev -->
  <build_depend>libmodbus-dev</build_depend>
  <exec_depend>libmodbus5</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## CMakeLists.txt

**Prerequisites**: Install libmodbus via `sudo apt install libmodbus-dev`

```cmake
cmake_minimum_required(VERSION 3.5)
project(cl_modbus_tcp_relay)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(smacc2 REQUIRED)
find_package(PkgConfig REQUIRED)

# Find system-installed libmodbus
pkg_check_modules(MODBUS REQUIRED libmodbus)

include_directories(
  include
  ${smacc2_INCLUDE_DIRS}
  ${MODBUS_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/cl_modbus_tcp_relay/cl_modbus_tcp_relay.cpp
  src/cl_modbus_tcp_relay/components/cp_modbus_connection.cpp
  src/cl_modbus_tcp_relay/components/cp_modbus_relay.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${smacc2_LIBRARIES}
  ${MODBUS_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} smacc2)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)

ament_package()
```

---

## Usage Example

### YAML Configuration File

```yaml
# sm_relay_test/config/sm_relay_test_config.yaml
sm_relay_test:
  ros__parameters:
    modbus_relay:
      ip_address: "192.168.1.254"
      port: 502
      slave_id: 1
      heartbeat_interval_ms: 1000
      connect_on_init: true
```

### In Orthogonal Definition

```cpp
class OrRelay : public smacc2::Orthogonal<OrRelay>
{
public:
  void onInitialize() override
  {
    // No constructor params - config loaded from YAML
    auto relay_client = this->createClient<cl_modbus_tcp_relay::ClModbusTcpRelay>();
  }
};
```

### In State Definition

```cpp
struct StActivateRelay : smacc2::SmaccState<StActivateRelay, SmRelayTest>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<EvCbSuccess<CbRelayOn, OrRelay>, StNextState>,
    smacc2::Transition<EvCbFailure<CbRelayOn, OrRelay>, StError>,
    smacc2::Transition<EvConnectionLost<CpModbusConnection, OrRelay>, StReconnect>
  >;

  static void staticConfigure()
  {
    configure_orthogonal<OrRelay, cl_modbus_tcp_relay::CbRelayOn>(1);  // Turn on channel 1
  }
};
```

### In Launch File

```python
# sm_relay_test/launch/sm_relay_test.launch.py
def generate_launch_description():
    # Load config file
    config_file = os.path.join(
        get_package_share_directory('sm_relay_test'),
        'config',
        'sm_relay_test_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='sm_relay_test',
            executable='sm_relay_test_node',
            name='sm_relay_test',
            output='screen',
            parameters=[config_file]
        )
    ])
```

---

## mbpoll CLI Reference

For manual testing and debugging:

```bash
# Read all 8 coil states
mbpoll -m tcp -a 1 -t 0 -r 1 -c 8 192.168.1.254

# Write single coil ON (channel 1)
mbpoll -m tcp -a 1 -t 0 -r 1 192.168.1.254 1

# Write single coil OFF (channel 1)
mbpoll -m tcp -a 1 -t 0 -r 1 192.168.1.254 0

# Write all coils ON
mbpoll -m tcp -a 1 -t 0 -r 1 -c 8 192.168.1.254 1 1 1 1 1 1 1 1
```

---

## Implementation Phases (Incremental Build & Test)

### Phase 1: Package Skeleton & Minimal Client (COMPILES)
**Goal:** Create package structure that compiles with empty client.

Files to create:
- `cl_modbus_tcp_relay/package.xml`
- `cl_modbus_tcp_relay/CMakeLists.txt`
- `cl_modbus_tcp_relay/include/cl_modbus_tcp_relay/cl_modbus_tcp_relay.hpp` (minimal)
- `cl_modbus_tcp_relay/src/cl_modbus_tcp_relay/cl_modbus_tcp_relay.cpp` (minimal)

**Build command:** `colcon build --packages-select cl_modbus_tcp_relay`

---

### Phase 2: CpModbusConnection Component (COMPILES)
**Goal:** Connection management with libmodbus that compiles and loads config.

Files to create:
- `cl_modbus_tcp_relay/include/cl_modbus_tcp_relay/components/cp_modbus_connection.hpp`
- `cl_modbus_tcp_relay/src/cl_modbus_tcp_relay/components/cp_modbus_connection.cpp`

Update:
- `cl_modbus_tcp_relay.hpp` to create CpModbusConnection in onComponentInitialization()

**Build command:** `colcon build --packages-select cl_modbus_tcp_relay`

---

### Phase 3: Test State Machine - Connection Only (RUNS)
**Goal:** Test state machine that connects to relay and monitors heartbeat.

Create new package: `sm_modbus_tcp_relay_test_1`

Files to create:
- `sm_modbus_tcp_relay_test_1/package.xml`
- `sm_modbus_tcp_relay_test_1/CMakeLists.txt`
- `sm_modbus_tcp_relay_test_1/config/sm_modbus_tcp_relay_test_1_config.yaml`
- `sm_modbus_tcp_relay_test_1/launch/sm_modbus_tcp_relay_test_1.launch.py`
- `sm_modbus_tcp_relay_test_1/include/sm_modbus_tcp_relay_test_1/sm_modbus_tcp_relay_test_1.hpp`
- `sm_modbus_tcp_relay_test_1/include/sm_modbus_tcp_relay_test_1/orthogonals/or_relay.hpp`
- `sm_modbus_tcp_relay_test_1/include/sm_modbus_tcp_relay_test_1/states/st_connect.hpp`
- `sm_modbus_tcp_relay_test_1/include/sm_modbus_tcp_relay_test_1/states/st_connected.hpp`
- `sm_modbus_tcp_relay_test_1/src/sm_modbus_tcp_relay_test_1/sm_modbus_tcp_relay_test_1.cpp`

**Build command:** `colcon build --packages-select cl_modbus_tcp_relay sm_modbus_tcp_relay_test_1`
**Run command:** `ros2 launch sm_modbus_tcp_relay_test_1 sm_modbus_tcp_relay_test_1.launch.py`

---

### Phase 4: CpModbusRelay Component (COMPILES)
**Goal:** Add relay read/write operations.

Files to create:
- `cl_modbus_tcp_relay/include/cl_modbus_tcp_relay/components/cp_modbus_relay.hpp`
- `cl_modbus_tcp_relay/src/cl_modbus_tcp_relay/components/cp_modbus_relay.cpp`

Update:
- `cl_modbus_tcp_relay.hpp` to create CpModbusRelay in onComponentInitialization()

**Build command:** `colcon build --packages-select cl_modbus_tcp_relay`

---

### Phase 5: CbRelayOn/CbRelayOff Behaviors (RUNS)
**Goal:** Basic on/off behaviors with test states.

Files to create:
- `cl_modbus_tcp_relay/include/cl_modbus_tcp_relay/client_behaviors/cb_relay_on.hpp`
- `cl_modbus_tcp_relay/include/cl_modbus_tcp_relay/client_behaviors/cb_relay_off.hpp`

Update test state machine:
- Add `st_relay_on.hpp` - turns on channel 1
- Add `st_relay_off.hpp` - turns off channel 1
- Add transitions between states

**Build command:** `colcon build --packages-select cl_modbus_tcp_relay sm_modbus_tcp_relay_test_1`
**Run command:** `ros2 launch sm_modbus_tcp_relay_test_1 sm_modbus_tcp_relay_test_1.launch.py`

---

### Phase 6: CbAllRelaysOn/CbAllRelaysOff (RUNS)
**Goal:** Batch relay operations.

Files to create:
- `cl_modbus_tcp_relay/include/cl_modbus_tcp_relay/client_behaviors/cb_all_relays_on.hpp`
- `cl_modbus_tcp_relay/include/cl_modbus_tcp_relay/client_behaviors/cb_all_relays_off.hpp`

Update test state machine:
- Add test states for all-on/all-off operations

**Build command:** `colcon build --packages-select cl_modbus_tcp_relay sm_modbus_tcp_relay_test_1`

---

### Phase 7: CbRelayStatus Behavior (RUNS)
**Goal:** Status reading behavior.

Files to create:
- `cl_modbus_tcp_relay/include/cl_modbus_tcp_relay/client_behaviors/cb_relay_status.hpp`

Update test state machine:
- Add status monitoring state

**Build command:** `colcon build --packages-select cl_modbus_tcp_relay sm_modbus_tcp_relay_test_1`

---

### Phase 8: Documentation & README (COMPLETE)
**Goal:** Complete documentation.

Files to create:
- `cl_modbus_tcp_relay/README.md`
- `sm_modbus_tcp_relay_test_1/README.md`

---

## Test State Machine: sm_modbus_tcp_relay_test_1

### State Machine Design

```
┌─────────────────┐
│   StConnect     │ ── EvCbSuccess ──► StRelayOn
│  (wait connect) │ ── EvConnectionLost ──► StConnect (retry)
└─────────────────┘
        │
        ▼
┌─────────────────┐
│   StRelayOn     │ ── EvCbSuccess ──► StWait1
│  (channel 1 ON) │ ── EvCbFailure ──► StError
└─────────────────┘
        │
        ▼
┌─────────────────┐
│    StWait1      │ ── EvTimer ──► StRelayOff
│  (2 sec delay)  │
└─────────────────┘
        │
        ▼
┌─────────────────┐
│   StRelayOff    │ ── EvCbSuccess ──► StWait2
│  (channel 1 OFF)│ ── EvCbFailure ──► StError
└─────────────────┘
        │
        ▼
┌─────────────────┐
│    StWait2      │ ── EvTimer ──► StAllOn
│  (2 sec delay)  │
└─────────────────┘
        │
        ▼
┌─────────────────┐
│    StAllOn      │ ── EvCbSuccess ──► StWait3
│  (all channels) │
└─────────────────┘
        │
        ▼
┌─────────────────┐
│    StWait3      │ ── EvTimer ──► StAllOff
│  (2 sec delay)  │
└─────────────────┘
        │
        ▼
┌─────────────────┐
│    StAllOff     │ ── EvCbSuccess ──► StComplete
│  (all channels) │
└─────────────────┘
        │
        ▼
┌─────────────────┐
│   StComplete    │ ── (logs success, stays) ───►
│  (test passed)  │
└─────────────────┘
```

### Test State Machine Config

```yaml
# sm_modbus_tcp_relay_test_1/config/sm_modbus_tcp_relay_test_1_config.yaml
sm_modbus_tcp_relay_test_1:
  ros__parameters:
    # Modbus relay configuration
    modbus_relay:
      ip_address: "192.168.1.254"
      port: 502
      slave_id: 1
      heartbeat_interval_ms: 1000
      connect_on_init: true

    # Timer configuration for wait states
    wait_duration_ms: 2000
```

### Test State Machine Package Structure

```
smacc2_sm_reference_library/sm_modbus_tcp_relay_test_1/
├── config/
│   └── sm_modbus_tcp_relay_test_1_config.yaml
├── include/sm_modbus_tcp_relay_test_1/
│   ├── orthogonals/
│   │   ├── or_relay.hpp
│   │   └── or_timer.hpp
│   ├── states/
│   │   ├── st_connect.hpp
│   │   ├── st_relay_on.hpp
│   │   ├── st_relay_off.hpp
│   │   ├── st_all_on.hpp
│   │   ├── st_all_off.hpp
│   │   ├── st_wait.hpp
│   │   ├── st_complete.hpp
│   │   └── st_error.hpp
│   └── sm_modbus_tcp_relay_test_1.hpp
├── launch/
│   └── sm_modbus_tcp_relay_test_1.launch.py
├── src/sm_modbus_tcp_relay_test_1/
│   └── sm_modbus_tcp_relay_test_1.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Files to Create

### Client Library: `src/SMACC2/smacc2_client_library/cl_modbus_tcp_relay/`

| File | Purpose |
|------|---------|
| `package.xml` | ROS2 package manifest |
| `CMakeLists.txt` | Build configuration with libmodbus |
| `include/cl_modbus_tcp_relay/cl_modbus_tcp_relay.hpp` | Main client |
| `include/cl_modbus_tcp_relay/components/cp_modbus_connection.hpp` | Connection component |
| `include/cl_modbus_tcp_relay/components/cp_modbus_relay.hpp` | Relay control component |
| `include/cl_modbus_tcp_relay/client_behaviors/cb_relay_on.hpp` | Turn on behavior |
| `include/cl_modbus_tcp_relay/client_behaviors/cb_relay_off.hpp` | Turn off behavior |
| `include/cl_modbus_tcp_relay/client_behaviors/cb_all_relays_on.hpp` | All on behavior |
| `include/cl_modbus_tcp_relay/client_behaviors/cb_all_relays_off.hpp` | All off behavior |
| `include/cl_modbus_tcp_relay/client_behaviors/cb_relay_status.hpp` | Status read behavior |
| `src/cl_modbus_tcp_relay/cl_modbus_tcp_relay.cpp` | Client implementation |
| `src/cl_modbus_tcp_relay/components/cp_modbus_connection.cpp` | Connection impl |
| `src/cl_modbus_tcp_relay/components/cp_modbus_relay.cpp` | Relay control impl |
| `README.md` | Documentation with mbpoll examples |

### Test State Machine: `src/SMACC2/smacc2_sm_reference_library/sm_modbus_tcp_relay_test_1/`

| File | Purpose |
|------|---------|
| `package.xml` | ROS2 package manifest |
| `CMakeLists.txt` | Build configuration |
| `config/sm_modbus_tcp_relay_test_1_config.yaml` | Modbus & timer config |
| `launch/sm_modbus_tcp_relay_test_1.launch.py` | Launch file |
| `include/sm_modbus_tcp_relay_test_1/sm_modbus_tcp_relay_test_1.hpp` | Main SM header |
| `include/sm_modbus_tcp_relay_test_1/orthogonals/or_relay.hpp` | Relay orthogonal |
| `include/sm_modbus_tcp_relay_test_1/orthogonals/or_timer.hpp` | Timer orthogonal |
| `include/sm_modbus_tcp_relay_test_1/states/st_connect.hpp` | Initial connection state |
| `include/sm_modbus_tcp_relay_test_1/states/st_relay_on.hpp` | Turn on channel 1 |
| `include/sm_modbus_tcp_relay_test_1/states/st_relay_off.hpp` | Turn off channel 1 |
| `include/sm_modbus_tcp_relay_test_1/states/st_all_on.hpp` | Turn on all channels |
| `include/sm_modbus_tcp_relay_test_1/states/st_all_off.hpp` | Turn off all channels |
| `include/sm_modbus_tcp_relay_test_1/states/st_wait.hpp` | Timer wait state (template) |
| `include/sm_modbus_tcp_relay_test_1/states/st_complete.hpp` | Test complete state |
| `include/sm_modbus_tcp_relay_test_1/states/st_error.hpp` | Error handling state |
| `src/sm_modbus_tcp_relay_test_1/sm_modbus_tcp_relay_test_1.cpp` | Main SM source |
| `README.md` | Test instructions |

---

## Reference Files (for implementation patterns)

- `src/SMACC2/smacc2_client_library/cl_http/include/cl_http/cl_http.hpp`
- `src/SMACC2/smacc2_client_library/cl_http/include/cl_http/components/cp_http_connection_manager.hpp`
- `src/SMACC2/smacc2_client_library/cl_lifecycle_node/include/cl_lifecycle_node/client_behaviors/cb_activate.hpp`
- `src/SMACC2/smacc2_client_library/cl_ros2_timer/include/cl_ros2_timer/components/cp_timer_listener_1.hpp`
- `src/libmodbus/src/modbus.h`
- `src/libmodbus/src/modbus-tcp.h`
