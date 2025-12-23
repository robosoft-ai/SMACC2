# cl_modbus_tcp_relay

SMACC2 client library for controlling Modbus TCP relay boards, specifically designed for the Waveshare 8-Channel POE ETH Relay.

## Overview

This client library provides a SMACC2-compatible interface for controlling relay boards via Modbus TCP protocol. It uses the libmodbus C library for communication and follows SMACC2's pure component-based architecture.

## Target Hardware

- **Device**: Waveshare 8-Channel POE ETH Relay (or compatible)
- **Protocol**: Modbus TCP
- **Default IP**: 192.168.1.254
- **Default Port**: 502
- **Slave ID**: 0x01
- **Coil Addresses**: 0x0000-0x0007 (channels 1-8)

## Dependencies

### System Dependencies

```bash
sudo apt install libmodbus-dev
```

### ROS2 Dependencies

- smacc2

## Architecture

### Client: ClModbusTcpRelay

Pure orchestrator that creates and configures components during initialization. Configuration is loaded from YAML parameters.

### Components

#### CpModbusConnection

Manages libmodbus context lifecycle, TCP connection, and heartbeat monitoring.

**Responsibilities:**
- Create/destroy modbus_t context
- Manage TCP connection state
- Periodic heartbeat via ISmaccUpdatable
- Emit connection state change signals
- Thread-safe connection access via mutex

**Signals:**
- `onConnectionLost_` - Emitted when heartbeat fails
- `onConnectionRestored_` - Emitted when reconnection succeeds
- `onConnectionError_` - Emitted on connection errors

#### CpModbusRelay

Handles Modbus coil read/write operations for the 8-channel relay.

**Methods:**
- `writeCoil(int channel, bool state)` - Write single channel (1-8)
- `writeAllCoils(bool state)` - Write all channels ON or OFF
- `writeAllCoils(uint8_t mask)` - Write all channels with bitmask
- `readCoil(int channel)` - Read single channel state
- `readAllCoils()` - Read all channel states

### Client Behaviors

| Behavior | Description |
|----------|-------------|
| `CbRelayOn` | Turn on a specific relay channel (1-8) |
| `CbRelayOff` | Turn off a specific relay channel (1-8) |
| `CbAllRelaysOn` | Turn on all 8 relay channels |
| `CbAllRelaysOff` | Turn off all 8 relay channels |
| `CbRelayStatus` | Read the status of relay channels |

### Events

```cpp
// Connection events (source: CpModbusConnection)
EvConnectionLost<CpModbusConnection, OrRelay>
EvConnectionRestored<CpModbusConnection, OrRelay>

// Relay operation events (source: CpModbusRelay)
EvRelayWriteSuccess<CpModbusRelay, OrRelay>
EvRelayWriteFailure<CpModbusRelay, OrRelay>
```

## Configuration

Configuration is loaded from ROS2 parameters (typically via YAML config file):

```yaml
your_state_machine:
  ros__parameters:
    modbus_relay:
      ip_address: "192.168.1.254"    # Relay board IP address
      port: 502                       # Modbus TCP port
      slave_id: 1                     # Modbus slave ID
      heartbeat_interval_ms: 1000     # Heartbeat check interval
      connect_on_init: true           # Auto-connect on initialization
```

### Default Values

| Parameter | Default |
|-----------|---------|
| `ip_address` | "192.168.1.254" |
| `port` | 502 |
| `slave_id` | 1 |
| `heartbeat_interval_ms` | 1000 |
| `connect_on_init` | true |

## Usage

### Orthogonal Definition

```cpp
#include <cl_modbus_tcp_relay/cl_modbus_tcp_relay.hpp>

class OrRelay : public smacc2::Orthogonal<OrRelay>
{
public:
  void onInitialize() override
  {
    // Configuration loaded from YAML parameters
    auto relay_client = this->createClient<cl_modbus_tcp_relay::ClModbusTcpRelay>();
  }
};
```

### State Definition

```cpp
#include <cl_modbus_tcp_relay/client_behaviors/cb_relay_on.hpp>
#include <cl_modbus_tcp_relay/client_behaviors/cb_relay_off.hpp>

struct StActivateRelay : smacc2::SmaccState<StActivateRelay, SmExample>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbRelayOn, OrRelay>, StNextState>,
    Transition<EvCbFailure<CbRelayOn, OrRelay>, StError>,
    Transition<EvConnectionLost<CpModbusConnection, OrRelay>, StReconnect>
  > reactions;

  static void staticConfigure()
  {
    // Turn on channel 1
    configure_orthogonal<OrRelay, cl_modbus_tcp_relay::CbRelayOn>(1);
  }
};
```

### Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('your_state_machine'),
        'config',
        'your_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='your_state_machine',
            executable='your_state_machine_node',
            name='your_state_machine',
            output='screen',
            parameters=[config_file]
        )
    ])
```

## Manual Testing with mbpoll

For debugging and manual testing, you can use the `mbpoll` command-line tool:

```bash
# Install mbpoll
sudo apt install mbpoll

# Read all 8 coil states
mbpoll -m tcp -a 1 -t 0 -r 1 -c 8 192.168.1.254

# Write single coil ON (channel 1)
mbpoll -m tcp -a 1 -t 0 -r 1 192.168.1.254 1

# Write single coil OFF (channel 1)
mbpoll -m tcp -a 1 -t 0 -r 1 192.168.1.254 0

# Write all coils ON
mbpoll -m tcp -a 1 -t 0 -r 1 -c 8 192.168.1.254 1 1 1 1 1 1 1 1

# Write all coils OFF
mbpoll -m tcp -a 1 -t 0 -r 1 -c 8 192.168.1.254 0 0 0 0 0 0 0 0
```

### mbpoll Options Reference

| Option | Description |
|--------|-------------|
| `-m tcp` | Use Modbus TCP protocol |
| `-a 1` | Slave address (1) |
| `-t 0` | Data type: coil (0) |
| `-r 1` | Starting reference (1 = address 0x0000) |
| `-c 8` | Number of coils to read/write |

## Modbus Protocol Reference

| Operation | Function Code | Address Range |
|-----------|---------------|---------------|
| Read Coils | 0x01 | 0x0000-0x0007 |
| Write Single Coil | 0x05 | 0x0000-0x0007 |
| Write Multiple Coils | 0x0F | 0x0000 (8 coils) |

### Coil Values

- ON: 0xFF00 (function 0x05) or 1 (function 0x0F)
- OFF: 0x0000

## Building

```bash
# From workspace root
colcon build --packages-select cl_modbus_tcp_relay
```

## Test State Machine

See `sm_modbus_tcp_relay_test_1` for a complete test state machine that exercises all behaviors.

## License

Apache-2.0
