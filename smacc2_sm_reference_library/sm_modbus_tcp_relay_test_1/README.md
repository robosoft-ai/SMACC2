# sm_modbus_tcp_relay_test_1

Test state machine for the `cl_modbus_tcp_relay` SMACC2 client library. This state machine exercises all relay behaviors to verify proper operation.

## Overview

This state machine connects to a Waveshare 8-Channel POE ETH Relay via Modbus TCP and tests all available client behaviors:

1. **CbRelayOn** - Turn on a single channel
2. **CbRelayOff** - Turn off a single channel
3. **CbAllRelaysOn** - Turn on all channels
4. **CbAllRelaysOff** - Turn off all channels
5. **CbRelayStatus** - Read channel status

## State Flow

```
StConnect ─► StConnected ─► StRelayOn ─► StRelayOff ─► StAllOn ─► StAllOff ─► StReadStatus ─► StComplete
    ▲                           │             │           │           │            │
    └───────────────────────────┴─────────────┴───────────┴───────────┴────────────┘
                              (EvConnectionLost returns to StConnect)
```

### State Descriptions

| State | Description |
|-------|-------------|
| `StConnect` | Waits for Modbus TCP connection to establish |
| `StConnected` | Connection established, transitions to relay tests |
| `StRelayOn` | Turns ON channel 1 |
| `StRelayOff` | Turns OFF channel 1 |
| `StAllOn` | Turns ON all 8 channels |
| `StAllOff` | Turns OFF all 8 channels |
| `StReadStatus` | Reads status of all channels |
| `StComplete` | Test complete, displays results |

## Configuration

Edit `config/sm_modbus_tcp_relay_test_1_config.yaml`:

```yaml
sm_modbus_tcp_relay_test_1:
  ros__parameters:
    signal_detector_loop_freq: 20

    modbus_relay:
      ip_address: "192.168.1.254"    # Your relay board IP
      port: 502                       # Modbus TCP port
      slave_id: 1                     # Modbus slave ID
      heartbeat_interval_ms: 1000     # Connection monitoring interval
      connect_on_init: true           # Auto-connect on startup
```

## Dependencies

- `cl_modbus_tcp_relay` - The Modbus TCP relay client library
- `smacc2` - SMACC2 framework

### System Dependencies

```bash
sudo apt install libmodbus-dev
```

## Building

```bash
# Build both packages
colcon build --packages-select cl_modbus_tcp_relay sm_modbus_tcp_relay_test_1
```

## Running

### Prerequisites

1. Ensure the relay board is powered and connected to the network
2. Verify network connectivity to the relay (default: 192.168.1.254)

### Manual Connectivity Test (Optional)

Before running the state machine, you can verify connectivity with mbpoll:

```bash
# Install mbpoll if not already installed
sudo apt install mbpoll

# Test connection by reading all coil states
mbpoll -m tcp -a 1 -t 0 -r 1 -c 8 192.168.1.254
```

### Launch the Test

```bash
source install/setup.bash
ros2 launch sm_modbus_tcp_relay_test_1 sm_modbus_tcp_relay_test_1.launch.py
```

### Expected Output

```
[INFO] [SmModbusTcpRelayTest1]: onInitialize
[INFO] [CpModbusConnection]: Config: 192.168.1.254:502 (slave=1, heartbeat=1000ms)
[INFO] [CpModbusConnection]: Connected to Modbus TCP device
[INFO] [StConnect]: Waiting for Modbus connection...
[INFO] [StConnect]: Connection established!
[INFO] [StConnected]: Connected! Running relay tests...
[INFO] [StRelayOn]: Turning ON relay channel 1...
[INFO] [CpModbusRelay]: Write coil 1 = ON: success
[INFO] [StRelayOff]: Turning OFF relay channel 1...
[INFO] [CpModbusRelay]: Write coil 1 = OFF: success
[INFO] [StAllOn]: Turning ON all relays...
[INFO] [CpModbusRelay]: Write all coils ON: success
[INFO] [StAllOff]: Turning OFF all relays...
[INFO] [CpModbusRelay]: Write all coils OFF: success
[INFO] [StReadStatus]: Reading relay status...
[INFO] [CpModbusRelay]: Read all coils: 0x00
[INFO] ========================================
[INFO]   cl_modbus_tcp_relay TEST COMPLETE!
[INFO] ========================================
[INFO] Test results:
[INFO]   [PASS] CbRelayOn(1) - Single channel ON
[INFO]   [PASS] CbRelayOff(1) - Single channel OFF
[INFO]   [PASS] CbAllRelaysOn - All channels ON
[INFO]   [PASS] CbAllRelaysOff - All channels OFF
[INFO]   [PASS] CbRelayStatus - Read all statuses
[INFO] State machine will remain in StComplete.
[INFO] Press Ctrl-C to exit.
[INFO] ========================================
```

## Monitoring

### State Machine Status

```bash
ros2 topic echo /sm_modbus_tcp_relay_test_1/smacc/status
```

### State Transitions

```bash
ros2 topic echo /sm_modbus_tcp_relay_test_1/smacc/transition_log
```

## Troubleshooting

### Connection Failed

If the state machine cannot connect:

1. **Verify network connectivity**:
   ```bash
   ping 192.168.1.254
   ```

2. **Check firewall settings**:
   ```bash
   # Port 502 must be accessible
   nc -zv 192.168.1.254 502
   ```

3. **Test with mbpoll**:
   ```bash
   mbpoll -m tcp -a 1 -t 0 -r 1 -c 8 192.168.1.254
   ```

4. **Check IP address in config**: Ensure the `ip_address` parameter matches your relay board

### Connection Lost During Test

The state machine handles connection loss automatically:
- If connection is lost in any state, it transitions back to `StConnect`
- The heartbeat mechanism monitors connection health every 1000ms (configurable)

### Relay Not Responding

1. **Verify slave ID**: Some relay boards use different slave IDs (0, 1, 254, etc.)
2. **Check power supply**: Ensure the relay board has adequate power
3. **Reset the relay board**: Power cycle the device

## Files

```
sm_modbus_tcp_relay_test_1/
├── config/
│   └── sm_modbus_tcp_relay_test_1_config.yaml
├── include/sm_modbus_tcp_relay_test_1/
│   ├── orthogonals/
│   │   └── or_relay.hpp
│   ├── states/
│   │   ├── st_connect.hpp
│   │   ├── st_connected.hpp
│   │   ├── st_relay_on.hpp
│   │   ├── st_relay_off.hpp
│   │   ├── st_all_on.hpp
│   │   ├── st_all_off.hpp
│   │   ├── st_read_status.hpp
│   │   └── st_complete.hpp
│   └── sm_modbus_tcp_relay_test_1.hpp
├── launch/
│   └── sm_modbus_tcp_relay_test_1.launch.py
├── src/sm_modbus_tcp_relay_test_1/
│   └── sm_modbus_tcp_relay_test_1.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

## License

Apache-2.0
