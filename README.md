# fnirsi_dps150_driver

ROS 2 C++ driver for the FNIRSI DPS-150 programmable DC power supply.

This package talks to the DPS-150 over its USB CDC serial interface. It exposes
device telemetry as a ROS 2 topic and provides services/subscriptions for setpoints,
presets, protection thresholds, UI settings, metering, and output enable.

The node does not enable the power output at startup. Output RUN/STOP changes only
happen through `enable_output` or the `output_enable` topic.

## Status

- ROS 2 package: `ament_cmake`
- Implementation language: C++17
- Tested target: ROS 2 Humble on Linux
- Serial transport: POSIX `termios`, no Python runtime dependency
- Protocol source: reverse-engineered `fnirsi-dps-150` WebSerial implementation

## Build

From a ROS 2 workspace:

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Raptor-zip/fnirsi_dps150_ros2_driver.git fnirsi_dps150_driver
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fnirsi_dps150_driver
source install/setup.bash
```

## Device Permissions

The DPS-150 normally appears as `/dev/ttyACM0` or under `/dev/serial/by-id`.
Using the stable `/dev/serial/by-id/...` path is recommended.

Your user must have serial permission, typically through the `dialout` group:

```sh
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership.

For production rigs, you can also install a udev rule that creates a stable
`/dev/fnirsi_dps150` symlink:

```sh
sudo cp docs/99-fnirsi-dps150.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then start the node with:

```sh
ros2 run fnirsi_dps150_driver dps150_node --ros-args -p port:=/dev/fnirsi_dps150
```

## Run

Start with `ros2 run`:

```sh
ros2 run fnirsi_dps150_driver dps150_node --ros-args -p port:=/dev/ttyACM0
```

Using the stable `/dev/serial/by-id/...` path is recommended when the device is
used on a robot or test rig:

```sh
ros2 run fnirsi_dps150_driver dps150_node --ros-args -p port:=/dev/serial/by-id/usb-Artery_AT32_Virtual_Com_Port_9605ACB81976-if00
```

The same port parameter is available from the XML launch file:

```sh
ros2 launch fnirsi_dps150_driver dps150.launch.xml port:=/dev/ttyACM0
ros2 launch fnirsi_dps150_driver dps150.launch.xml port:=/dev/serial/by-id/usb-Artery_AT32_Virtual_Com_Port_9605ACB81976-if00
```

Read the state:

```sh
ros2 topic echo /dps150/state
```

Request a full state refresh:

```sh
ros2 service call /dps150/refresh std_srvs/srv/Trigger {}
```

Diagnostics are published on the standard ROS topic:

```sh
ros2 topic echo /diagnostics
```

Useful runtime parameters:

| Parameter | Default | Purpose |
|---|---:|---|
| `auto_reconnect` | `true` | Reopen the serial port after USB disconnects or read errors |
| `reconnect_period_ms` | `2000` | Reconnect attempt interval |
| `enforce_setpoint_limits` | `true` | Reject setpoints above known device/protection limits |
| `allow_output_enable_during_protection` | `false` | Allow RUN while the device reports OVP/OCP/OPP/OTP/LVP/REP |
| `auto_refresh_after_write` | `true` | Request a full state refresh after writes |

## Control

Set voltage and current without enabling output:

```sh
ros2 service call /dps150/set_voltage fnirsi_dps150_driver/srv/SetFloat32 "{value: 5.0, refresh: true}"
ros2 service call /dps150/set_current fnirsi_dps150_driver/srv/SetFloat32 "{value: 1.0, refresh: true}"
```

Enable or disable output explicitly:

```sh
ros2 service call /dps150/enable_output std_srvs/srv/SetBool "{data: true}"
ros2 service call /dps150/enable_output std_srvs/srv/SetBool "{data: false}"
```

The same controls are also available as simple topics:

```sh
ros2 topic pub --once /dps150/set_voltage std_msgs/msg/Float32 "{data: 5.0}"
ros2 topic pub --once /dps150/set_current std_msgs/msg/Float32 "{data: 1.0}"
ros2 topic pub --once /dps150/output_enable std_msgs/msg/Bool "{data: false}"
```

## Services

| Service | Type | Purpose |
|---|---|---|
| `/dps150/connect` | `std_srvs/srv/Trigger` | Open the serial port and start a session |
| `/dps150/disconnect` | `std_srvs/srv/Trigger` | Disable the session and close the port |
| `/dps150/refresh` | `std_srvs/srv/Trigger` | Request a full memory dump |
| `/dps150/enable_output` | `std_srvs/srv/SetBool` | RUN/STOP output |
| `/dps150/enable_metering` | `std_srvs/srv/SetBool` | Enable capacity/energy telemetry |
| `/dps150/set_voltage` | `fnirsi_dps150_driver/srv/SetFloat32` | Set active voltage in volts |
| `/dps150/set_current` | `fnirsi_dps150_driver/srv/SetFloat32` | Set active current limit in amperes |
| `/dps150/set_brightness` | `fnirsi_dps150_driver/srv/SetUInt8` | Set display brightness |
| `/dps150/set_volume` | `fnirsi_dps150_driver/srv/SetUInt8` | Set buzzer volume |
| `/dps150/set_preset` | `fnirsi_dps150_driver/srv/SetPreset` | Write M1-M6 preset values |
| `/dps150/select_preset` | `fnirsi_dps150_driver/srv/SelectPreset` | Copy an already-known preset into C1/C2 |
| `/dps150/set_protection` | `fnirsi_dps150_driver/srv/SetProtection` | Set OVP/OCP/OPP/OTP/LVP threshold |

## Safety Notes

- Treat `enable_output` as a real power operation.
- Verify voltage/current setpoints before enabling output.
- Use a stable serial path on production rigs.
- Keep `output_enable` under explicit operator or test-sequence control.

## Acknowledgements

The protocol implementation is based on the MIT-licensed reverse engineering work in
`fnirsi-dps-150` by cho45. See `THIRD_PARTY_NOTICES.md`.

## License

MIT
