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
- Tested targets: ROS 2 Humble and Jazzy on Linux
- Serial transport: POSIX `termios`, no Python runtime dependency
- Protocol source: reverse-engineered `fnirsi-dps-150` WebSerial implementation

## Build

From a ROS 2 workspace:

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Raptor-zip/fnirsi_dps150_ros2_driver.git fnirsi_dps150_driver
cd ~/ros2_ws
source /opt/ros/humble/setup.bash   # or: source /opt/ros/jazzy/setup.bash
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

Log out and back in after changing group membership. If you see
`Permission denied` on `/dev/ttyACM0` or `/dev/fnirsi_dps150` immediately after
running `usermod`, the current shell hasn't picked up the new group yet — run
`newgrp dialout` to start a sub-shell with the group applied, or just log out
and back in.

For production rigs, you can also install a udev rule that creates a stable
`/dev/fnirsi_dps150` symlink (run from the workspace root, e.g. `~/ros2_ws`):

```sh
sudo cp src/fnirsi_dps150_driver/docs/99-fnirsi-dps150.rules /etc/udev/rules.d/
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

### Feedback / state rate

The node publishes `/dps150/state` on **every telemetry frame received from the
device**, so the topic rate tracks the device's own streaming rate rather than a
fixed timer. `state_publish_period_ms` (default `1000`) only acts as a keepalive
lower bound that re-publishes the last snapshot when no frames arrive.

Measured on the real device (DPS-150 on `/dev/ttyACM0` @ 115200 baud, ROS 2 Humble,
~21.6 s window):

| Source | Measured rate |
|---|---:|
| Device telemetry frames (`frames_received`) | ~9.3 Hz |
| `/dps150/state` topic | ~9.3 Hz |

So `/dps150/state` is published at roughly **9–14 Hz** (it varies with the device
stream). You can confirm it yourself with:

```sh
ros2 topic hz /dps150/state
```

#### Effective value-update rate per field

The topic rate is **not** the rate at which the analog readings actually change.
The DPS-150 refreshes its measured values more slowly than it streams frames, so
each value typically repeats 7–8 times before it updates. Measured on the real
device (12 V / 0.1 A output enabled, ~30 s window, topic ~13 Hz), counting how
often each field's value actually changes:

| Field | Effective update rate | Avg. repeats per value |
|---|---:|---:|
| `output_current` | ~1.8 Hz | 7.2 |
| `output_power` | ~1.8 Hz | 7.2 |
| `temperature` | ~1.8 Hz | 7.3 |
| `input_voltage` | ~1.6 Hz | 8.2 |
| `output_voltage` | n/a* | n/a* |

\* `output_voltage` stayed pinned at the regulated 12.0 V during the test, so its
refresh rate could not be inferred from value changes.

Practical takeaway: treat the **real telemetry refresh as ~2 Hz** even though the
topic publishes at ~13 Hz. If you only care about new readings, deduplicate on the
subscriber side (compare against the last value) instead of processing every
message, and use the `connected` field plus the `/diagnostics` `last_state_age_sec`
value to judge freshness — the message `stamp` is set at publish time and always
looks current even when the underlying data is stale.

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
