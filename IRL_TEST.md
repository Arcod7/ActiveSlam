# Real-world control test procedure

This is the go/no-go procedure for taking the ActiveSlam controller from
Stonefish to a real BlueROV-class vehicle. It is deliberately conservative.
Any person at the test site may call STOP. A stopped test is a successful safety
decision, not a failed experiment.

## Current readiness and hard limitation

The repository now has a fail-closed ROS body-command gate:

```text
motion controller or keyboard
        │
        ▼
/motion/body_command (normalized Twist)
        │
        ▼
motion_safety_gate
        │
        ▼
/motion/body_command_safe
        │
        ├── heavy_sim_mixer ──► 8 Stonefish thruster values
        │
        └── ardusub_adapter
              ├── manual_control ──► MAVLink MANUAL_CONTROL
              └── local_ned_velocity ──► SET_POSITION_TARGET_LOCAL_NED
```

The gate prevents output unless it is explicitly enabled and both the command
and odometry are fresh. It rejects non-finite, out-of-range, and
multiple-publisher command inputs. It publishes a zero `Twist` continuously
whenever it is not `ACTIVE`. In simulation, `heavy_sim_mixer` is the only node
that converts that gated body demand into eight motor fractions.

The MAVLink adapter is implemented but has not yet been accepted against this
physical vehicle. It validates the safe `Twist` again, monitors ArduSub
heartbeat, arm state and mode, and requires a fresh `ACTIVE` gate status. It
never arms the vehicle or changes its mode. When authority is lost after active
control, it sends a short neutral burst and then stops transmitting so the
pilot's control station can own the pilot-input stream. Any fault during active
control latches a reauthorization requirement: correct the fault, disable the
ROS gate, and explicitly enable it again. A recovered link cannot silently
resume an earlier command.

The gate's safe `Twist` contains normalized demands, not SI velocities. The
`manual_control` backend applies a configurable fraction of the MAVLink pilot
axis range. The `local_ned_velocity` backend applies configurable m/s and rad/s
limits before sending velocity plus yaw-rate fields in
`SET_POSITION_TARGET_LOCAL_NED`.

**Do not remap the Stonefish thruster output directly to real ESC or servo
channels. Do not start autonomous exploration until the adapter, frame signs,
takeover path and ArduSub navigation estimate have passed Phase 3.**

The intended real architecture is:

```text
planner/controller (`/motion/body_command`)
        │ normalized body demand (surge, sway, vertical, yaw)
        ▼
motion safety gate (`/motion/body_command_safe`)
        │
        ▼
MAVLink hardware adapter
        │
        ▼
ArduSub stabilization and frame mixer
        │
        ▼
ESCs and thrusters
```

ArduSub, rather than the exploration node, must own attitude stabilization and
final motor allocation. This experiment uses a BlueROV2 Heavy: configure and
verify ArduSub's eight-motor `Vectored_6DOF` frame. The Stonefish scenario uses
`data/robot/bluerov2_unphy.scn`, also with four horizontal and four vertical
thrusters. The Stonefish array order is not evidence of ArduSub motor numbering;
verify the real motor assignment with the official setup procedure.

For the first wet test use `manual_control` in `ALT_HOLD`, with vertical control
disabled. `local_ned_velocity` requires `GUIDED`, and ArduSub documents Guided
as requiring position and depth. Do not use it underwater until a suitable
horizontal position/velocity source (for example reviewed ROS SLAM ExternalNav
input) is healthy in ArduSub's EKF. Merely having ROS odometry available to the
safety gate does not put that estimate into the ArduSub EKF.

Official references:

- [ArduSub pilot control](https://ardupilot.org/sub/docs/pilot-control.html)
- [ArduSub frame configurations](https://ardupilot.org/sub/docs/sub-frames.html)
- [BlueROV2 software and motor setup](https://bluerobotics.com/learn/bluerov2-software-setup/)
- [BlueOS MAVLink endpoints](https://blueos.cloud/docs/stable/usage/advanced/)
- [ArduSub battery failsafe](https://ardupilot.org/sub/docs/failsafe-battery.html)
- [ArduSub leak failsafe](https://ardupilot.org/sub/docs/internal-leak-failsafe.html)

## People and control authority

Never perform a powered wet test alone. Assign these roles before connecting a
battery:

- **Pilot:** holds the physical joystick and can immediately take control,
  activate motor emergency stop, or disarm.
- **Test director:** reads this checklist, authorizes each transition, and is
  the only person allowed to enable autonomy.
- **Vehicle observer:** watches the vehicle, tether, people, leaks, unusual
  noise, bubbles, and uncontrolled motion instead of watching a laptop.
- **Data operator:** watches ROS, ArduSub and sensor health and records events.
  The director may combine this role in a small team; do not combine pilot and
  vehicle observer.

Agree on one spoken command: **STOP, STOP, STOP**. On hearing it, the pilot uses
the hardware/ArduSub motor emergency stop first. The data operator disables the
ROS gate second. Do not depend on a ROS message as the only emergency stop,
because the reason for stopping may be a failed computer or network.

## Gate operation

The gate starts disabled by default. The project RViz layouts load a **Motion
Safety** panel automatically. Wait for its status to show `DISABLED`, then use:

- **ENABLE MOTION**: shows a confirmation prompt and publishes `true` to
  `/motion/enable`.
- **DISABLE NOW**: immediately publishes `false` to `/motion/enable`.

The panel will not offer Enable while the gate reports a fault. Press Disable,
correct the fault, wait for `DISABLED`, then reconsider enabling. Disable before
closing RViz; an RViz or computer crash is not an emergency-stop mechanism.

Monitor the underlying topics with:

```bash
ros2 topic echo /motion/safety_status
ros2 topic echo /motion/body_command_safe
ros2 topic echo /bluerov2/controller/thruster_setpoints_sim
```

If RViz is unavailable, the equivalent fallback command after the director
completes the relevant go/no-go checklist is:

```bash
ros2 topic pub --once /motion/enable std_msgs/msg/Bool '{data: true}'
```

The fallback disable command is:

```bash
ros2 topic pub --once /motion/enable std_msgs/msg/Bool '{data: false}'
```

The RViz panel controls only the ROS command gate. It does not arm/disarm
ArduSub. For any emergency, the pilot uses the ArduSub/hardware motor stop
first; the data operator presses **DISABLE NOW** second.

Every transition from disabled to enabled discards any earlier command. A new
fresh command must arrive after enabling. The possible states are:

| State | Meaning and required action |
|---|---|
| `ACTIVE` | One valid publisher, fresh command, fresh odometry; motion can pass. |
| `DISABLED` | Explicitly disabled; output is zero. |
| `NO_COMMAND` | Wait for a new controller command after enabling. |
| `STALE_COMMAND` | Controller stopped or communication stalled; investigate. |
| `NO_ODOMETRY` | No pose input has arrived; never bypass this for autonomy. |
| `STALE_ODOMETRY` | Pose estimator stalled; stop and diagnose timestamps/data flow. |
| `INVALID_COMMAND` | A Twist field is NaN/Inf or has magnitude above 1; stop and fix the publisher. |
| `MULTIPLE_COMMAND_SOURCES` | More than one controller/teleop publisher; stop one of them. |

`safety_start_enabled:=true` exists for controlled simulation automation. Do not
use it for a real-vehicle launch.

## Phase 0: software acceptance in simulation

Complete this before booking water time.

- [ ] Build the workspace from a clean shell.
- [ ] Run the full `frontier_slam` test suite.
- [ ] Launch the intended mapper, SLAM and controller configuration with the
      gate disabled.
- [ ] Confirm `/motion/body_command_safe` is a zero Twist and the actuator topic
      contains eight zeros while the controller publishes a non-zero normalized
      Twist on `/motion/body_command`.
- [ ] Click **ENABLE MOTION** in RViz and confirm the expected simulation motion.
- [ ] Click **DISABLE NOW** and confirm zero output and a stopped vehicle.
- [ ] Kill the controller while enabled; output must become zero within the
      command timeout (default 0.5 s).
- [ ] Stop odometry while enabled; output must become zero within the odometry
      timeout (default 0.5 s).
- [ ] Start keyboard teleop while autonomy is publishing; the gate must report
      `MULTIPLE_COMMAND_SOURCES` and output zero.
- [ ] With the normal controller stopped, publish `NaN` and magnitude-above-one
      Twist fields from one test publisher; each
      must produce `INVALID_COMMAND` and zero output.
- [ ] Restart the gate; it must return in `DISABLED`, never resume an old command.
- [ ] Save a rosbag and logs proving each result.

Do at least two complete rehearsals, including the emergency-stop call, using
the same computers and operator roles planned for the wet test.

## Phase 1: vehicle commissioning without ROS authority

Do not run the exploration stack in this phase.

- [ ] Record vehicle serial numbers and current ArduSub, BlueOS and control
      station versions.
- [ ] Export and archive all ArduSub parameters before changing anything.
- [ ] Load and verify the BlueROV2 Heavy eight-thruster `Vectored_6DOF` frame
      parameters.
- [ ] Complete accelerometer, compass, pressure/depth and joystick calibration.
- [ ] Resolve every pre-arm failure; do not disable arming checks to save time.
- [ ] Configure and test GCS-heartbeat, battery, leak, internal-pressure and
      internal-temperature warnings/failsafes appropriate to the test site.
- [ ] Verify the physical motor numbers and directions using the official
      BlueROV procedure. Perform automatic direction detection floating in
      water with clearance. Keep any manual operation in air very brief.
- [ ] Verify the physical joystick, motor emergency stop, disarm and pilot mode
      changes without ROS running.
- [ ] Verify Cockpit/QGroundControl telemetry and download an ArduSub log.

Go/no-go: the vehicle must be safely and predictably operable by the pilot before
any ROS process receives control authority.

## Phase 2: physical and site preparation

- [ ] Inspect enclosure O-rings, penetrators, cable strain relief, propellers,
      guards, fasteners and tether termination.
- [ ] Perform the manufacturer-specified vacuum/leak test.
- [ ] Check battery condition, voltage, capacity setting, connectors and fuse.
- [ ] Ballast the vehicle neutrally or slightly positively buoyant. It should
      return toward the surface after loss of thrust.
- [ ] Choose calm, clear, shallow water with generous clearance from walls,
      floor, vegetation, lines, swimmers and metallic structures.
- [ ] Establish a marked exclusion zone around the thrusters and vehicle.
- [ ] Route the tether so it cannot enter a propeller or pull the vehicle into a
      wall. Assign one person to manage slack if necessary.
- [ ] Confirm recovery equipment and a non-powered retrieval plan.
- [ ] Record water conditions, visibility, approximate current and test-area
      dimensions.

No person touches or approaches the vehicle while it is armed. Water does not
make a spinning thruster safe.

## Phase 3: hardware-adapter acceptance

This phase accepts the implemented adapter against the real vehicle. Perform it
before any autonomous trajectory.

Install the non-ROS Python dependencies, build, and source the workspace:

```bash
python3 -m pip install -r requirements.txt
colcon build --symlink-install --packages-select frontier_slam
source install/setup.zsh
```

In BlueOS, create a dedicated external UDP endpoint targeting the test
computer's IP and port `14560`. Keep the normal Cockpit/QGroundControl endpoint
running. Confirm `target_system` and `target_component` in
`config/ardusub.yaml` match the vehicle heartbeat (normally `1/1`). Start only
the gate and adapter for acceptance—not the planner:

```bash
ros2 launch frontier_slam ardusub_adapter.launch.py \
  backend:=manual_control \
  connection_url:=udpin:0.0.0.0:14560 \
  odom_topic:=/your/real/odometry
```

Monitor both layers:

```bash
ros2 topic echo /motion/safety_status
ros2 topic echo /motion/ardusub_status
```

The adapter must remain inhibited until the gate is `ACTIVE`, the vehicle is
armed, and the pilot has selected `ALT_HOLD`. For SITL, unarmed message
inspection, or a powered test with the vehicle safely submerged and the site
prepared as above, publish one low normalized demand continuously from exactly
one source. Enable the gate only for the intended short pulse, then press
**DISABLE NOW**. Never use this command for sustained dry thruster operation:

```bash
ros2 topic pub -r 10 /motion/body_command geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}}"
```

Stop this publisher before starting any autonomous controller.

- [ ] The real launch must not start Stonefish, ground-truth odometry, or a
      simulator actuator subscriber.
- [ ] Confirm ROS connects through a dedicated BlueOS MAVLink endpoint without
      disrupting Cockpit/QGroundControl or its heartbeat.
- [ ] Confirm exactly one ROS command publisher is visible.
- [ ] Keep arming and mode changes under pilot control; the adapter must not arm
      automatically on startup or reconnection.
- [ ] Confirm the adapter outputs neutral until both pilot authorization and the
      ROS safety gate are enabled.
- [ ] Confirm `/motion/ardusub_status` reports each inhibited cause correctly:
      no/stale heartbeat, disabled gate, disarmed vehicle and wrong mode.
- [ ] Confirm ROS and ArduSub agree on NED/ENU conventions, yaw sign, positive
      depth, body X forward and body Y starboard.
- [ ] Test one body axis at a time at the lowest practical authority: surge,
      sway, yaw, then vertical.
- [ ] Confirm stopping ROS commands, killing the adapter, unplugging Ethernet,
      stopping odometry and disabling the gate all lead to the predefined safe
      action.
- [ ] Confirm pilot takeover works while ROS is actively requesting motion.
- [ ] Confirm changing out of `ALT_HOLD` inhibits the manual backend and that
      Cockpit/QGroundControl regains sole pilot-input authority.

For initial tests, let ArduSub hold attitude and depth. Command horizontal body
motion and yaw only. Do not run the existing ROS heave P-controller against an
ArduSub depth-hold loop until their responsibilities and signs have been tested
and documented.

Only after ExternalNav/EKF acceptance, repeat this phase with:

```bash
ros2 launch frontier_slam ardusub_adapter.launch.py \
  backend:=local_ned_velocity \
  connection_url:=udpin:0.0.0.0:14560 \
  odom_topic:=/your/real/odometry
```

The pilot must select `GUIDED`. Verify the outgoing MAVLink message is number
84, frame `MAV_FRAME_BODY_NED`, type mask `1479`, with only velocity and yaw
rate active. Vertical velocity remains zero by default.

## Phase 4: low-power wet control

Use a restrained gain/thrust limit, initially around 10–15% if appropriate for
the vehicle and site.

1. Launch and operate manually with Cockpit/QGroundControl only.
2. Put the vehicle at a stable shallow depth and verify neutral buoyancy.
3. Start the real ROS sensor and state-estimation stack with the gate disabled.
4. Compare ROS yaw and depth against the control station while manually moving
   the vehicle. Stop on any sign or frame mismatch.
5. Start the hardware adapter, still neutral and gated.
6. Use the RViz panel to enable one short low-power surge pulse, then press
   **DISABLE NOW**. Verify direction and stopping distance.
7. Repeat separately for reverse, sway and yaw.
8. Test command timeout, odometry timeout and pilot takeover in the water.
9. Test depth behaviour separately, with a strict shallow-depth limit and the
   pilot ready to surface.

Go/no-go: do not proceed unless every body axis, stop path and takeover path is
repeatable and correctly logged.

## Phase 5: fixed trajectory before exploration

Do not make frontier exploration the first autonomous trajectory.

1. Use a known, obstacle-free path with a small number of waypoints.
2. Disable the controller's automatic startup/goal-reached scan for the first
   runs; motion should begin only from an explicit test instruction.
3. Use conservative speed, acceleration, yaw-rate and depth limits.
4. Run one axis and one waypoint at a time before attempting a rectangle or
   out-and-return trajectory.
5. Compare desired and measured surge/sway/yaw/depth, cross-track error,
   stopping distance and command latency.
6. Repeat with the tether approaching its least favourable direction.
7. Increase authority only after reviewing logs between runs.

## Phase 6: mapping and autonomous exploration

- [ ] Verify real sonar timestamps, frame, ranges, invalid-value handling and
      point-cloud orientation while the vehicle is manually driven.
- [ ] Verify SLAM odometry remains fresh and continuous during yaw turns and
      short sonar dropouts.
- [ ] Set real-world map inflation and standoff margins using measured vehicle
      size, stopping distance, localization error and sonar uncertainty.
- [ ] Place the first autonomous frontier far from walls, tether hazards and
      people.
- [ ] Keep the pilot in control for the entire run; autonomy is never
      unattended.
- [ ] Use short bounded runs with a predefined time, depth, distance and area.
- [ ] Review the bag, controller CSV, safety status and ArduSub log after every
      run before expanding the envelope.

## Immediate stop criteria

Call STOP and end the run for any of the following:

- unexpected motor, axis direction, dive or surfacing command;
- `STALE_ODOMETRY`, `INVALID_COMMAND`, or repeated safety-state transitions;
- more than one command publisher;
- loss or degradation of pilot video, telemetry, heartbeat or joystick control;
- unexplained estimator jump, yaw discontinuity or depth disagreement;
- vehicle crossing the permitted depth, range or exclusion boundary;
- tether approaching a thruster, snagging, high tension or unexpected drag;
- leak, abnormal internal pressure/temperature, battery alarm, smoke, smell,
  bubbles, unusual noise or vibration;
- current or oscillation beyond the pilot's easy recovery authority;
- any observer losing confidence in the safety of the run.

After STOP: activate motor emergency stop/disarm, disable the ROS gate, recover
the vehicle without thrust if possible, disconnect power when safe, preserve
all logs, and document the reason before attempting another run.

## Data to record for every run

- Git commit, launch command and parameter files.
- ArduSub, BlueOS and control-station versions and exported parameters.
- ROS bag containing commands before and after the gate, safety status,
  odometry, TF, raw sonar, map, goals and paths.
- Controller CSV and ArduSub onboard/tlog logs.
- Operator event log with enable, arm, mode, STOP, faults and recovery times.
- Video with a visible time reference if available.
- Battery start/end voltage, water/site conditions and tether configuration.
- Pass/fail against the planned objective and any anomaly, even if recovered.

Never reuse a successful simulation gain as proof that the corresponding
real-world gain is safe. Each increase in speed, authority, depth, range or
autonomy is a new test-envelope expansion and needs its own go/no-go decision.
