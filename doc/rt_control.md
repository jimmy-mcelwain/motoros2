<!--
SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
SPDX-FileCopyrightText: 2025, Delft University of Technology

SPDX-License-Identifier: CC-BY-SA-4.0
-->

# R/T Motion Control

The real-time motion control server is intended to be used in a closed loop system.
It allows the user to command joint positions or TCP poses, or increments of either, at the rate of the robot controller's interpolation clock.
The commands are passed directly to the MotoPlus motion API (`mpExRcsIncrementMove`), with as little overhead as possible.

Note: seven axis robots, robots with four or five axes (SCARA robots, palletizers) and several control groups have only been tested with simulated servos (dummy servo).
PFL (HC robots) has not been tested.

## Activation

This control mode is activated using the [start_rt_mode](ros_api.md#start_rt_mode) service.
The user must specify the `control_mode`:

| `control_mode` | Packets contain |
| -------------- | --------------- |
| `JOINT_INCREMENTS (1)` | joint increments (radians) |
| `CARTESIAN_INCREMENTS (2)` | TCP increments (meters / quaternion) |
| `JOINT_ABSOLUTE (3)` | joint positions (radians) |
| `CARTESIAN_ABSOLUTE (4)` | TCP poses (meters / quaternion) |

The joint modes are recommended, because their motion is more predictable: each joint moves as commanded, and MotoROS2 checks each joint's maximum speed.
In the Cartesian modes, the controller computes the joint motion, which can differ from what the client expects (see [Cartesian](#cartesian)).
In the absolute modes, a lost command or a slowdown of the robot is made up for by the next command, within limits (see [Absolute positions](#absolute-positions)).
In the incremental modes, the client has to make up for it.

The Cartesian modes are only supported when every control group is a robot (no positioners or base tracks) with at least six axes: on robots with fewer axes, the orientation can't be controlled.

While a session is running, `start_rt_mode` is rejected, and the session continues.
It returns `NOT_READY_ALREADY_ACTIVE (120)`, or `NOT_READY_OTHER_TRAJ_MODE_ACTIVE (114)` if it switches between joint and Cartesian control.
If the robot isn't ready for motion for a reason `start_rt_mode` can't fix itself (ex: the `Error` bit is set), it returns that reason instead (ex: `NOT_READY_ERROR`).
To start a new session (ex: after the client was restarted, or with other settings), call `stop_traj_mode` first.
Once a session has ended (ex: after a timeout), `start_rt_mode` alone starts a new one, except that switching between joint and Cartesian control always needs `stop_traj_mode` first.

If the service is successful, it returns a `result_code` of `Ready (1)`.
Otherwise, see the `result_code` and `message` fields of the response.
It also returns the `period` in milliseconds at which commands are expected: the controller's interpolation period (ex: 4 milliseconds for a single robot on a YRC1000; it depends on the robot model and the number of control groups).

## Usage

### Command Flow

The UDP port (`rt_listener_udp_port_number`, `22000` by default) is opened when MotoROS2 starts.
Packets which arrived while no session was running are discarded when `start_rt_mode` is called.

The first command must have its `sequenceId` set to `0`.
It is a handshake, in every control mode: its positions or increments are ignored, and only its `toolIndex` takes effect.
Its reply reports the position the robot has been commanded to (`previousCommandPositionJoints` / `previousCommandPositionCartesian`), and the next command moves from there.
Each subsequent command must increment the `sequenceId`, and must not be sent before the reply to the previous command.

The reply is sent once the motion for the command has been passed to the robot's interpolation cycle, so the replies come at the rate of the `period`.
Send the next command as soon as possible after the reply: it has to arrive within part of a `period` (about 2.3 milliseconds after the reply on a GP12 with a 4 millisecond `period`), or the robot does not move during the next cycle.

Other packets which arrive before the reply to the previous command (a resend, or a packet with the wrong size or `version`) are discarded without a reply, and the session continues.
A resend of a command which was already executed (ex: because its reply was lost) is not executed again: the latest reply is sent again.

If no command is received within `timeout_for_rt_msg` (default: 5 seconds; `-1`: never), the session ends (`Timeout`).
To keep the session alive while the robot stands still, keep sending commands: the same position in the absolute modes, or zero increments in the incremental modes.
This also keeps the controller's energy saving mode from engaging.
If it engages anyway (ex: with `timeout_for_rt_msg` set to `-1`), the session ends within 100 milliseconds with `MotionRejected` (`endDetail` `-20`), and nothing more is executed.
`start_rt_mode` takes the robot out of energy saving mode, so a new session can be started right away.

The session also ends if:

- a command has the wrong size, `version` or `packetType`
- a packet is received from another IP address than the session's owner (see below)
- a command is sent before the reply to the previous command (`CommandTooEarly`)
- the first `sequenceId` is not `0`, or the `sequenceId` skips ahead by more than `max_sequence_diff_for_rt_msg` (default: `3`)
- a command is invalid (not a finite number, not a unit quaternion, a `toolIndex` not `0` - `63`) or exceeds the speed limits
- the robot rejects the motion or stops (ex: E-stop, HOLD, alarm, PFL, servo power off, energy saving mode), or the pendant leaves REMOTE mode (see [Robot status](#robot-status))
- MotoROS2 can't read the robot's status or position, or receiving from the R/T port fails (`InternalError`)
- `stop_traj_mode` is called, or the micro-ROS agent disconnects

The R/T port accepts packets on every network interface of the controller, without authentication.
The session belongs to the host which sends the first valid command, which doesn't have to be the host that called `start_rt_mode`.
Any packet from another IP address ends the session (`MultipleSources`, or an invalid command before the first one), even one which isn't an R/T command (ex: from a network scanner).
So make sure that no other host sends packets to the R/T port during a session (ex: with a firewall in front of the controller).

MotoROS2 notices that the micro-ROS agent disconnected after a few seconds (ex: 4 to 8 seconds on a YRC1000), and the session goes on until then.
A restart of the agent within a few seconds may not be noticed at all: MotoROS2's services, `stop_traj_mode` included, then stay unavailable.
Stop sending commands to end the session (`Timeout`), or, if `timeout_for_rt_msg` is `-1`, press HOLD.

When the session ends, a final reply is sent with `replyType` set to `SessionEnded`, and `endReason` / `endDetail` indicating why (see [Session end](#session-end), and the debug log for more detail).
A new session can be started as soon as it has been received.
It is sent over UDP, so it can be lost: if no reply arrives within `timeout_for_rt_msg` (or a time of the client's choosing, if that is `-1`), assume that the session has ended.

MotoROS2 doesn't decelerate the robot: when commands stop arriving, or the session ends, no more motion is sent.
The controller's smoothing (see [Latency](#latency)) slows the robot down over a few tenths of a second, but that can be too abrupt (ex: at high speed, or with a payload).
So bring the robot to rest before ending a session.

The motion API only accepts whole units (pulses, micrometers and 0.0001 degrees).
Increments are rounded to the nearest whole unit, and the rest is carried over to the next cycle, so small motions are not lost.

<img src="img/RtFlow.png" alt="Command Flow" />

### Latency

Measured on a GP12 (YRC1000, 4 millisecond `period`):

- The reply arrived 3.6 milliseconds after the command was sent (median, network included).
- The commanded position in the reply already includes the command it answers.
- The feedback position trailed the commanded position by about 0.2 seconds.
- The controller smooths the motion: a step took about 0.2 seconds to rise from 10 % to 90 %, and motion which changes direction faster than about 2 Hz is attenuated (70 % of the commanded amplitude at 2 Hz, almost nothing at 5 Hz).

This is the controller's behavior, not MotoROS2's, and it may differ for other robots and controllers.
A control loop which uses the feedback position has to allow for about 0.2 seconds of delay.

While MotoROS2 publishes (`joint_states`, `robot_status` and `tf`), a reply can be sent late, and the robot then gets no motion for one cycle, which the controller's smoothing absorbs.
On a GP12 with the default settings, this happened about once or twice a minute.
Publishing less often (`controller_status_monitor_period`, `executor_sleep_period`) and `publish_tf: false` make it rarer.

### Data format (command)

The command packet is a *packed* `RtPacket` structure.

```c
//##########################################################################
//                  !All data is little-endian!
//##########################################################################
struct RtPacket
{
    //Must match the version expected by MotoROS2.

    int version;

    //Must match the control_mode selected with start_rt_mode.

    PacketType packetType;

    //Must increment with each new command. The first command of a
    //session must be 0.

    UINT32 sequenceId;

    //The positions to move to (absolute control modes), or the
    //increments (incremental control modes).
    //Joints: radians (or meters for linear axes), in the order selected
    //by joint_order. See Joints below.
    //Cartesian: [X Y Z Qx Qy Qz Qw Re]. See Cartesian below.

    double command[MAX_GROUPS][MP_GRP_AXES_NUM];

    //The tool of each control group. Cartesian commands, and the Cartesian
    //poses in the reply, are for this tool's TCP. The select_motion_tool
    //service doesn't affect R/T sessions.

    int toolIndex[MAX_GROUPS]; //TOOL 0 - 63

    //Reserved for future expansion. Set it to 0: a later version may use it.

    char reserved[64];
}
```

The current `version` is `2`.
If a command has another version, the session ends (`InvalidVersion`), and `endDetail` is the version MotoROS2 expects.
Every reply starts with this version, so a client can check it even if the rest of the reply isn't laid out as it expects.

The `packetType` must be the same as the `control_mode`:

```c
enum PacketType
{
    Joint_Increments = 1,   //JOINT_INCREMENTS (1)
    Cart_Increments = 2,    //CARTESIAN_INCREMENTS (2)
    Joint_Absolute = 3,     //JOINT_ABSOLUTE (3)
    Cart_Absolute = 4,      //CARTESIAN_ABSOLUTE (4)
}
```

#### Joints

In the joint modes, the `command` array contains the position (`JOINT_ABSOLUTE`) or the increment (`JOINT_INCREMENTS`) of each joint.
The order of the joints is selected by the `joint_order` field of the [start_rt_mode](ros_api.md#start_rt_mode) service.
It is also used for the joint positions in the reply, in all control modes, and it applies to each control group separately.

| `joint_order` | Order | Seven axes | Four axes (ex: palletizer) |
| ------------- | ----- | ---------- | -------------------------- |
| `JOINT_STATES (0)` (default) | Same as the `joint_states` topic (no gaps) | `S L E U R B T` | `S L U T` |
| `MOTOMAN (1)` | Motoman pulse order | `S L U R B T E` | `S L U - - T` |

The orders only differ for robots with seven axes (`E` moves to third place) and robots with fewer than six (the gaps are removed).
For all other robots, both give the controller's pulse order, as `joint_states` does (usually `S L U R B T`, but not on every robot, ex: an MH900).

Linear axes (ex: base tracks) are in meters.

On five axis robots whose B axis moves automatically to keep the tool level (ex: MPL80/100), the joint positions and increments are those of the physical joints, as in `joint_states`.
So moving L or U without B tilts the tool.
The controller limits that tilt (ex: to 15 degrees on a PL80, alarm 4420), which ends the session with `MotionRejected`.

#### Cartesian

In the Cartesian modes, the `command` array contains the TCP pose (`CARTESIAN_ABSOLUTE`) or the change of it (`CARTESIAN_INCREMENTS`), in the order `X Y Z` (meters), `Qx Qy Qz Qw` (quaternion), `Re` (radians).
Both are expressed in the robot coordinate frame (RF), for the tool selected by `toolIndex`.
With several robots, each robot's pose is in its own robot frame, and the robots move independently (no coordinated motion).

The rotation must be a unit quaternion: a length within `0.01` of `1` is accepted, and normalized.
In `CARTESIAN_INCREMENTS`, no rotation is `0 0 0 1`, or all zeros, so a zero-filled command can be used as a keep-alive.
`Re` is the elbow angle of seven axis robots (ignored for other robots), and wraps around at ±180 degrees like the orientation.
The controller may change the elbow angle by itself, as it does when jogging.
In `CARTESIAN_ABSOLUTE`, the next targets bring it back.
In `CARTESIAN_INCREMENTS`, the reply shows it (and `motionDeviationDetected` is set), and the client can correct it.

To catch client errors (ex: wrong units, or a pose sent in `CARTESIAN_INCREMENTS`), the session ends if the motion in one cycle exceeds 1500 mm/sec of translation, or 360 deg/sec of rotation or of elbow angle.
These are not safety limits: the robot controller and the Functional Safety Unit (FSU) enforce those.
MotoROS2 doesn't check joint limits, interference zones, the FSU's range limits, acceleration, or (in the Cartesian modes) the joint speeds: the client has to send motion the robot can follow.
Otherwise the controller stops the robot with an alarm (ex: `4414`, `4416`, `4784`), and the session ends with `MotionRejected`.

Near a singularity, the controller may not execute a Cartesian motion as sent, as with the pendant's Cartesian jog (ex: near the wrist singularity, it can hold a wrist joint still and drop the part of the motion which needs it).
No alarm is raised, and the session continues: only the commanded position in the reply and `motionDeviationDetected` show it.
In the absolute modes, the difference can then grow until the session ends (`InvalidIncrement`, or `MotionRejected` with an alarm).
Stop, or move away from the singularity, when the flag is set, or use joint mode near singularities.

#### Absolute positions

The motion API only accepts increments.
So each cycle, MotoROS2 sends the difference between the position in the command and the position the robot has been commanded to.
A fraction of a unit which can't be sent, a lost command, or a slowdown of the robot (ex: by the FSU) is made up for by the next command, as long as that stays within one cycle's motion.
The same limits apply as for the increments (the maximum speed of each joint, 1500 mm/sec, and 360 deg/sec of rotation and of elbow angle): a position which is further away ends the session with `InvalidIncrement`.

So every command must contain the position of every axis of every control group, not only of those which move (ex: start from the positions in the reply to the handshake): `0` is a position, not "don't move".
As in every mode, the first command is only a handshake, and is **not** executed (see [Command Flow](#command-flow)).

When the robot is slowed down (ex: by the FSU speed limit), it falls behind, and the next position may be more than one cycle's motion away.
So a client which plays back a trajectory planned in advance should slow its playback down when `motionDeviationDetected` is set, or when the commanded position in the reply falls behind.
A client which computes each position from the commanded position in the reply is never more than one cycle behind.

In `CARTESIAN_ABSOLUTE`, a command which changes the tool of a control group doesn't move that group (its pose for that group is ignored), like the handshake: its reply reports the pose of the new tool's TCP, and the next command must be within one cycle's motion of it.
So change the tool while that group is standing still.
In the other modes, a tool change takes effect with the command that makes it (in the joint modes, only the Cartesian poses in the reply change).

In joint mode, the commanded position is read from the controller every cycle.
In Cartesian mode, MotoROS2 keeps track of the commanded pose itself, because the pose the controller reports is computed from whole encoder pulses.
It only reads it again when the two differ by more than `rt_motion_deviation_threshold_micrometers` or `rt_motion_deviation_threshold_deg_0001`, or the tool changes.
So after the robot was slowed down by less than these thresholds, it can stop up to that far short of the target (lower them if needed, but not below the TCP motion of about one encoder pulse).
For the same reason, the pose in the reply can differ from the pose MotoROS2 keeps track of by up to about one pulse: to keep the robot still, send the same target again, rather than the pose from the reply.

### Data format (reply)

The reply packet is a *packed* `RtReply` structure.

```c
//##########################################################################
//                  !All data is little-endian!
//##########################################################################
struct RtReply
{
    //The version of the R/T interface which MotoROS2 implements.

    int version;

    UINT32 sequenceEcho;

    //See ReplyType enum. The final reply of a session (SessionEnded) echoes
    //the last command which was executed, and its position data is read
    //when the session ends (with the tools of that command).

    UINT32 replyType;

    //See EndReason enum. Only set when replyType is SessionEnded.

    UINT32 endReason;

    //Additional information about the endReason. See EndReason enum.

    int endDetail;

    //Bitfield of RobotStatusBits. Read right after the previous reply was
    //sent, so it is one cycle old (current in a SessionEnded reply).

    UINT32 robotStatus;

    //Where the robot physically is. This trails behind the commanded
    //position. Joints in the order of the command packet.

    double feedbackPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double feedbackPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];

    //The position the robot has been commanded to: the sum of all the
    //increments, or (in the absolute control modes) the positions, as far
    //as the robot has followed them.

    double previousCommandPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double previousCommandPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];

    //The controller did not execute the increments up to the *previous*
    //command as sent. Says nothing about this most recent command.
    //Always false in a SessionEnded reply.

    bool motionDeviationDetected;
}
```

`motionDeviationDetected` is set when the controller did not execute the increments as sent (ex: the FSU speed limit truncated them, or near a singularity, see [Cartesian](#cartesian)).
The motion continues, but not as commanded.
MotoROS2 compares the commanded position with the position it expects (a position read from the controller, plus every increment sent since), so small deviations add up until they pass the thresholds in the configuration file: `rt_motion_deviation_threshold_pulses` (joint mode), and `rt_motion_deviation_threshold_micrometers` and `rt_motion_deviation_threshold_deg_0001` (Cartesian mode).
Then the flag is set, and the comparison starts again from the commanded position, so a steady deviation sets the flag every few cycles.
In joint mode, the increments are sent in whole pulses, so the threshold is `0` by default: any difference sets the flag.
The flag is set if any control group deviates: compare each group's commanded position with the expected one to see which one did.
When the FSU (or anything else) stops the robot, the session ends with `MotionRejected` instead.

### Session end

```c
enum ReplyType
{
    Feedback = 0,         //Reply to a command packet. The next command may be sent.
    SessionEnded,         //The session has ended (see EndReason). No more replies will be sent.
}

enum EndReason
{
    None = 0,
    StopRequested,        //stop_traj_mode was called, or the micro-ROS agent disconnected
    Timeout,              //No command was received within timeout_for_rt_msg
    InvalidPacketSize,    //endDetail: the number of bytes received
    InvalidVersion,       //endDetail: the version MotoROS2 expects
    InvalidPacketType,    //endDetail: the packetType received
    MultipleSources,      //A command was received from a second IP address. endDetail: that address (network byte order)
    SequenceGap,          //The sequenceId skipped ahead, or the first one isn't 0. endDetail: the sequenceId received
    InvalidIncrement,     //Not a finite number, not a unit quaternion, exceeds a speed limit, or invalid toolIndex
    MotionRejected,       //The robot rejected the motion or stopped. endDetail: see below
    InternalError,        //See debug log (ex: the robot's status or position couldn't be read)
    CommandTooEarly,      //A command was sent before the reply to the previous one. endDetail: its sequenceId
}
```

`endDetail` is `0` for the reasons which don't list a value.
For `MotionRejected`, it is the return code of `mpExRcsIncrementMove`, or `0`:

| `endDetail` | Meaning |
| ----------- | ------- |
| `0` | The robot stopped (ex: E-stop, HOLD), its drives are off, the pendant left REMOTE mode, or the job stopped: after the controller accepted the increment, or while no commands arrived. See `robotStatus` for the cause. |
| `-1` | The job is not executing its `WAIT` instruction (ex: an alarm or HOLD stopped it). See `robotStatus` for the cause. |
| `-2` | No job is running |
| `-8` | The control group is being jogged |
| `-9` | The control group is executing a move instruction |
| `-13` | Another job is using the control group |
| `-19` | PFL is active (HC robots) |
| `-20` | Energy saving mode is active: the drives are off. Also when energy saving mode engages while no commands arrive (see [Command Flow](#command-flow)) |

The other return codes (`-3` - `-7`, `-10` - `-12`) should not occur.
The same cause can give either `0` or `-1` (ex: HOLD), depending on whether the controller accepted the increment before it stopped.

### Robot status

```c
enum RobotStatusBits
{
    DrivesPowered   = (1 << 0),     //Servo power is on, and not in energy saving mode
    EStopped        = (1 << 1),     //Pendant, external or controller E-stop
    Hold            = (1 << 2),
    Alarm           = (1 << 3),     //Major, minor, system or user alarm
    Error           = (1 << 4),
    PlayMode        = (1 << 5),     //Pendant is in PLAY or REMOTE mode (not TEACH)
    PflActive       = (1 << 6),     //PFL has stopped the robot, or is escaping/avoiding (HC robots only)
    RemoteMode      = (1 << 7),     //Pendant is in REMOTE mode
    JobRunning      = (1 << 8),     //The job is running (not held or stopped)
}
```

While a session is active, the robot is powered, in REMOTE mode, free of E-stops, HOLD, alarms and PFL stops, and its job is running.
Otherwise the session ends with `MotionRejected` (`endDetail` `0`, or `-20` in energy saving mode, unless the controller rejected the motion first).
This is needed because the controller doesn't always reject the motion when the robot stops (ex: while the job is held, increments can be accepted but not executed).
MotoROS2 reads the status after each command, every 100 milliseconds while no commands arrive, and again before it executes the command which ends such a wait.
An error (ex: a wrong password entered on the pendant) sets `Error`, but doesn't stop the robot, so the session goes on.

If the status, or a position for a reply, can't be read, the session ends with `InternalError` (the final reply may then contain zeros).
In joint mode, a Cartesian pose which can't be computed is left at zero (with an invalid quaternion), and the session goes on.

The status can show the cause of a stop a few cycles after the motion was rejected, so the final reply waits for the cause to appear, for up to 200 milliseconds.
Some bits can still lag behind in the final reply:

- After an E-stop, it can still show `DrivesPowered`: the servo power goes off a moment later (about 0.5 seconds on a GP12).
- It can still show `JobRunning` together with the cause (ex: after an E-stop, always; after HOLD, sometimes): the job stops about 15 to 30 milliseconds after the cause appears (on a GP12).
  So use the cause bits, not `JobRunning`, to tell why the robot stopped.
- A short HOLD can be released before the status is read, so the reply doesn't show `Hold`.
  The job stays stopped, though, so the reply then shows `JobRunning` cleared.

For complete controller status (ex: alarm codes), use the [robot_status](ros_api.md#robot_status) topic.

If the session ends before the first command was accepted, the final reply is only sent if a packet was rejected, to the host which sent that packet, and its position data is all zeros.

### Wire format

All structures are packed (no padding) and little-endian.
The field sizes are:

| Type | Size (bytes) |
| ---- | ------------ |
| `int`, `UINT32`, `PacketType` | 4 |
| `double` | 8 |
| `bool` | 1 |

| Structure | Size (bytes) | Python `struct` format |
| --------- | ------------ | ---------------------- |
| `RtPacket` | 620 | `<iiI64d8i64x` |
| `RtReply` | 2073 | `<iIIIiI256d?` |

`MAX_GROUPS` and `MP_GRP_AXES_NUM` are both `8`.
Unused groups and axes are ignored (set them to `0`).
Each control group has its own row of `command` and of each array in the reply, and its own `toolIndex`.
The groups are in the controller's order, as in `joint_states` (`group_1`, `group_2`, ...): robots first (`R1`, `R2`, ...), then base axes (`B1`, ...), then stations (`S1`, ...).

The reply doesn't fit in one standard Ethernet frame (MTU 1500), so it is always sent as two IP fragments, and it is lost if either fragment is.
Firewalls, NAT routers and other devices between the client and the controller must pass fragmented UDP: otherwise `start_rt_mode` succeeds, but the client never receives a reply (see [Firewall issues](network_configuration.md#firewall-issues)).

## Deactivation

Other motion modes may not be used while the R/T mode is active, even after a session has ended (ex: after a timeout).
`start_traj_mode` and `start_point_queue_mode` return `NOT_READY_OTHER_TRAJ_MODE_ACTIVE (114)`, and a `FollowJointTrajectory` goal is aborted with `INIT_TRAJ_WRONG_MODE` (`error_code` `-100209`).
Calling `stop_traj_mode` ends the session (`StopRequested`) and the R/T mode, so another motion mode may be used.
The UDP port stays open, so a new session can be started with `start_rt_mode`.
