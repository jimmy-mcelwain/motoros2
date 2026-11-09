<!--
SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
SPDX-FileCopyrightText: 2025, Delft University of Technology

SPDX-License-Identifier: CC-BY-SA-4.0
-->

# R/T Motion Control

The real-time motion control server is intended to be used in a closed loop system.
It allows the user to command joint positions or TCP poses, or increments of either, at the rate of the robot controller's interpolation clock.
This control mode minimizes overhead as much as possible by routing the user commands directly to the MotoPlus motion API, mpExRcsIncrementMove.

Note: R/T control has not been tested with robots with fewer than six axes (four and five axis arms, and SCARA robots).
Seven axis robots have only been tested with simulated servos (dummy servo).
On five axis robots whose B axis moves automatically to keep the tool level (ex: MPL80/100), a joint increment for B may be relative to that automatic motion, rather than to the physical joint.

## Activation

This control mode is activated using the [start_rt_mode](ros_api.md#start_rt_mode) service.
The user must specify the `control_mode`:

| `control_mode` | Packets contain |
| -------------- | --------------- |
| `JOINT_INCREMENTS (1)` | joint increments (radians) |
| `CARTESIAN_INCREMENTS (2)` | TCP increments (meters / quaternion) |
| `JOINT_ABSOLUTE (3)` | joint positions (radians) |
| `CARTESIAN_ABSOLUTE (4)` | TCP poses (meters / quaternion) |

The joint modes are recommended over the Cartesian modes, because their motion is more predictable: each joint moves as commanded, and MotoROS2 checks each joint's maximum speed.
In the Cartesian modes, the controller computes the joint motion itself, and it can differ from what the client expects (ex: near a singularity, or the elbow angle of a seven axis robot, see [Cartesian](#cartesian)).

In the absolute modes, a lost command or a slowdown of the robot is made up for by the next command, within limits (see [Absolute positions](#absolute-positions)).
In the incremental modes, the client has to make up for that itself.
Choose whichever fits how the client computes its motion.

The Cartesian modes are only supported when every control group is a robot (no positioners or base tracks).

While a session is running, `start_rt_mode` is rejected with `NOT_READY_ALREADY_ACTIVE (120)` (or `NOT_READY_OTHER_TRAJ_MODE_ACTIVE (114)`, if it selects joint control in a Cartesian session, or the reverse), and the session continues.
If the robot isn't ready for motion at that moment (ex: the `Error` bit is set, which doesn't end the session, see [Robot status](#robot-status)), it is rejected with that reason instead (ex: `NOT_READY_ERROR`), and the session continues as well.
To start a new session (ex: after the client was restarted, or with other settings), call `stop_traj_mode` first.
Once a session has ended (ex: after a timeout), calling `start_rt_mode` alone starts a new one.
Switching between joint and Cartesian control always needs `stop_traj_mode` first, even after a session has ended: otherwise `start_rt_mode` returns `NOT_READY_OTHER_TRAJ_MODE_ACTIVE (114)`.

If this service is successful, it will return a `result_code` of `Ready (1)`.
Otherwise, please examine the `result_code` and `message` fields in the response for more information.

The service will also return a `period` in milliseconds.
This indicates the rate at which commands will be expected by the robot.
It is the controller's interpolation period, which depends on the robot model and the number of control groups (ex: 4 milliseconds for a single robot on a YRC1000, 2 milliseconds for a SCARA robot on a YRC1000micro, and 8 or 16 with several control groups).

## Usage

### Command Flow

The UDP port (`22000` by default, set by `rt_listener_udp_port_number`) is opened when MotoROS2 starts.
Packets which arrived while no session was running are discarded when `start_rt_mode` is called.
The user then sends the first command with the `sequenceId` field set to `0`.
In the absolute modes, the first command is not executed: its reply reports the position the robot has been commanded to, which the next command moves from.
See [Absolute positions](#absolute-positions).
In the incremental modes, the first command is executed.
It is recommended that it contains zero increments, so the client can read the commanded position from the reply before it moves the robot.
Each subsequent command must increment the `sequenceId`, and must not be sent until the robot has replied to the previous command.
The replies are sent at the rate of the `period` from the [start_rt_mode](ros_api.md#start_rt_mode) service.

The reply is sent right after the motion for the command has been passed to the robot's interpolation cycle.
To keep the motion smooth, the next command must arrive in time to be processed before the following interpolation cycle.
That is less than one `period` after the reply, because the reply is only sent once the cycle has started (on a GP12 with a 4 millisecond `period`, a command which arrived more than about 2.3 milliseconds after the reply missed the next cycle).
So send the next command as soon as possible after the reply.
If it arrives late, the robot does not move during that cycle.

A command which is sent before the reply to the previous command would be discarded (in the incremental modes, its increment would be lost).
So the session is dropped instead (`CommandTooEarly`).
Other packets which arrive before that reply is sent (a resend, or a packet with the wrong size or `version`) are discarded, without a reply and without ending the session.
A packet from another IP address still ends it (`MultipleSources`).

A resend of a command which was already executed (ex: because its reply was lost) is not executed again.
Instead, the latest reply is sent again, so a client whose reply was lost can resend its command to get one.

If a command is not received within `timeout_for_rt_msg` (default: 5 seconds; `-1`: never), then the session times out and is dropped.
To keep the session alive while the robot stands still, keep sending commands: the same position in the absolute modes, or zero increments in the incremental modes.
These commands also keep the controller's energy saving mode from engaging.
If the robot is already in energy saving mode, `start_rt_mode` takes it out of it.
While no commands arrive (ex: with `timeout_for_rt_msg` set to `-1`), energy saving mode can engage during a session.
The next command then ends the session with `MotionRejected` (`endDetail` `-20`), and is not executed: `robotStatus` shows `DrivesPowered` cleared.
Call `stop_traj_mode` and then `start_rt_mode`, which takes the robot out of energy saving mode.

The session is also dropped if:

- a command has the wrong size, `version` or `packetType`
- a command is received from an IP address other than the one which sent the first valid command (that host owns the session)
- a command is sent before the reply to the previous command
- the `sequenceId` skips ahead by more than `max_sequence_diff_for_rt_msg` (default: `3`)
- a command is invalid (not a finite number, not a unit quaternion) or exceeds the speed limits
- a `toolIndex` is not `0` - `63`
- the robot rejects the motion, or stops (ex: E-stop, HOLD, alarm, PFL, servo power off)
- the pendant leaves REMOTE mode (ex: it is switched to PLAY or TEACH)
- `stop_traj_mode` is called, or the micro-ROS agent disconnects

The R/T port accepts packets on every network interface of the controller, and there is no authentication.
The session belongs to whichever host sends the first valid command after `start_rt_mode`, which doesn't have to be the host that called `start_rt_mode`.
While a session is running, any packet from another IP address ends it (`MultipleSources`), even one which isn't an R/T command (ex: from a network scanner).
Before the first command, such a packet ends it as an invalid command.
So make sure that no other host sends packets to the R/T port while a session is running (ex: block the port for other hosts with a firewall in front of the controller).

MotoROS2 notices that the micro-ROS agent disconnected after a few seconds (ex: 4 to 8 seconds on a YRC1000).
Until then, the session goes on, but replies may come late.
A restart of the agent within a few seconds may not be noticed at all: the session then goes on, but MotoROS2's services, `stop_traj_mode` included, stay unavailable, so stop sending commands to end the session (`Timeout`).
With `timeout_for_rt_msg` set to `-1`, the session then doesn't end by itself: only a command which the robot rejects ends it (ex: one sent while the robot is held). This has not been tested.

When the session ends, a final reply is sent with `replyType` set to `SessionEnded`, and `endReason` / `endDetail` indicating why.
See [Session end](#session-end).
More detail is printed to the debug log.
A new session can be started as soon as the final reply has been received.

MotoROS2 doesn't decelerate the robot: when commands stop arriving, or the session ends, no more motion is sent.
So bring the robot to rest before ending a session (ex: before calling `stop_traj_mode`).
The controller smooths the motion (see [Latency](#latency)), so the robot still slows down over a few tenths of a second, but that can still be too abrupt (ex: at high speed, or with a payload).

The final reply is best-effort, as it is sent over UDP.
Therefore, if the client does not receive a reply packet within `timeout_for_rt_msg` (returned by `start_rt_mode`; if it is `-1`, within a time of the client's choosing), then it should still assume that the session is dead.

The motion API only accepts whole units (pulses, micrometers and 0.0001 degrees).
Increments are rounded to the nearest whole unit, and the rest is carried over to the next cycle, so small motions are not lost.
A joint increment (or, in `JOINT_ABSOLUTE`, the distance to the next position) may exceed the joint's maximum per cycle by up to 0.001 pulse (ex: from rounding in the client's unit conversion).
It is still sent within the maximum, and the excess is sent in a later cycle.

<img src="img/RtFlow.png" alt="Command Flow" />

### Latency

Measured on a GP12 (YRC1000, 4 millisecond `period`), with sines from 0.5 to 5 Hz and smoothed steps, on joint T and on the TCP:

- The reply arrived 3.6 milliseconds after the command was sent (median, network included; 3.9 milliseconds for 99 % of the commands).
- The commanded position in the reply already includes the command it answers.
- The feedback position trailed the commanded position by about 0.19 seconds, at every frequency.
- The controller smooths the motion: a step took about 0.2 seconds to rise from 10 % to 90 %, with less than 1 % overshoot.
  Motion which changes direction faster than about 2 Hz is attenuated: the robot moved about 70 % of the commanded amplitude at 2 Hz, 35 to 40 % at 3 Hz, and almost nothing at 5 Hz.

The delay and the smoothing of the feedback are the controller's behavior, not MotoROS2's, and they may differ for other robots and controllers.
So the robot can't follow motion faster than about 2 Hz, and a control loop which uses the feedback position has to allow for about 0.2 seconds of delay.

While MotoROS2 publishes (`joint_states`, `robot_status` and `tf`, every `controller_status_monitor_period`), a reply can be sent late, so the client's next command can miss its interpolation cycle, and the robot gets no increment for that cycle.
The controller's smoothing absorbs a single missed cycle.
On a GP12 with the default settings (`executor_sleep_period` 10, `controller_status_monitor_period` 10, `publish_tf` true), this happened about once or twice a minute; with 100 / 100 and `publish_tf: false`, about ten times less often.

### Data format (command)

The command packet is a *packed* `RtPacket` structure.

```c
//##########################################################################
//                  !All data is little-endian!
//##########################################################################
struct RtPacket
{
    //The version of the command packet must match the value expected
    //by MotoROS2.

    int version;

    //The packet type must match the control_mode which was specified
    //when invoking the start_rt_mode service.

    PacketType packetType;

    //Must increment sequentially with each new command packet.

    UINT32 sequenceId;

    //The positions to move to (absolute control modes), or the
    //increments (incremental control modes).
    //
    //For joint-space, this will be radians (or meters for linear axes)
    //of each joint, in the order selected by the joint_order field of
    //the start_rt_mode service. See Joints below.
    //
    //For cartesian, this will be meters and quaternion of the TCP.
    //The order must be [X Y Z Qx Qy Qz Qw Re]. See Cartesian below.

    double command[MAX_GROUPS][MP_GRP_AXES_NUM];

    //The tool of each control group. Cartesian commands, and the Cartesian
    //poses in the reply, are for this tool's TCP. The select_motion_tool
    //service doesn't affect R/T sessions.

    int toolIndex[MAX_GROUPS]; //TOOL 0 - 63

    //Reserved for future expansion. Set it to 0: a later version may use it.

    char reserved[64];
}
```

The `version` must match the version number expected by the server.
If it does not match, the session ends (`InvalidVersion`).
The current version is `2`.

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
This order is also used for the joint positions in the reply, in all control modes.
It applies to each control group separately: in a session with a seven axis robot and a palletizer, each group's row uses that robot's order.

| `joint_order` | Order | Seven axes | Four axes (ex: palletizer) |
| ------------- | ----- | ---------- | -------------------------- |
| `JOINT_STATES (0)` (default) | Same as the `joint_states` topic (no gaps) | `S L E U R B T` | `S L U T` |
| `MOTOMAN (1)` | Motoman pulse order | `S L U R B T E` | `S L U - - T` |

MotoROS2 reorders the joints the same way as for `joint_states`, and only for robots with seven axes (`E` moves to third place) and robots with fewer than six (the gaps are removed).
For all other robots, both settings give the controller's pulse order.
For most of them that is `S L U R B T`, but not for all: on a robot whose axes the controller numbers in another order (ex: an MH900), both settings give that order, as `joint_states` does.

Linear axes (ex: base tracks) are in meters.

#### Cartesian

In the Cartesian modes, the `command` array contains the TCP pose (`CARTESIAN_ABSOLUTE`) or the change of it (`CARTESIAN_INCREMENTS`), in the order `X Y Z` (meters), `Qx Qy Qz Qw` (quaternion), `Re` (radians).

Both are expressed in the robot coordinate frame (RF), for the tool selected by `toolIndex`.
With several robots, each robot's pose is in its own robot frame, and the robots move independently (no coordinated motion).
The rotation must be a unit quaternion: a length within `0.01` of `1` is accepted, and normalized.
In `CARTESIAN_INCREMENTS`, no rotation is `Qx Qy Qz Qw` = `0 0 0 1`.
An all-zero quaternion is also accepted there as "no rotation", so a zero-filled command can be used as a keep-alive.
`Re` is the elbow angle (or its change) of seven axis robots.
It is ignored for other robots.
Like the orientation, it wraps around at ±180 degrees (ex: in `CARTESIAN_ABSOLUTE`, a target of 179 degrees is 2 degrees from -179).
The controller may change the elbow angle by itself during Cartesian motion, as it does when jogging.
In `CARTESIAN_ABSOLUTE`, the next targets bring it back.
In `CARTESIAN_INCREMENTS`, the reply shows it (and `motionDeviationDetected` is set), and the client can correct it.
Near some poses, the controller also doesn't carry out a commanded change of the elbow angle completely (ex: on a seven axis robot with every joint at `0` except B, it carried out about 2 of 10 degrees).

To catch client errors (ex: wrong units, or a pose sent in `CARTESIAN_INCREMENTS`), the session is dropped if the motion in one cycle exceeds:

- 1500 mm/sec of translation
- 360 deg/sec of rotation
- 360 deg/sec of elbow angle (seven axis robots)

These are not safety limits.
The robot controller and the Functional Safety Unit (FSU) still enforce the actual limits.
MotoROS2 doesn't check joint limits, interference zones or the FSU's range limits: the controller stops the robot with an alarm (ex: `4416` for a joint's pulse limit, `4784` for the FSU robot range limit), and the session ends with `MotionRejected`.
It doesn't limit acceleration either: the client has to send motion the robot can follow.
The Cartesian limits don't limit the joint speeds: MotoROS2 only checks those in joint mode.
A Cartesian command within the limits can still need a joint faster than its maximum speed (ex: a fast rotation of the tool, or an elbow angle step of more than about one degree per 8 ms cycle on a seven axis robot).
The controller then either executes it anyway (without setting `motionDeviationDetected`), or stops the robot with an alarm (ex: `4414`), and the session ends with `MotionRejected`.

Near a singularity, a small Cartesian motion can require very fast joint motion.
The controller may then not execute the motion as sent.
For example, on a six axis robot near the wrist singularity, it can hold a wrist joint still and execute only the part of the motion which doesn't need that joint: a translation is still carried out, but the orientation of the tool changes, and a rotation which needs that joint is dropped.
The teach pendant's Cartesian jog behaves the same way.
Where this happens depends on the robot and the controller.
`mpExRcsIncrementMove` still succeeds, the session continues, and no alarm is raised: only the commanded position in the reply, and the `motionDeviationDetected` flag, show it.
In the absolute modes, MotoROS2 then tries to make up the difference with the next commands, and the difference can grow.
It ends the session when it exceeds one cycle's motion (`InvalidIncrement`), or when the controller rejects it with an alarm (ex: `4414` "excessive segment", `MotionRejected`).
Stop, or move away from the singularity, when the flag is set, or use joint mode near singularities.

#### Absolute positions

In the `JOINT_ABSOLUTE` and `CARTESIAN_ABSOLUTE` modes, the `command` array contains the position to move to (see [Joints](#joints) and [Cartesian](#cartesian)).
In `CARTESIAN_ABSOLUTE`, a command which changes the tool of a control group is not executed for that group, like the first command of a session: the client can't know where the new tool's TCP is yet.
Its pose for that group is ignored (not checked), and its reply reports the pose of the new tool's TCP (`previousCommandPositionCartesian`).
The next command must be within one cycle's motion of that pose.
The other groups move as commanded.
So change the tool while that group is standing still: the switching command doesn't move it.
In the other modes, a tool change takes effect with the command that makes it: in `CARTESIAN_INCREMENTS`, its increment moves the new tool's TCP, and in the joint modes, only the Cartesian poses in the reply change.

The motion API only accepts increments.
So each cycle, MotoROS2 sends the difference between the position in the command and the position which the robot has been commanded to.
A fraction of a unit which can't be sent is made up for by the next command.
So is a lost command, or a slowdown of the robot (ex: by the FSU), in one cycle, as long as the next position is within one cycle's motion (see below).

In joint mode, the commanded position is read from the controller every cycle.
In Cartesian mode, MotoROS2 keeps track of the commanded pose itself, and only reads it from the controller again when the two differ by more than `rt_motion_deviation_threshold_micrometers` or `rt_motion_deviation_threshold_deg_0001`, or the tool changes.
(The pose the controller reports is computed from whole encoder pulses, but the commanded pose is usually between two pulses.
If MotoROS2 started from the reported pose every cycle, it would send that small difference again every cycle, until a joint stepped one pulse too far, and then back: the robot would step back and forth by a pulse while standing still.)
So after the robot was slowed down by less than these thresholds in total, it can stop up to that far short of the target.
Lower them if that is too much, but not below the TCP motion of about one encoder pulse.
In joint mode, a client which sends the commanded position from a reply back as its target does not move the robot.
In Cartesian mode, the pose in the reply is the one computed from whole encoder pulses, so after the robot has moved, it can differ from the pose MotoROS2 keeps track of by up to about one pulse (at the start of a session, they are the same).
Sending it back as the target can therefore move the robot by up to about one pulse.
To keep the robot still, send the same target again, rather than the pose from the reply.

In the absolute modes, the first command of a session is **not** executed (in the incremental modes, it is).
The reply to the first command reports the position the robot has been commanded to (`previousCommandPositionJoints` / `previousCommandPositionCartesian`), and the next command moves from there.
Each command after that must be within one cycle's motion of the position the robot has been commanded to: the same limits apply as for the increments (the maximum speed of each joint, 1500 mm/sec, and 360 deg/sec of rotation and of elbow angle).
A position which is further away ends the session with `InvalidIncrement`.
So every command must contain the position of every axis of every control group (except a group whose tool it changes in `CARTESIAN_ABSOLUTE`, see above), not only of those which move (ex: start from the positions in the reply to the first command): `0` is a position, not "don't move".

When the robot is slowed down (ex: by the FSU speed limit), it falls behind the commanded positions, and the next position may be further away than one cycle's motion.
That ends the session with `InvalidIncrement`.
So a client which plays back a trajectory planned in advance should slow its playback down when `motionDeviationDetected` is set, or when the commanded position in the reply falls behind, so that each position stays within one cycle's motion of the robot.
That keeps the robot on its path, and several control groups in step with each other (with an FSU speed limit on one of two robots, the controller slowed both of them down).
A client which computes each position from the commanded position in the reply is never more than one cycle behind.

`JOINT_ABSOLUTE` is not supported for robots whose B axis moves automatically to maintain the orientation of the tool (ex: MPL80/100).

### Data format (reply)

The reply packet is a *packed* `RtReply` structure.
This will echo the sequence ID, indicate whether the session is still active, report the state of the robot, provide feedback position, and provide commanded position.

Additionally, the `motionDeviationDetected` flag indicates that the controller did not execute the increments up to the **previous** command cycle as sent.
Ex: the Functional Safety Unit (FSU) speed limit truncated them, or the controller changed them near a singularity (see above).
The motion continues, but not as commanded.

MotoROS2 keeps the position the robot should be commanded to: a position read from the controller, plus every increment sent since.
Each cycle, it compares that with the commanded position.
So a deviation which is too small to see in a single cycle still adds up, until it passes the thresholds.
Then the flag is set for that reply, and the comparison starts again from the commanded position.
A steady deviation therefore sets the flag again every few cycles (every cycle, if it is larger than the thresholds per cycle).
Compare the commanded position in the reply with the expected one to see what happened.
The flag is set if any control group deviates: compare each group's row to see which one did.
When the FSU (or anything else) stops the robot, the session ends with `MotionRejected` instead (the flag may still be set in the last few cycles before that).
How far the motion may differ before the flag is set is configured with `rt_motion_deviation_threshold_pulses` (joint mode), and `rt_motion_deviation_threshold_micrometers` and `rt_motion_deviation_threshold_deg_0001` (Cartesian mode) in the configuration file.
In joint mode, the increments are sent in whole pulses, so the commanded position only differs from the expected one when the controller didn't execute them as sent.
The joint mode threshold is therefore `0` by default: any difference sets the flag.

```c
//##########################################################################
//                  !All data is little-endian!
//##########################################################################
struct RtReply
{
    UINT32 sequenceEcho;

    //See ReplyType enum.
    //When a session ends for any reason, a final reply of type SessionEnded
    //is sent. Its sequenceEcho is that of the last command which was
    //executed. Its position data is read when the session ends (with the
    //tools of that command).

    UINT32 replyType;

    //See EndReason enum. Only set when replyType is SessionEnded.

    UINT32 endReason;

    //Additional information about the endReason. See EndReason enum.

    int endDetail;

    //Bitfield of RobotStatusBits. This is read right after the previous reply
    //was sent, so it is one cycle old. (It is current in a SessionEnded reply.)
    //Any change which prevents motion will also end the session.

    UINT32 robotStatus;

    //This is indicative of where the robot is physically located.
    //Please note that this will trail behind the commanded position.
    //The joint ordering will match that of the command packet.
    //See Joints and Cartesian above.

    double feedbackPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double feedbackPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];

    //The position the robot has been commanded to: the sum of all the
    //increments, or (in the absolute control modes) the positions, as far
    //as the robot has followed them.
    //
    //This can be used to check whether the robot executed the
    //commands as sent (see motionDeviationDetected), and to monitor
    //the latency between command and feedback.

    double previousCommandPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double previousCommandPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];

    //Set when the controller did not execute the increments up to the
    //*previous* command cycle as sent (ex: the FSU speed limit truncated
    //them, or the controller changed them near a singularity). Small
    //deviations add up until they pass the configured thresholds. It
    //does NOT indicate anything about this most recent command packet.
    //Always false in a SessionEnded reply.

    bool motionDeviationDetected;
}
```

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
    InvalidVersion,       //endDetail: the version received
    InvalidPacketType,    //endDetail: the packetType received
    MultipleSources,      //A command was received from a second IP address. endDetail: that address (network byte order)
    SequenceGap,          //endDetail: the sequenceId received
    InvalidIncrement,     //Not a finite number, not a unit quaternion, exceeds a speed limit, or invalid toolIndex
    MotionRejected,       //endDetail: the return code of mpExRcsIncrementMove, or 0 if the robot stopped anyway or left REMOTE mode (ex: E-stop, HOLD)
    InternalError,        //See debug log
    CommandTooEarly,      //A command was sent before the reply to the previous one. endDetail: its sequenceId
}
```

`endDetail` is `0` for the reasons which don't list a value.

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
}
```

While a session is active, the robot is necessarily powered, in REMOTE mode, and free of E-stops and alarms (the session ends otherwise, see below).
An error (ex: a wrong password entered on the pendant) sets `Error`, but doesn't stop the robot, so the session goes on.
These bits are most useful in the `SessionEnded` reply, to determine why the robot rejected motion (`MotionRejected`).
The controller doesn't always reject the motion when the robot stops: on HOLD, `mpExRcsIncrementMove` can keep returning OK while the job is held, so every increment is accepted but not executed, even after HOLD is released (the job stays held).
So the session also ends with `MotionRejected` (`endDetail` `0`) when the status shows an E-stop, HOLD, an alarm or PFL, or the job is no longer running.
It also ends this way when the pendant leaves REMOTE mode (ex: it is switched to PLAY or TEACH): in PLAY mode, the controller keeps executing the increments, but MotoROS2 only allows motion in REMOTE mode.
The controller rejects the motion before its status shows why (ex: HOLD can take a few cycles to appear).
So after `MotionRejected`, the final reply waits until the cause appears in the status, for up to 200 milliseconds.
After an E-stop, the final reply can still show `DrivesPowered`: the controller stops the robot first, and switches the servo power off a moment later (about 0.5 seconds on a GP12).
For complete controller status (ex: alarm codes), use the [robot_status](ros_api.md#robot_status) topic.

If the session ends before the first command was accepted, the final reply is only sent if a packet was rejected, and it is sent to the host which sent that packet.
In that case, the position data is all zeros.

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
| `RtReply` | 2069 | `<IIIiI256d?` |

`MAX_GROUPS` and `MP_GRP_AXES_NUM` are both `8`.
Unused groups and axes are ignored (set them to `0`).
Each control group has its own row of `command` and of each array in the reply, and its own `toolIndex`.
The groups are in the controller's order: robots first (`R1`, `R2`, ...), then base axes (`B1`, ...), then stations (`S1`, ...).
This is the same order as the groups in `joint_states` (`group_1`, `group_2`, ...).

The reply is larger than the 1472 bytes of UDP data which fit in one standard Ethernet frame (MTU 1500), so it is always sent as two IP fragments.
The command packet fits in one frame.
Firewalls, NAT routers and other devices between the client and the controller must pass fragmented UDP: if they drop it, `start_rt_mode` succeeds, but the client never receives a reply (see [Firewall issues](network_configuration.md#firewall-issues)).
If either fragment is lost, the whole reply is lost.

## Deactivation

Other motion modes may not be used at the same time as an R/T session.
Starting one fails, and the session goes on: `start_traj_mode` and `start_point_queue_mode` return `NOT_READY_OTHER_TRAJ_MODE_ACTIVE (114)`, and a `FollowJointTrajectory` goal is aborted with `INIT_TRAJ_WRONG_MODE` (`error_code` `-100209`).
The R/T mode also stays active after a session has ended (ex: after a timeout), so they fail the same way until `stop_traj_mode` is called.
Calling `stop_traj_mode` ends the session (the final reply has `endReason` `StopRequested`).
At that time, another motion mode may be used.
The UDP port stays open, so a new session can be started with `start_rt_mode`.
