<!--
SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
SPDX-FileCopyrightText: 2025, Delft University of Technology

SPDX-License-Identifier: CC-BY-SA-4.0
-->

# R/T Motion Control

The real-time motion control server is intended to be used in a closed loop system.
It allows the user to command incremental offsets at the rate of the robot controller's interpolation clock.
This control mode minimizes overhead as much as possible by routing the user commands directly to the MotoPlus motion API, mpExRcsIncrementMove.

Note: R/T control has not been tested with seven axis robots (including the elbow angle `Re`), or with robots with fewer than six axes (four and five axis arms, and SCARA robots).
On five axis robots whose B axis moves automatically to keep the tool level (ex: MPL80/100), a joint increment for B may be relative to that automatic motion, rather than to the physical joint.

## Activation

This control mode is activated using the [start_rt_mode](ros_api.md#start_rt_mode) service.
The user must specify the `control_mode` to indicate whether the increments will be joint offsets (radians) or cartesian TCP offsets (meters / quaternion).
Cartesian mode is only supported when every control group is a robot (no positioners or base tracks).

If this service is successful, it will return a `result_code` of `Ready (1)`.
Otherwise, please examine the `result_code` and `message` fields in the response for more information.

The service will also return a `period` in milliseconds.
This indicates the rate at which increment commands will be expected by the robot.
The default period for a single manipulator is 4 milliseconds.
However, that value will increase as additional axes or manipulators are added to the system.

## Usage

### Command Flow

The UDP port (`22000` by default, set by `rt_listener_udp_port_number`) is opened when MotoROS2 starts.
The user then sends the first command with the `sequenceId` field set to `0`.
It is recommended that the first command contains zero increments, as the client does not yet know the position of the robot.
After that, the user must wait until the robot replies before sending the next increment.
Each subsequent command must increment the `sequenceId`. Additionally, each subsequent command must not be sent until the robot replies to the previous command.
This will occur at the rate of the `period` from the [start_rt_mode](ros_api.md#start_rt_mode) service.

The reply is sent right after the increment has been passed to the robot's interpolation cycle.
To keep the motion smooth, the next command must arrive in time to be processed before the following interpolation cycle.
That is less than one `period` after the reply, because the reply is only sent once the cycle has started (on a GP12 with a 4 millisecond `period`, a command which arrived more than about 2.3 milliseconds after the reply missed the next cycle).
So send the next command as soon as possible after the reply.
If it arrives late, the robot does not move during that cycle.

If a command is not received within `timeout_for_rt_msg` (default: 5 seconds; `-1`: never), then the session times out and is dropped.
At that point, a new session must be started by calling `start_rt_mode` again.
A "keep-alive" can be used by sending a command with zero increments.

The session is also dropped if:

- a command has the wrong size, `version` or `packetType`
- a command is received from an IP address other than the one which sent the first valid command (that host owns the session)
- the `sequenceId` skips ahead by more than `max_sequence_diff_for_rt_msg` (default: `3`)
- an increment is invalid (not a finite number, not a unit quaternion) or exceeds the speed limits
- the robot rejects the increment (ex: E-stop, HOLD, alarm, PFL, servo power off)
- `stop_traj_mode` is called, or the micro-ROS agent disconnects

The R/T port accepts packets on every network interface of the controller, and there is no authentication.
The session belongs to whichever host sends the first valid command after `start_rt_mode`, which doesn't have to be the host that called `start_rt_mode`.
While a session is running, any packet from another IP address ends it (`MultipleSources`), even one which isn't an R/T command (ex: from a network scanner).
Before the first command, such a packet ends it as an invalid command.
So make sure that no other host sends packets to the R/T port while a session is running (ex: block the port for other hosts with a firewall in front of the controller).

When the session ends, a final reply is sent with `replyType` set to `SessionEnded`, and `endReason` / `endDetail` indicating why.
See [Session end](#session-end).
More detail is printed to the debug log.

MotoROS2 doesn't decelerate the robot: when commands stop arriving, or the session ends, no more motion is sent.
So bring the robot to rest before ending a session (ex: before calling `stop_traj_mode`).
The controller smooths the motion (on a GP12, the feedback trails the command by about 0.2 seconds), so the robot still slows down over a few tenths of a second, but that can still be too abrupt (ex: at high speed, or with a payload).

The final reply is best-effort, as it is sent over UDP.
Therefore, if the client does not receive a reply packet within `timeout_for_rt_msg` (returned by `start_rt_mode`), then it should still assume that the session is dead.

The motion API only accepts whole units (pulses, micrometers and 0.0001 degrees).
Increments are rounded to the nearest whole unit, and the rest is carried over to the next cycle, so small increments are not lost.
A joint increment may exceed the joint's maximum per cycle by up to 0.001 pulse (ex: from rounding in the client's unit conversion).
It is still sent within the maximum, and the excess is sent in a later cycle.

<img src="img/RtFlow.png" alt="Command Flow" />

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

    //The order of the joints must be in the order of [S L U R B T E 8].
    //Please note that for seven axis robots, the 'E' joint is physically
    //mounted in the middle of the arm. But it must be sent at the end
    //of the joint array. See JointIndices enum.
    //
    //For joint-space, this will be radians of each joint.
    //
    //For cartesian, this will be meters and quaternion of the TCP.
    //The order of the joints must be in the order of [X Y Z Qx Qy Qz Qw Re].
    //See CartesianIndices enum.

    double delta[MAX_GROUPS][MP_GRP_AXES_NUM];

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

#### Joints

When the `control_mode` is `JOINT_ANGLES (1)`, the order of the joints in the `delta` array must be in the order of `S L U R B T E 8`.
Please note that for seven axis robots, the `E` joint is physically mounted in the middle of the arm.
But it must be sent at the end of the joint array.
This is different from the order used by the `joint_states` topic, which follows the physical order of the joints.

Linear axes (ex: base tracks) are in meters.

See `JointIndices` enum.

```c
enum JointIndices
{
    Joint_S = 0,    //radians
    Joint_L,
    Joint_U,
    Joint_R,
    Joint_B,
    Joint_T,
    Joint_E,
    Joint_8,

    MAX_JOINTS
}
```

#### Cartesian

When the `control_mode` is `CARTESIAN (2)`, the order of the joints in the `delta` array must be in order of `X Y Z Qx Qy Qz Qw Re`.

The increments are expressed in the robot coordinate frame (RF), and are applied to the tool selected by `toolIndex`.
The rotation must be a unit quaternion: a length within `0.01` of `1` is accepted, and normalized.
No rotation is `Qx Qy Qz Qw` = `0 0 0 1`.
An all-zero quaternion is also accepted as "no rotation", so a zero-filled command can be used as a keep-alive.
`Re` is the change in elbow angle of seven axis robots. It is ignored for other robots.

See `CartesianIndices` enum.

```c
enum CartesianIndices
{
    TCP_X = 0,      //meters
    TCP_Y,
    TCP_Z,

    TCP_Qx,         //quaternion
    TCP_Qy,
    TCP_Qz,
    TCP_Qw,

    TCP_Re,         //radians

    MAX_AXES
}
```

### Data format (reply)

The reply packet is a *packed* `RtReply` structure.
This will echo the sequence ID, indicate whether the session is still active, report the state of the robot, provide feedback position, and provide commanded position.

Additionally, the `motionDeviationDetected` flag indicates that the controller did not execute the increment of the **previous** command cycle as sent.
Ex: the Functional Safety Unit (FSU) speed limit truncated it, or the controller changed it near a singularity.
The motion continues, but not as commanded.
Compare the commanded position in the reply with the expected one to see what happened.

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
    //The joint ordering will match that of the original command
    //packet. See JointIndices and CartesianIndices enums.

    double feedbackPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double feedbackPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];

    //The command position is the target destination you are instructing
    //the robot to reach. It's the calculated endpoint based on the sum
    //of all position increments received from the user.
    //
    //This can be used to check whether the robot executed the
    //increments as sent (see motionDeviationDetected), and to monitor
    //the latency between command and feedback.

    double previousCommandPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double previousCommandPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];

    //Set when the controller did not execute the *previous* command
    //cycle's increment as sent (ex: the FSU speed limit truncated it, or
    //the controller changed it near a singularity). It does NOT indicate
    //anything about this most recent command packet.

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
    InvalidIncrement,     //Not a finite number, not a unit quaternion, or exceeds a speed limit
    MotionRejected,       //endDetail: the return code of mpExRcsIncrementMove (ex: E-stop, HOLD, alarm, PFL)
    InternalError,        //See debug log
}
```

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

While a session is active, the robot is necessarily powered, in REMOTE mode, and free of E-stops, alarms and errors.
These bits are most useful in the `SessionEnded` reply, to determine why the robot rejected motion (`MotionRejected`).
For complete controller status (ex: alarm codes), use the [robot_status](ros_api.md#robot_status) topic.

If the session ends before any command has been executed, the final reply is only sent if a packet was rejected, and it is sent to the host which sent that packet.
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
Each control group has its own row of `delta` and of each array in the reply, and its own `toolIndex`.
The groups are in the controller's order: robots first (`R1`, `R2`, ...), then base axes (`B1`, ...), then stations (`S1`, ...).
This is the same order as the groups in `joint_states` (`group_1`, `group_2`, ...).

The reply is larger than the 1472 bytes of UDP data which fit in one standard Ethernet frame (MTU 1500), so it is always sent as two IP fragments.
The command packet fits in one frame.
Firewalls, NAT routers and other devices between the client and the controller must pass fragmented UDP: if they drop it, `start_rt_mode` succeeds, but the client never receives a reply.
If either fragment is lost, the whole reply is lost.

## Deactivation

Other motion modes may not be used at the same time as an R/T session.
The service to start those modes will fail when invoked.
Calling `stop_traj_mode` ends the session (the final reply has `endReason` `StopRequested`).
At that time, another motion mode may be used.
The UDP port stays open, so a new session can be started with `start_rt_mode`.
