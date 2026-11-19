// RealTimeMotionControl.h

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_REALTIME_MOTION_CONTROL_H
#define MOTOROS2_REALTIME_MOTION_CONTROL_H

#define VERSION_REAL_TIME_INTERFACE 2

#define PACKED __attribute__ ((__packed__))

extern void Ros_RtMotionControl_RtIncMoveLoopStart(MOTION_MODE mode);
extern void Ros_RtMotionControl_OpenSocket();
extern BOOL Ros_RtMotionControl_IsSocketOpen();
extern void Ros_RtMotionControl_PrepareSession(RtSessionSettings const* settings);
extern BOOL Ros_RtMotionControl_IsSessionRunning();
extern BOOL Ros_RtMotionControl_StopSession();
extern void Ros_RtMotionControl_AbortSession();

typedef enum
{
    //Same values as motoros2_interfaces/msg/ControlModeEnum. 0 is not valid.
    PacketType_Joint_Increments = 1,
    PacketType_Cart_Increments = 2,
    PacketType_Joint_Absolute = 3,
    PacketType_Cart_Absolute = 4,
} PacketType;

typedef enum
{
    ReplyType_Feedback = 0,         //Reply to a command packet. The next command may be sent.
    ReplyType_SessionEnded,         //The session has ended (see EndReason). No more replies will be sent.
} ReplyType;

typedef enum
{
    EndReason_None = 0,
    EndReason_StopRequested,        //stop_traj_mode was called, or the micro-ROS agent disconnected
    EndReason_Timeout,              //No command was received within timeout_for_rt_msg
    EndReason_InvalidPacketSize,    //endDetail: the number of bytes received
    EndReason_InvalidVersion,       //endDetail: the version received
    EndReason_InvalidPacketType,    //endDetail: the packetType received
    EndReason_MultipleSources,      //A command was received from a second IP address. endDetail: that address (network byte order)
    EndReason_SequenceGap,          //The sequenceId skipped ahead, or the first one isn't 0. endDetail: the sequenceId received
    EndReason_InvalidIncrement,     //Not a finite number, not a unit quaternion, exceeds a speed limit, or invalid toolIndex. See debug log.
    EndReason_MotionRejected,       //endDetail: the return code of mpExRcsIncrementMove, or 0 if the robot stopped (ex: E-stop, HOLD), its drives are off, it left REMOTE mode, or the job stopped, after an accepted increment or while waiting for a command. -20 (energy saving mode) also when MotoROS2 didn't send the increment
    EndReason_InternalError,        //See debug log (ex: the robot's status couldn't be read)
    EndReason_CommandTooEarly,      //A command was sent before the reply to the previous one. endDetail: its sequenceId
} EndReason;

typedef enum
{
    RobotStatus_DrivesPowered   = (1 << 0),     //Servo power is on, and not in energy saving mode
    RobotStatus_EStopped        = (1 << 1),     //Pendant, external or controller E-stop
    RobotStatus_Hold            = (1 << 2),
    RobotStatus_Alarm           = (1 << 3),     //Major, minor, system or user alarm
    RobotStatus_Error           = (1 << 4),
    RobotStatus_PlayMode        = (1 << 5),     //Pendant is in PLAY or REMOTE mode (not TEACH)
    RobotStatus_PflActive       = (1 << 6),     //PFL has stopped the robot, or is escaping/avoiding (HC robots only)
    RobotStatus_RemoteMode      = (1 << 7),     //Pendant is in REMOTE mode
    RobotStatus_JobRunning      = (1 << 8),     //The job is running (not held or stopped). It stays stopped after a short HOLD is released.
} RobotStatusBits;

//The number of control groups in a packet. Unused groups are ignored.
#define MAX_GROUPS  8

typedef enum
{
    TCP_X = 0,      //meters
    TCP_Y,
    TCP_Z,

    TCP_Qx,         //quaternion
    TCP_Qy,
    TCP_Qz,
    TCP_Qw,

    TCP_Re,         //radians (elbow angle of 7-axis robots)

    MAX_AXES
} CartesianIndices;

//##########################################################################
//                  !All data is little-endian!
//##########################################################################

struct RtPacket_
{
    //The version of the command packet must match the value expected
    //by MotoROS2.
    int version;

    //The packet type must match the control_mode which was specified
    //when invoking the start_rt_mode service.
    PacketType packetType;

    //Must increment sequentially with each new command packet. The
    //first command of a session must be 0.
    UINT32 sequenceId;

    //The positions to move to (absolute control modes), or the
    //increments (incremental control modes).
    //
    //For joint-space, this will be radians (or meters for linear axes)
    //of each joint, in the order selected by the joint_order field of
    //the start_rt_mode service. See doc/rt_control.md (Joints).
    //
    //For cartesian, this will be meters and quaternion of the TCP.
    //The order must be [X Y Z Qx Qy Qz Qw Re].
    //See CartesianIndices enum.
    double command[MAX_GROUPS][MP_GRP_AXES_NUM];

    //The tool of each control group. Cartesian commands, and the Cartesian
    //poses in the reply, are for this tool's TCP. The select_motion_tool
    //service doesn't affect R/T sessions.
    int toolIndex[MAX_GROUPS]; //TOOL 0 - 63

    //Reserved for future expansion. Set it to 0: a later version may use it.
    char reserved[64];

} PACKED;
typedef struct RtPacket_ RtPacket;


//##########################################################################
//                  !All data is little-endian!
//##########################################################################

struct RtReply_
{
    UINT32 sequenceEcho;

    //See ReplyType enum.
    //When a session ends for any reason, a final reply of type ReplyType_SessionEnded is
    //sent, so the client doesn't have to wait for a timeout. Its sequenceEcho is that of the
    //last command which was executed. Its position data is read when the session ends (with
    //the tools of that command). This is best-effort (UDP), so the client should still treat
    //a missing reply as the end of the session.
    UINT32 replyType;

    //See EndReason enum. Only set when replyType is ReplyType_SessionEnded.
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
    //See doc/rt_control.md (Joints) and the CartesianIndices enum.
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
} PACKED;
typedef struct RtReply_ RtReply;


#undef PACKED

#endif //MOTOROS2_REALTIME_MOTION_CONTROL_H
