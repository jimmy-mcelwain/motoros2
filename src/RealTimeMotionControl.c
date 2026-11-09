//RealTimeMotionControl.c

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

// Based loosely on @adv4ncr's modifications to MotoROS1 for real-time control through ROS2.
// https://github.com/adv4ncr/motoman_ROS2/blob/cdf63a592596ff711df842680a1e4c730fd547a7/controller_driver/RealTimeMotionServer.c

#include "MotoROS.h"

//The sizes of the packets, as documented in doc/rt_control.md (Wire format). Clients rely on them.
//(Fails to compile otherwise: an array can't have a negative size.)
typedef char RtPacketSizeIsAsDocumented[(sizeof(RtPacket) == 620) ? 1 : -1];
typedef char RtReplySizeIsAsDocumented[(sizeof(RtReply) == 2069) ? 1 : -1];

//The packetType of a command must be the same as the control_mode of the session.
//(Fails to compile otherwise: an array can't have a negative size.)
typedef char RtPacketTypeMatchesControlMode[
    (PacketType_Joint_Increments == motoros2_interfaces__msg__ControlModeEnum__JOINT_INCREMENTS &&
     PacketType_Cart_Increments == motoros2_interfaces__msg__ControlModeEnum__CARTESIAN_INCREMENTS &&
     PacketType_Joint_Absolute == motoros2_interfaces__msg__ControlModeEnum__JOINT_ABSOLUTE &&
     PacketType_Cart_Absolute == motoros2_interfaces__msg__ControlModeEnum__CARTESIAN_ABSOLUTE) ? 1 : -1];

static void Ros_RtMotionControl_InitJointSpace(MP_EXPOS_DATA* moveData);
static bool Ros_RtMotionControl_InitCartesian(MP_EXPOS_DATA* moveData);
static bool Ros_RtMotionControl_ParseJointSpace(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData);
static bool Ros_RtMotionControl_ParseCartesian(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData);
static void Ros_RtMotionControl_StartAbsolute(MOTION_MODE mode, RtPacket* incomingCommand, MP_EXPOS_DATA* moveData);
static bool Ros_RtMotionControl_ParseJointAbsolute(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData);
static bool Ros_RtMotionControl_ParseCartesianAbsolute(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData);
static bool Ros_RtMotionControl_AreToolsValid(RtPacket* incomingCommand);
static void Ros_RtMotionControl_PopulateReplyMessage(MOTION_MODE mode, RtPacket* command, RtReply* reply);
static bool Ros_RtMotionControl_CheckForMotionDeviation(MOTION_MODE mode, int* tools, bool* pbDeviation);
static void Ros_RtMotionControl_AddToExpectedPosition(MOTION_MODE mode, MP_EXPOS_DATA* moveData);
static void Ros_RtMotionControl_PurgeBufferedPackets();
static UINT32 Ros_RtMotionControl_ReadRobotStatus();
static UINT32 Ros_RtMotionControl_WaitForStopCause();
static BOOL Ros_RtMotionControl_IsJobRunning();
static EndReason Ros_RtMotionControl_DiscardEarlyPackets(const struct sockaddr_in* commander, UINT32 executedSequenceId, int* endDetail);
static EndReason Ros_RtMotionControl_MultipleSources(const struct sockaddr_in* commander, const struct sockaddr_in* sender, int* endDetail);
static int Ros_RtMotionControl_WaitForPacket();
static double Ros_RtMotionControl_RotationAngle(const Quaternion* from, const Quaternion* to);

//Timing of one R/T cycle, in system clock ticks. Used to find out where a cycle loses time.
typedef struct
{
    ULONG received;     //command received
    ULONG moved;        //increment passed to the motion API
    ULONG tick;         //interpolation clock tick
    ULONG populated;    //reply message filled in
    ULONG discarded;    //early packets discarded
    ULONG replied;      //reply sent
} RtCycleTimes;

//A cycle which lost a tick or replied late. Kept until the session ends, and only logged then,
//so logging doesn't take time from the cycles being measured. The sequenceId lets the client
//match it to its own log.
typedef struct
{
    UINT32 sequenceId;
    ULONG sincePrevTick;    //the previous cycle's tick to this cycle's tick
    ULONG waitForCommand;   //the previous reply to this command
    ULONG processing;       //this command to the motion API
    ULONG waitForTick;      //the motion API to the tick
    ULONG buildingReply;    //the tick to this reply. The sum of the next three:
    ULONG populating;       //  the tick to the reply message being filled in
    ULONG discarding;       //  discarding early packets
    ULONG sending;          //  sending the reply
} RtTimingEvent;

#define RT_TIMING_MAX_EVENTS    32

typedef struct
{
    UINT32 cycles;
    UINT32 lostTickCycles;  //cycles which started more than one tick after the previous one
    UINT32 lateReplies;     //replies sent more than half a period after the tick
    UINT32 problemCycles;   //either of the above
    int numEvents;
    RtTimingEvent events[RT_TIMING_MAX_EVENTS];
} RtTimingStats;

static void Ros_RtMotionControl_CheckCycleTiming(UINT32 sequenceId, const RtCycleTimes* prev, const RtCycleTimes* now, RtTimingStats* stats);
static void Ros_RtMotionControl_LogCycleTiming(const RtTimingStats* stats);

//How often the R/T task checks for a stop request while waiting for a command packet
#define RT_STOP_POLL_PERIOD_MS      100
//How long to wait for the R/T task to exit on its own before it gets deleted
#define RT_STOP_WAIT_TIMEOUT_MS     1000
//How long to wait for the cause of a rejected increment to appear in the robot's status
#define RT_STOP_CAUSE_WAIT_MS       200
//The pendant modes the session requires. Like Ros_Controller_GetNotReadySubcode, REMOTE mode is not
//required in DUMMY_SERVO_MODE builds.
#ifndef DUMMY_SERVO_MODE
#define RT_REQUIRED_MODES           (RobotStatus_PlayMode | RobotStatus_RemoteMode)
#else
#define RT_REQUIRED_MODES           (RobotStatus_PlayMode)
#endif
//The status bits which show that the robot stopped
#define RT_STOPPED_STATUS           (RobotStatus_EStopped | RobotStatus_Hold | RobotStatus_Alarm | RobotStatus_PflActive)

//The session ends when the task leaves its command loop. The task then still sends the final
//reply and writes its log, so it keeps running for a short time after the session has ended.
static volatile BOOL bRtSessionRunning = FALSE;
static volatile BOOL bRtTaskRunning = FALSE;
static volatile BOOL bRtStopRequested = FALSE;

//The order of the joints in the packets. See motoros2_interfaces/msg/JointOrderEnum.
//Internally, everything is in Motoman order. Set by Ros_RtMotionControl_PrepareSession.
static UINT16 sessionJointOrder = motoros2_interfaces__msg__JointOrderEnum__JOINT_STATES;

//Whether the packets contain positions (absolute) or increments. Set like the joint order.
static BOOL sessionAbsolute = FALSE;

static int sockRtCommandListener = -1;

//Joint mode: the commanded pulses, as read this cycle by the deviation check
static LONG prevRtCmdPosition[MAX_GROUPS][MP_GRP_AXES_NUM];

//The position the robot should have been commanded to: a position read from the controller,
//plus every increment sent since. The deviation check compares it with the position read each
//cycle, so a loss which is too small to see in a single cycle still adds up until it passes the
//thresholds. Then the flag is set, and the position read becomes the new starting point. (In the
//incremental modes, motion which wasn't executed isn't made up, so the difference would never
//shrink again.)
//The Cartesian pose read from the controller is forward kinematics of the commanded joint pulses,
//which are whole numbers, so it can differ from this by up to about one pulse of each joint. The
//difference doesn't accumulate: the controller carries the fraction of a pulse it hasn't moved yet
//over to the next increment. (On a GP12, 2500 increments of 1 um ended 1 um from their sum.)
static LONG expectedPulses[MAX_GROUPS][MP_GRP_AXES_NUM];  //joint mode, Motoman order
static bool bExpectedPulsesRead;                          //joint mode: false until the first command
static double expectedTranslation[MAX_GROUPS][3];         //Cartesian mode (robot frame), micrometers
static Quaternion expectedOrientation[MAX_GROUPS];
static double expectedElbow[MAX_GROUPS];                  //0.0001 degrees
//Cartesian mode: the tool of the expected pose (-1 = not read yet). A TCP pose is only comparable
//with another pose of the same tool.
static int expectedTool[MAX_GROUPS];

//The motion API only accepts whole units (pulse, micrometer, 0.0001 degree). The fraction
//of a unit which was commanded but not sent is carried over to the next cycle. Otherwise the
//truncation would accumulate into a steady drift when streaming small increments.
static double residualIncrement[MAX_GROUPS][MP_GRP_AXES_NUM];

//Task to receive increments from the R/T client and pass them to the motion API
void Ros_RtMotionControl_RtIncMoveLoopStart(MOTION_MODE mode) //<-- IP_CLK priority task
{
    MP_EXPOS_DATA moveData;
    int bytes_received;

    bool bFirstRecv = true;
    struct sockaddr_in client_addr;
    struct sockaddr_in previous_client_addr;
    int client_addr_len = sizeof(client_addr);

    //One byte larger than RtPacket. UDP truncates a datagram to fit the buffer, so an
    //oversized packet would otherwise be received as exactly sizeof(RtPacket) bytes.
    char receiveBuffer[sizeof(RtPacket) + 1];

    RtPacket incomingCommand;
    RtPacket lastExecutedCommand;
    RtReply outgoingReply;

    EndReason endReason = EndReason_InternalError;
    int endDetail = 0;
    bool bEndReplyToSender = false; //the session ended because of the packet which was just received

    UINT32 previousSequenceId = 0;

    bool motionDeviationDetected;

    bool packetTypeOK;

    UINT32 robotStatus;

    RtCycleTimes cycleTimes;
    RtCycleTimes prevCycleTimes;
    bool bHavePrevCycle = false;
    RtTimingStats timingStats;

    //=========================================================================================

    bzero(prevRtCmdPosition, sizeof(prevRtCmdPosition));
    bzero(residualIncrement, sizeof(residualIncrement));

    if (mode == MOTION_MODE_RT_JOINT)
        Ros_RtMotionControl_InitJointSpace(&moveData);
    else
    {
        if (!Ros_RtMotionControl_InitCartesian(&moveData))
        {
            bRtSessionRunning = FALSE;
            bRtTaskRunning = FALSE;
            mpSuspendSelf;
            return;
        }
    }

    Ros_Debug_BroadcastMsg("Starting RT session");

    bzero(&timingStats, sizeof(timingStats));
    Ros_Debug_BroadcastMsg("R/T timing: the system clock ticks every %.3f ms", (float)mpGetRtc());

    robotStatus = Ros_RtMotionControl_ReadRobotStatus();

    //=========================================================================================
    while (TRUE)
    {
        int waitResult = Ros_RtMotionControl_WaitForPacket();
        if (waitResult > 0)
        {
            client_addr_len = sizeof(client_addr);
            bytes_received = mpRecvFrom(sockRtCommandListener, receiveBuffer, sizeof(receiveBuffer), 0, (struct sockaddr*)&client_addr, &client_addr_len);
            cycleTimes.received = tickGet();
            if (bytes_received < 0)
            {
                Ros_Debug_BroadcastMsg("ERROR: recvFrom returned an error");
                endReason = EndReason_InternalError;
                break;
            }

            //Only allow a single commander. A second sender means the system is misconfigured,
            //so the session is ended rather than letting two clients command the robot.
            if (!bFirstRecv && client_addr.sin_addr.s_addr != previous_client_addr.sin_addr.s_addr)
            {
                endReason = Ros_RtMotionControl_MultipleSources(&previous_client_addr, &client_addr, &endDetail);
                break; //drop the connection
            }

            if (bytes_received != sizeof(RtPacket))
            {
                Ros_Debug_BroadcastMsg("ERROR: Received a packet of [%d] bytes. RtPacket must be [%d] bytes.",
                                        bytes_received, (int)sizeof(RtPacket));
                endReason = EndReason_InvalidPacketSize;
                endDetail = bytes_received;
                bEndReplyToSender = true;
                break; //drop the connection
            }
            memcpy(&incomingCommand, receiveBuffer, sizeof(RtPacket));

            //Verify version of the command packet
            if (incomingCommand.version != VERSION_REAL_TIME_INTERFACE)
            {
                Ros_Debug_BroadcastMsg("ERROR: The command packet is version [%d]. It must be version [%d]",
                                        incomingCommand.version, VERSION_REAL_TIME_INTERFACE);
                endReason = EndReason_InvalidVersion;
                endDetail = incomingCommand.version;
                bEndReplyToSender = true;
                break; //drop the connection
            }

            //Verify the packet type
            packetTypeOK = TRUE;
            switch (mode)
            {
            case MOTION_MODE_RT_JOINT:
                if (incomingCommand.packetType != (sessionAbsolute ? PacketType_Joint_Absolute : PacketType_Joint_Increments))
                {
                    packetTypeOK = false;
                    Ros_Debug_BroadcastMsg("ERROR: The packet type does not match the control_mode specified in start_rt_mode (%s)",
                        sessionAbsolute ? "Joint Absolute" : "Joint Increments");
                }
                break;
            case MOTION_MODE_RT_CARTESIAN:
                if (incomingCommand.packetType != (sessionAbsolute ? PacketType_Cart_Absolute : PacketType_Cart_Increments))
                {
                    packetTypeOK = false;
                    Ros_Debug_BroadcastMsg("ERROR: The packet type does not match the control_mode specified in start_rt_mode (%s)",
                        sessionAbsolute ? "Cartesian Absolute" : "Cartesian Increments");
                }
                break;
            default:
                packetTypeOK = false;
                Ros_Debug_BroadcastMsg("ERROR: Unexpected motion mode is active");
            }
            if (!packetTypeOK)
            {
                endReason = EndReason_InvalidPacketType;
                endDetail = incomingCommand.packetType;
                bEndReplyToSender = true;
                break; //drop the connection
            }

            //Only the IP address identifies the commander. The port can change if the client
            //reopens its socket, so keep the latest one for the final reply.
            previous_client_addr = client_addr;

            //Check for old or same sequence ID (wraparound safe)
            if (((int32_t)(incomingCommand.sequenceId - previousSequenceId) <= 0) && !bFirstRecv)
            {
                // This packet is old or a duplicate.
                Ros_Debug_BroadcastMsg("WARN: Received old command packet (prev: %u, new: %u)", previousSequenceId, incomingCommand.sequenceId);

                // Resend the previous reply and drop this packet.
                mpSendTo(sockRtCommandListener, (char*)&outgoingReply, sizeof(RtReply), 0, (struct sockaddr*)&client_addr, client_addr_len);
                continue;
            }

            //Check if the sequence ID jumped too far ahead (wraparound safe)
            if ((incomingCommand.sequenceId - previousSequenceId) > g_nodeConfigSettings.max_sequence_diff_for_rt_msg)
            {
                Ros_Debug_BroadcastMsg("ERROR: Missed too many command packets (prev: %u, new: %u)",
                    previousSequenceId, incomingCommand.sequenceId);
                endReason = EndReason_SequenceGap;
                endDetail = (int)incomingCommand.sequenceId;
                bEndReplyToSender = true;
                break; // Drop the connection
            }

            //The deviation check reads the pose with these tools
            if (!Ros_RtMotionControl_AreToolsValid(&incomingCommand))
            {
                endReason = EndReason_InvalidIncrement;
                bEndReplyToSender = true;
                break; //drop the connection
            }

            //Check the previous increment before parsing this command. The absolute modes start
            //from the position it reads.
            if (!Ros_RtMotionControl_CheckForMotionDeviation(mode, incomingCommand.toolIndex, &motionDeviationDetected))
            {
                endReason = EndReason_InternalError;
                bEndReplyToSender = true;
                break; //drop the connection
            }

            bool bParsedOK = true;
            if (sessionAbsolute && bFirstRecv)
                Ros_RtMotionControl_StartAbsolute(mode, &incomingCommand, &moveData);
            else if (mode == MOTION_MODE_RT_JOINT)
            {
                bParsedOK = sessionAbsolute ? Ros_RtMotionControl_ParseJointAbsolute(&incomingCommand, &moveData) :
                                              Ros_RtMotionControl_ParseJointSpace(&incomingCommand, &moveData);
            }
            else
            {
                bParsedOK = sessionAbsolute ? Ros_RtMotionControl_ParseCartesianAbsolute(&incomingCommand, &moveData) :
                                              Ros_RtMotionControl_ParseCartesian(&incomingCommand, &moveData);
            }

            if (!bParsedOK)
            {
                endReason = EndReason_InvalidIncrement;
                bEndReplyToSender = true;
                break; //drop the connection
            }

            // Send increment to robot
            int ret = mpExRcsIncrementMove(&moveData);
            cycleTimes.moved = tickGet();
            if (ret != OK)
            {
                Ros_Debug_BroadcastMsg("WARN: mpExRcsIncrementMove returned %d", ret);
                endReason = EndReason_MotionRejected;
                endDetail = ret;
                bEndReplyToSender = true;
                break; //drop the connection
            }

            bFirstRecv = false;
            lastExecutedCommand = incomingCommand;

            // Wait for next interpolation cycle
            mpClkAnnounce(MP_INTERPOLATION_CLK);
            cycleTimes.tick = tickGet();

            //send status back to the PC and notify it that I'm ready for another packet
            previousSequenceId = incomingCommand.sequenceId;
            Ros_RtMotionControl_PopulateReplyMessage(mode, &incomingCommand, &outgoingReply);
            outgoingReply.replyType = ReplyType_Feedback;
            outgoingReply.endReason = EndReason_None;
            outgoingReply.robotStatus = robotStatus;
            outgoingReply.motionDeviationDetected = motionDeviationDetected;
            cycleTimes.populated = tickGet();

            //Discard anything which arrived while waiting for the interpolation clock. This must
            //happen before the reply is sent, so a client which waits for the reply can't be affected.
            EndReason discardReason = Ros_RtMotionControl_DiscardEarlyPackets(&previous_client_addr, incomingCommand.sequenceId, &endDetail);
            if (discardReason != EndReason_None)
            {
                endReason = discardReason;
                break; //drop the connection
            }
            cycleTimes.discarded = tickGet();

            mpSendTo(sockRtCommandListener, (char*)&outgoingReply, sizeof(RtReply), 0, (struct sockaddr*)&client_addr, client_addr_len);
            cycleTimes.replied = tickGet();

            if (bHavePrevCycle)
                Ros_RtMotionControl_CheckCycleTiming(incomingCommand.sequenceId, &prevCycleTimes, &cycleTimes, &timingStats);
            prevCycleTimes = cycleTimes;
            bHavePrevCycle = true;

            //Read the status for the next reply now, while the client is computing its next
            //command, so it doesn't delay the reply.
            robotStatus = Ros_RtMotionControl_ReadRobotStatus();

            //The controller doesn't always reject the increment when the robot stops. (ex: On
            //HOLD, it can accept the increment and hold the job anyway. It then accepts every
            //increment and ignores it, even after HOLD is released.) So the session also ends
            //when the status shows a stop, or the job isn't running. (PFL can stop the robot while the
            //job stays on its WAIT.)
            //The controller keeps executing increments after the pendant leaves REMOTE mode, but
            //MotoROS2 only allows motion in REMOTE mode. So the session ends then too.
            if ((robotStatus & RT_STOPPED_STATUS) ||
                ((robotStatus & RT_REQUIRED_MODES) != RT_REQUIRED_MODES) || !Ros_RtMotionControl_IsJobRunning())
            {
                Ros_Debug_BroadcastMsg("WARN: The robot stopped or left PLAY/REMOTE mode, but mpExRcsIncrementMove accepted the increment (status 0x%02X)", robotStatus);
                endReason = EndReason_MotionRejected;
                endDetail = 0;
                break; //drop the connection
            }

            //Next cycle, the deviation check compares the commanded position with this
            Ros_RtMotionControl_AddToExpectedPosition(mode, &moveData);
        }
        else if (waitResult < 0)
        {
            Ros_Debug_BroadcastMsg("ERROR: Waiting for a command packet failed (mpSelect returned %d)", waitResult);
            endReason = EndReason_InternalError;
            break;
        }
        else
        {
            if (bRtStopRequested)
            {
                Ros_Debug_BroadcastMsg("Stop requested");
                endReason = EndReason_StopRequested;
            }
            else
            {
                Ros_Debug_BroadcastMsg("No packets received for %d milliseconds", g_nodeConfigSettings.timeout_for_rt_msg);
                endReason = EndReason_Timeout;
            }
            break;
        }
    }

    //=========================================================================================
    //The session ends here, before the client is told. A client may call start_rt_mode as soon
    //as it gets the final reply, and that must not be rejected because this task hasn't
    //returned yet. (start_rt_mode waits for it. See Ros_RtMotionControl_StopSession.)
    bRtSessionRunning = FALSE;

    //Tell the client why the session ended, so it doesn't have to wait for a timeout.
    //Once there is a commander, only it is told. (client_addr may belong to another sender.)
    //Before that, only the sender of a rejected first packet is told.
    if (!bFirstRecv || bEndReplyToSender)
    {
        struct sockaddr_in* endDestination = bFirstRecv ? &client_addr : &previous_client_addr;

        if (bFirstRecv)
            bzero(&outgoingReply, sizeof(outgoingReply)); //nothing has been executed yet
        else
            Ros_RtMotionControl_PopulateReplyMessage(mode, &lastExecutedCommand, &outgoingReply);

        outgoingReply.replyType = ReplyType_SessionEnded;
        outgoingReply.endReason = endReason;
        outgoingReply.endDetail = endDetail;
        if (endReason == EndReason_MotionRejected)
            outgoingReply.robotStatus = Ros_RtMotionControl_WaitForStopCause();
        else
            outgoingReply.robotStatus = Ros_RtMotionControl_ReadRobotStatus();

        mpSendTo(sockRtCommandListener, (char*)&outgoingReply, sizeof(RtReply), 0, (struct sockaddr*)endDestination, sizeof(struct sockaddr_in));
    }

    Ros_Debug_BroadcastMsg("Ending Rt Session (reason: %d)", endReason);
    Ros_RtMotionControl_LogCycleTiming(&timingStats);

    //Ros_RtMotionControl_StopSession waits for this
    bRtTaskRunning = FALSE;
    mpSuspendSelf;
}

//Waits for a command packet to arrive, while periodically checking whether
//the session has been asked to stop. This allows the task to exit on its own,
//instead of being deleted while it is blocked on the socket.
//
//Returns >0 if a packet is available, 0 on timeout or stop request, and <0 on error.
static int Ros_RtMotionControl_WaitForPacket()
{
    struct fd_set fds;
    struct timeval tv;
    int remaining_ms = g_nodeConfigSettings.timeout_for_rt_msg; //-1 never times out
    int slice_ms;
    int ret;

    while (!bRtStopRequested)
    {
        slice_ms = RT_STOP_POLL_PERIOD_MS;
        if (remaining_ms != -1 && remaining_ms < slice_ms)
            slice_ms = remaining_ms;

        FD_ZERO(&fds);
        FD_SET(sockRtCommandListener, &fds);

        tv.tv_usec = (slice_ms % 1000) * 1000;
        tv.tv_sec = slice_ms / 1000;

        ret = mpSelect(sockRtCommandListener + 1, &fds, NULL, NULL, &tv);
        if (ret != 0)
            return ret;

        if (remaining_ms != -1)
        {
            remaining_ms -= slice_ms;
            if (remaining_ms <= 0)
                return 0;
        }
    }

    return 0;
}

//Must be called before the R/T task is created, and only then, so the settings can't change in
//the middle of a session
void Ros_RtMotionControl_PrepareSession(RtSessionSettings const* settings)
{
    //Discard stale packets here, before start_rt_mode responds, rather than in the R/T task.
    //Otherwise a client which sends its first command as soon as it gets the response could
    //have it discarded, depending on how soon the task gets to run.
    Ros_Debug_BroadcastMsg("Flushing stale packets from socket buffer...");
    Ros_RtMotionControl_PurgeBufferedPackets();

    sessionJointOrder = settings->jointOrder;
    sessionAbsolute = settings->bAbsolute;
    bRtStopRequested = FALSE;
    bRtSessionRunning = TRUE;
    bRtTaskRunning = TRUE;
}

//FALSE as soon as the session has ended. Its task may not have returned yet:
//Ros_RtMotionControl_StopSession waits for that.
BOOL Ros_RtMotionControl_IsSessionRunning()
{
    return bRtSessionRunning;
}

//Asks the R/T task to exit and waits for it to do so.
//Returns FALSE if the task did not exit in time.
BOOL Ros_RtMotionControl_StopSession()
{
    int waited_ms;

    bRtStopRequested = TRUE;

    for (waited_ms = 0; bRtTaskRunning && waited_ms < RT_STOP_WAIT_TIMEOUT_MS; waited_ms += 10)
        Ros_Sleep(10);

    return !bRtTaskRunning;
}

//Called after the R/T task has been forcibly deleted, or could not be created
void Ros_RtMotionControl_AbortSession()
{
    bRtSessionRunning = FALSE;
    bRtTaskRunning = FALSE;
}


static void Ros_RtMotionControl_InitJointSpace(MP_EXPOS_DATA* moveData)
{
    int i;

    bzero(moveData, sizeof(MP_EXPOS_DATA));

    for (i = 0; i < g_Ros_Controller.numGroup; i++)
    {
        moveData->ctrl_grp |= (0x01 << i);
        moveData->grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(g_Ros_Controller.ctrlGroups[i]);
        moveData->grp_pos_info[i].pos_tag.data[3] = MP_INC_PULSE_DTYPE;
    }

    //The starting position is read on the first command (see Ros_RtMotionControl_CheckForMotionDeviation)
    bExpectedPulsesRead = false;
}

static bool Ros_RtMotionControl_InitCartesian(MP_EXPOS_DATA* moveData)
{
    int i;

    bzero(moveData, sizeof(MP_EXPOS_DATA));

    moveData->m_ctrl_grp = 0;
    moveData->s_ctrl_grp = 0;

    for (i = 0; i < g_Ros_Controller.numGroup; i++)
    {
        CtrlGroup* group = g_Ros_Controller.ctrlGroups[i];

        //Cartesian increments only apply to robots, not to base tracks or positioners
        if (!Ros_CtrlGroup_IsRobot(group))
        {
            Ros_Debug_BroadcastMsg("ERROR: Group [%d] is not a robot (ex: base track or positioner). Cartesian control mode is not supported for this group.", i);
            return false;
        }

        moveData->ctrl_grp |= (1 << i);
        moveData->grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(g_Ros_Controller.ctrlGroups[i]);
        moveData->grp_pos_info[i].pos_tag.data[3] = MP_INC_RF_DTYPE;

        //The starting pose is read on the first command, with the tool that command selects.
        expectedTool[i] = -1;
    }

    return true;
}

//A joint may be commanded this much more than its maximum increment per cycle, so that a client
//which computes the maximum from rounded conversion factors isn't rejected. What is sent is
//still limited to the maximum; the excess is sent in a later cycle.
#define RT_MAX_INCREMENT_TOLERANCE_PULSES   0.001

static LONG Ros_RtMotionControl_Round(double value)
{
    return (LONG)((value < 0.0) ? (value - 0.5) : (value + 0.5));
}

//Converts an increment (already scaled to motoman units) to the nearest whole number of
//units, and carries the rest (up to half a unit, either way) over to the next cycle.
static LONG Ros_RtMotionControl_TakeWholeUnits(double increment, double* residual)
{
    double total = increment + *residual;
    LONG whole = Ros_RtMotionControl_Round(total);

    *residual = total - whole;
    return whole;
}

//The difference of two elbow angles (0.0001 degrees), the short way around. The elbow angle wraps
//around at +-180 degrees, like the Euler angles (ex: 179 degrees is 2 degrees past -179).
static double Ros_RtMotionControl_ElbowDifference(double to, double from)
{
    double difference = to - from;
    return difference - 3600000.0 * floor((difference + 1800000.0) / 3600000.0);
}

static bool Ros_RtMotionControl_AreToolsValid(RtPacket* incomingCommand)
{
    for (int groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        int toolIndex = incomingCommand->toolIndex[groupNo];
        if (toolIndex < 0 || toolIndex > MAX_VALID_TOOL_INDEX)
        {
            Ros_Debug_BroadcastMsg("ERROR: Group [%d] Tool [%d] is invalid (must be 0 - %d)", groupNo, toolIndex, MAX_VALID_TOOL_INDEX);
            return false;
        }
    }
    return true;
}

//Pulses per radian (rotary axis) or per meter (linear axis)
static double Ros_RtMotionControl_PulsesPerUnit(CtrlGroup* ctrlGroup, int axis)
{
    if (ctrlGroup->axisType.type[axis] == AXIS_ROTATION)
        return ctrlGroup->pulseToRad.PtoR[axis];
    else if (ctrlGroup->axisType.type[axis] == AXIS_LINEAR)
        return ctrlGroup->pulseToMeter.PtoM[axis];
    else
        return 1.0;
}

static bool Ros_RtMotionControl_ParseJointSpace(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    int axis, groupNo;
    double pulses;
    double motoOrderDelta[MP_GRP_AXES_NUM];
    double const* delta;

    // For each control group, convert radians (or meters) to pulses and prepare moveData
    for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[groupNo];

        //joints must be in moto-order
        if (sessionJointOrder == motoros2_interfaces__msg__JointOrderEnum__JOINT_STATES)
        {
            Ros_CtrlGroup_ConvertSequentialJointOrderToMotoJointOrder(ctrlGroup, incomingCommand->command[groupNo], motoOrderDelta);
            delta = motoOrderDelta;
        }
        else
            delta = incomingCommand->command[groupNo];

        //Check every axis, not just the first numAxes. Some robots have gaps in their
        //axis layout. (ex: a 4-axis palletizer (SLU--T-) has 4 axes, but T is at index 5)
        for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
        {
            if (Ros_CtrlGroup_IsInvalidAxis(ctrlGroup, axis))
            {
                moveData->grp_pos_info[groupNo].pos[axis] = 0;
                continue;
            }

            pulses = delta[axis] * Ros_RtMotionControl_PulsesPerUnit(ctrlGroup, axis);

            //Written so that NaN and infinity also fail this check
            if (!(fabs(pulses) <= ctrlGroup->maxInc.maxIncrement[axis] + RT_MAX_INCREMENT_TOLERANCE_PULSES))
            {
                Ros_Debug_BroadcastMsg("ERROR: Group [%d] Axis [%d] has been commanded to move [%f] pulse counts this increment, exceeding the maximum limit of [%d] pulse counts",
                    groupNo, axis, pulses, ctrlGroup->maxInc.maxIncrement[axis]);
                return false;
            }

            LONG whole = Ros_RtMotionControl_TakeWholeUnits(pulses, &residualIncrement[groupNo][axis]);

            //The carried fraction can round an increment at the limit one pulse past it.
            //Send the limit instead, and carry that pulse over too.
            //(maxIncrement is unsigned. Compare it as a signed number, or -maxIncrement wraps around.)
            LONG maxIncrement = (LONG)ctrlGroup->maxInc.maxIncrement[axis];
            if (whole > maxIncrement || whole < -maxIncrement)
            {
                LONG limit = (whole > 0) ? maxIncrement : -maxIncrement;
                residualIncrement[groupNo][axis] += whole - limit;
                whole = limit;
            }

            moveData->grp_pos_info[groupNo].pos[axis] = whole;
        }

        moveData->grp_pos_info[groupNo].pos_tag.data[2] = incomingCommand->toolIndex[groupNo];
    }

    return true;
}

//A quaternion whose length differs from 1 by more than this is rejected, rather than
//being silently normalized. It most likely indicates a bug in the client.
#define RT_QUATERNION_NORM_TOLERANCE    0.01

//1500 mm/sec is 1.5 mm/ms
#define RT_MAX_TCP_SPEED_MM_PER_MS      1.5

//360 deg/sec is 0.36 deg/ms (the controller's maximum TCP rotation speed)
#define RT_MAX_TCP_ROTATION_DEG_PER_MS  0.36

//No maximum speed is documented for the elbow angle, so the TCP rotation limit is used. Like the
//TCP limits, this only catches client errors (ex: wrong units). The controller limits the joints.
#define RT_MAX_ELBOW_SPEED_DEG_PER_MS   0.36

//Normalizes q, unless its length (*pNorm) differs from 1 by more than RT_QUATERNION_NORM_TOLERANCE
//or isn't a number. Then it returns false.
static bool Ros_RtMotionControl_NormalizeUnitQuaternion(Quaternion* q, double* pNorm)
{
    double norm = sqrt((q->x * q->x) + (q->y * q->y) + (q->z * q->z) + (q->w * q->w));
    *pNorm = norm;

    if (!(fabs(norm - 1.0) <= RT_QUATERNION_NORM_TOLERANCE)) //NaN also fails this check
        return false;

    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
    q->w /= norm;
    return true;
}

static bool Ros_RtMotionControl_ParseCartesian(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    int groupNo;
    double* delta;
    double magnitude;
    double norm;
    double rotation;
    double rx, ry, rz;
    Quaternion q;

    // For each control group, convert incoming command and prepare moveData
    for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        delta = incomingCommand->command[groupNo];

        moveData->grp_pos_info[groupNo].pos_tag.data[2] = incomingCommand->toolIndex[groupNo];

        //=========================================================================================
        //Translation
        magnitude = METERS_TO_MILLIMETERS(sqrt(pow(delta[TCP_X], 2) + //x^2
                                               pow(delta[TCP_Y], 2) + //y^2
                                               pow(delta[TCP_Z], 2))); //z^2

        //Written so that NaN and infinity also fail this check
        if (!(magnitude <= (RT_MAX_TCP_SPEED_MM_PER_MS * g_Ros_Controller.interpolPeriod)))
        {
            Ros_Debug_BroadcastMsg("ERROR: Group [%d] The increment for the TCP [%f mm] exceeds the maximum limit of 1500 mm/sec", groupNo, magnitude);
            return false;
        }

        moveData->grp_pos_info[groupNo].pos[TCP_X] = Ros_RtMotionControl_TakeWholeUnits(METERS_TO_MICROMETERS(delta[TCP_X]), &residualIncrement[groupNo][TCP_X]);
        moveData->grp_pos_info[groupNo].pos[TCP_Y] = Ros_RtMotionControl_TakeWholeUnits(METERS_TO_MICROMETERS(delta[TCP_Y]), &residualIncrement[groupNo][TCP_Y]);
        moveData->grp_pos_info[groupNo].pos[TCP_Z] = Ros_RtMotionControl_TakeWholeUnits(METERS_TO_MICROMETERS(delta[TCP_Z]), &residualIncrement[groupNo][TCP_Z]);

        //=========================================================================================
        //Rotation
        q.x = delta[TCP_Qx];
        q.y = delta[TCP_Qy];
        q.z = delta[TCP_Qz];
        q.w = delta[TCP_Qw];

        if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0)
        {
            //An all-zero packet is treated as "no rotation", so a zero-filled
            //packet can be used as a keep-alive
            q.w = 1.0;
        }
        else if (!Ros_RtMotionControl_NormalizeUnitQuaternion(&q, &norm))
        {
            Ros_Debug_BroadcastMsg("ERROR: Group [%d] The rotation must be a unit quaternion (length is [%f])", groupNo, norm);
            return false;
        }

        rotation = Ros_RtMotionControl_RotationAngle(NULL, &q) * DEGREES_PER_RAD;
        if (rotation > (RT_MAX_TCP_ROTATION_DEG_PER_MS * g_Ros_Controller.interpolPeriod))
        {
            Ros_Debug_BroadcastMsg("ERROR: Group [%d] The rotation increment for the TCP [%f deg] exceeds the maximum limit of 360 deg/sec", groupNo, rotation);
            return false;
        }

        QuatConversion_GeomMsgsQuaternion_To_EulerRad(&q, &rx, &ry, &rz);

        moveData->grp_pos_info[groupNo].pos[3] = Ros_RtMotionControl_TakeWholeUnits(RAD_TO_DEG_0001(rx), &residualIncrement[groupNo][3]);
        moveData->grp_pos_info[groupNo].pos[4] = Ros_RtMotionControl_TakeWholeUnits(RAD_TO_DEG_0001(ry), &residualIncrement[groupNo][4]);
        moveData->grp_pos_info[groupNo].pos[5] = Ros_RtMotionControl_TakeWholeUnits(RAD_TO_DEG_0001(rz), &residualIncrement[groupNo][5]);

        //=========================================================================================
        //Elbow angle (7-axis robots). Other robots have no elbow angle.
        moveData->grp_pos_info[groupNo].pos[6] = 0;
        if (g_Ros_Controller.ctrlGroups[groupNo]->numAxes >= 7)
        {
            //Written so that NaN and infinity also fail this check
            if (!(fabs(delta[TCP_Re]) * DEGREES_PER_RAD <= (RT_MAX_ELBOW_SPEED_DEG_PER_MS * g_Ros_Controller.interpolPeriod)))
            {
                Ros_Debug_BroadcastMsg("ERROR: Group [%d] The elbow angle increment [%f deg] exceeds the maximum limit of 360 deg/sec", groupNo, delta[TCP_Re] * DEGREES_PER_RAD);
                return false;
            }

            moveData->grp_pos_info[groupNo].pos[6] = Ros_RtMotionControl_TakeWholeUnits(RAD_TO_DEG_0001(delta[TCP_Re]), &residualIncrement[groupNo][6]);
        }

        moveData->grp_pos_info[groupNo].pos[7] = 0;
    }

    return true;
}

//=================================================================================================
//Absolute control modes
//
//The motion API only accepts increments. So each cycle, the increment is the target minus the
//position the robot has been commanded to. Whatever the robot didn't execute (ex: because the
//FSU slowed it down), and any fraction of a unit which couldn't be sent, is made up by the next
//increment, as long as the target is within one cycle's motion. Each target is absolute, so
//nothing is lost if a packet is lost either.
//
//Joint: the commanded pulses are read from the controller every cycle (by the deviation check). They
//are exact. (On hardware, the position read already includes the previous cycle's increment.)
//
//Cartesian: the expected pose of the deviation check is used instead: the pose read at the start,
//plus every increment which was sent. The pose read from the controller is computed from the
//commanded pulses, so it doesn't include the part of an increment which the controller has not
//turned into a whole pulse yet. Sending the difference to that pose would send that part again
//every cycle, until a joint steps a pulse too far, and then back (the robot buzzes at rest). The
//pose is only read again when it differs from the expected pose by more than the deviation
//thresholds, or the tool changes.

//CARTESIAN_ABSOLUTE: the tool of the previous command. A command which changes the tool isn't
//executed for that group (see Ros_RtMotionControl_ParseCartesianAbsolute).
static int absPrevTool[MAX_GROUPS];

//Rejects NaN and infinity
static bool Ros_RtMotionControl_IsFinite(double value)
{
    return (value - value) == 0.0;
}

//out = a * b
static void Ros_RtMotionControl_QuatMultiply(const Quaternion* a, const Quaternion* b, Quaternion* out)
{
    Quaternion result;

    result.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    result.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    result.y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    result.z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;

    *out = result;
}

//The first command of an absolute session is not executed. The client doesn't know the
//robot's position yet. The reply to this command tells it.
static void Ros_RtMotionControl_StartAbsolute(MOTION_MODE mode, RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    for (int groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        //A later command which changes the tool isn't executed either
        if (mode == MOTION_MODE_RT_CARTESIAN)
            absPrevTool[groupNo] = incomingCommand->toolIndex[groupNo];

        bzero(moveData->grp_pos_info[groupNo].pos, sizeof(moveData->grp_pos_info[groupNo].pos));
        moveData->grp_pos_info[groupNo].pos_tag.data[2] = incomingCommand->toolIndex[groupNo];
    }
}

static bool Ros_RtMotionControl_ParseJointAbsolute(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    int axis, groupNo;
    double increment;
    double motoOrderTarget[MP_GRP_AXES_NUM];
    double const* target;

    for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[groupNo];

        //The pulses the robot has been commanded to, as the deviation check read them this cycle
        LONG const* commanded = prevRtCmdPosition[groupNo];

        //joints must be in moto-order
        if (sessionJointOrder == motoros2_interfaces__msg__JointOrderEnum__JOINT_STATES)
        {
            Ros_CtrlGroup_ConvertSequentialJointOrderToMotoJointOrder(ctrlGroup, incomingCommand->command[groupNo], motoOrderTarget);
            target = motoOrderTarget;
        }
        else
            target = incomingCommand->command[groupNo];

        for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
        {
            moveData->grp_pos_info[groupNo].pos[axis] = 0;
            if (Ros_CtrlGroup_IsInvalidAxis(ctrlGroup, axis))
                continue;

            if (!Ros_RtMotionControl_IsFinite(target[axis]))
            {
                Ros_Debug_BroadcastMsg("ERROR: Group [%d] Axis [%d] The target is not a finite number", groupNo, axis);
                return false;
            }

            increment = target[axis] * Ros_RtMotionControl_PulsesPerUnit(ctrlGroup, axis) - commanded[axis];

            //The target must be within one cycle's motion of the position the robot has been commanded to
            if (!(fabs(increment) <= ctrlGroup->maxInc.maxIncrement[axis] + RT_MAX_INCREMENT_TOLERANCE_PULSES))
            {
                Ros_Debug_BroadcastMsg("ERROR: Group [%d] Axis [%d] The target is [%f] pulse counts away, exceeding the maximum limit of [%d] pulse counts per cycle",
                    groupNo, axis, increment, ctrlGroup->maxInc.maxIncrement[axis]);
                return false;
            }

            moveData->grp_pos_info[groupNo].pos[axis] = Ros_RtMotionControl_Round(increment);
        }

        moveData->grp_pos_info[groupNo].pos_tag.data[2] = incomingCommand->toolIndex[groupNo];
    }

    return true;
}

static bool Ros_RtMotionControl_ParseCartesianAbsolute(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    int groupNo, i;
    double* target;
    double translation[3];  //micrometers
    double magnitude;       //mm
    double norm;
    double rotation;        //degrees
    double elbow;           //0.0001 degrees
    double elbowRotation;   //degrees
    double rx, ry, rz;
    Quaternion qTarget, qCurrentInverse, qIncrement;

    const double maxTranslation = RT_MAX_TCP_SPEED_MM_PER_MS * g_Ros_Controller.interpolPeriod;
    const double maxRotation = RT_MAX_TCP_ROTATION_DEG_PER_MS * g_Ros_Controller.interpolPeriod;
    const double maxElbowRotation = RT_MAX_ELBOW_SPEED_DEG_PER_MS * g_Ros_Controller.interpolPeriod;

    for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        int tool = incomingCommand->toolIndex[groupNo];

        //A command which changes the tool isn't executed for this group, like the first command of
        //a session: the client can't know where the new tool's TCP is. The deviation check has just
        //read that pose with the new tool, and the reply reports it. The next target must be within
        //one cycle's motion of it.
        if (tool != absPrevTool[groupNo])
        {
            absPrevTool[groupNo] = tool;
            bzero(moveData->grp_pos_info[groupNo].pos, sizeof(moveData->grp_pos_info[groupNo].pos));
            moveData->grp_pos_info[groupNo].pos_tag.data[2] = tool;
            continue;
        }

        target = incomingCommand->command[groupNo];

        //Like unused groups, the elbow angle (the last slot) is ignored for robots which don't have one
        const int numAxesUsed = (g_Ros_Controller.ctrlGroups[groupNo]->numAxes >= 7) ? (TCP_Re + 1) : TCP_Re;
        for (i = 0; i < numAxesUsed; i += 1)
        {
            if (!Ros_RtMotionControl_IsFinite(target[i]))
            {
                Ros_Debug_BroadcastMsg("ERROR: Group [%d] The target [%d] is not a finite number", groupNo, i);
                return false;
            }
        }

        //=========================================================================================
        //Translation
        for (i = 0; i < 3; i += 1)
            translation[i] = METERS_TO_MICROMETERS(target[TCP_X + i]) - expectedTranslation[groupNo][i];

        magnitude = sqrt(pow(translation[0], 2) + pow(translation[1], 2) + pow(translation[2], 2)) / 1000.0;

        //=========================================================================================
        //Rotation
        qTarget.x = target[TCP_Qx];
        qTarget.y = target[TCP_Qy];
        qTarget.z = target[TCP_Qz];
        qTarget.w = target[TCP_Qw];

        if (!Ros_RtMotionControl_NormalizeUnitQuaternion(&qTarget, &norm))
        {
            Ros_Debug_BroadcastMsg("ERROR: Group [%d] The orientation must be a unit quaternion (length is [%f])", groupNo, norm);
            return false;
        }

        //Increments are in the robot frame, so: target = increment * current
        qCurrentInverse.x = -expectedOrientation[groupNo].x;
        qCurrentInverse.y = -expectedOrientation[groupNo].y;
        qCurrentInverse.z = -expectedOrientation[groupNo].z;
        qCurrentInverse.w = expectedOrientation[groupNo].w;
        Ros_RtMotionControl_QuatMultiply(&qTarget, &qCurrentInverse, &qIncrement);

        //take the short way around
        if (qIncrement.w < 0.0)
        {
            qIncrement.x = -qIncrement.x;
            qIncrement.y = -qIncrement.y;
            qIncrement.z = -qIncrement.z;
            qIncrement.w = -qIncrement.w;
        }

        rotation = Ros_RtMotionControl_RotationAngle(NULL, &qIncrement) * DEGREES_PER_RAD;

        //=========================================================================================
        //Elbow angle (7-axis robots). Other robots have no elbow angle, so nothing is sent for
        //it, whatever the controller reports in its place.
        elbow = 0.0;
        if (g_Ros_Controller.ctrlGroups[groupNo]->numAxes >= 7)
            elbow = Ros_RtMotionControl_ElbowDifference(RAD_TO_DEG_0001(target[TCP_Re]), expectedElbow[groupNo]);
        elbowRotation = fabs(elbow) / 10000.0;

        //=========================================================================================
        //The target must be within one cycle's motion of the pose the robot has been commanded to
        if (!(magnitude <= maxTranslation && rotation <= maxRotation && elbowRotation <= maxElbowRotation))
        {
            Ros_Debug_BroadcastMsg("ERROR: Group [%d] The target is [%f mm], [%f deg] and [%f deg] of elbow angle away, exceeding the maximum limit of 1500 mm/sec and 360 deg/sec",
                groupNo, magnitude, rotation, elbowRotation);
            return false;
        }

        //=========================================================================================
        //Send whole units. What was sent is added to the expected pose after the move, and
        //whatever is left over is made up next cycle.
        for (i = 0; i < 3; i += 1)
            moveData->grp_pos_info[groupNo].pos[TCP_X + i] = Ros_RtMotionControl_Round(translation[i]);

        QuatConversion_GeomMsgsQuaternion_To_EulerRad(&qIncrement, &rx, &ry, &rz);
        moveData->grp_pos_info[groupNo].pos[3] = Ros_RtMotionControl_Round(RAD_TO_DEG_0001(rx));
        moveData->grp_pos_info[groupNo].pos[4] = Ros_RtMotionControl_Round(RAD_TO_DEG_0001(ry));
        moveData->grp_pos_info[groupNo].pos[5] = Ros_RtMotionControl_Round(RAD_TO_DEG_0001(rz));

        moveData->grp_pos_info[groupNo].pos[6] = Ros_RtMotionControl_Round(elbow);

        moveData->grp_pos_info[groupNo].pos[7] = 0;
        moveData->grp_pos_info[groupNo].pos_tag.data[2] = tool;
    }

    return true;
}

void Ros_RtMotionControl_OpenSocket()
{
    struct sockaddr_in server_addr;

    //Not fatal, like a failed bind below: only the R/T interface needs this socket
    sockRtCommandListener = mpSocket(AF_INET, SOCK_DGRAM, 0);
    if (sockRtCommandListener < 0)
    {
        Ros_Debug_BroadcastMsg("ERROR: Could not allocate socket for RT interface. start_rt_mode will be rejected.");
        sockRtCommandListener = -1;
        mpSetAlarm(ALARM_ASSERTION_FAIL, "Failed to allocate RT socket", SUBCODE_FAIL_ALLOCATE_RT_CMD_SOCKET);
        return;
    }

    // Bind socket to port
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = mpHtons(g_nodeConfigSettings.rt_listener_udp_port_number);

    //Not fatal: only the R/T interface needs this port. start_rt_mode is rejected instead.
    if (mpBind(sockRtCommandListener, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
        Ros_Debug_BroadcastMsg("ERROR: Failed to bind UDP port %d for real-time motion control. start_rt_mode will be rejected.",
            g_nodeConfigSettings.rt_listener_udp_port_number);
        mpClose(sockRtCommandListener);
        sockRtCommandListener = -1;
        mpSetAlarm(ALARM_CONFIGURATION_FAIL, "rt_listener_udp_port in use", SUBCODE_CONFIGURATION_RT_PORT_IN_USE);
    }
}

BOOL Ros_RtMotionControl_IsSocketOpen()
{
    return sockRtCommandListener >= 0;
}

//Converts joints from moto-order to the order requested for this session
static void Ros_RtMotionControl_ToSessionJointOrder(CtrlGroup* group, double joints[MP_GRP_AXES_NUM])
{
    double motoOrder[MP_GRP_AXES_NUM];

    if (sessionJointOrder != motoros2_interfaces__msg__JointOrderEnum__JOINT_STATES)
        return;

    memcpy(motoOrder, joints, sizeof(motoOrder));
    bzero(joints, sizeof(double) * MP_GRP_AXES_NUM); //the conversion doesn't fill the trailing axes
    Ros_CtrlGroup_ConvertMotoJointOrderToSequentialJointOrder(group, motoOrder, joints);
}

static void Ros_RtMotionControl_PopulateReplyMessage(MOTION_MODE mode, RtPacket* command, RtReply* reply)
{
    long pulsePos_moto[MAX_PULSE_AXES];
    long anglePos_moto[MAX_PULSE_AXES];
    BITSTRING figure;
    MP_COORD coord;
    MP_CTRL_GRP_SEND_DATA ctrlGroup;
    MP_PULSE_POS_RSP_DATA cmdPulse;
    MP_CARTPOS_EX_SEND_DATA cartSendData;
    MP_CART_POS_RSP_DATA_EX cartRespData;

    bzero(reply, sizeof(RtReply));

    reply->sequenceEcho = command->sequenceId;

    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
    {
        CtrlGroup* group = g_Ros_Controller.ctrlGroups[groupIndex];

        //================================================================================
        //FB pos
        //================================================================================
        //If it can't be read, the joints are 0, and the Cartesian pose is left at 0
        BOOL bFeedbackRead = Ros_CtrlGroup_GetFBPulsePos(group, pulsePos_moto);

        //Angles (or meters for a linear track)
        Ros_CtrlGroup_ConvertMotoUnitsToRosUnits(group, pulsePos_moto, reply->feedbackPositionJoints[groupIndex]);

        if (bFeedbackRead && Ros_CtrlGroup_IsRobot(group)) //is a robot and not an external axis
        {
            //Cart. If it can't be computed, it is left at 0. mpConvPulseToAngle also converts
            //linear axes (ex: SCARA U-axis), which the joint positions above have in meters.
            if (mpConvPulseToAngle(groupIndex, pulsePos_moto, anglePos_moto) == OK &&
                mpConvAxesToCartPos(groupIndex, anglePos_moto, command->toolIndex[groupIndex], &figure, &coord) == OK)
            {
                Quaternion qFb;
                QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(coord.rx, coord.ry, coord.rz, &qFb);

                reply->feedbackPositionCartesian[groupIndex][TCP_X] = MICROMETERS_TO_METERS(coord.x);
                reply->feedbackPositionCartesian[groupIndex][TCP_Y] = MICROMETERS_TO_METERS(coord.y);
                reply->feedbackPositionCartesian[groupIndex][TCP_Z] = MICROMETERS_TO_METERS(coord.z);

                reply->feedbackPositionCartesian[groupIndex][TCP_Qx] = qFb.x;
                reply->feedbackPositionCartesian[groupIndex][TCP_Qy] = qFb.y;
                reply->feedbackPositionCartesian[groupIndex][TCP_Qz] = qFb.z;
                reply->feedbackPositionCartesian[groupIndex][TCP_Qw] = qFb.w;
                //Other robots have no elbow angle
                if (group->numAxes >= 7)
                    reply->feedbackPositionCartesian[groupIndex][TCP_Re] = DEG_0001_TO_RAD(coord.ex1);
            }
        }

        //after the FK, which needs moto-order
        Ros_RtMotionControl_ToSessionJointOrder(group, reply->feedbackPositionJoints[groupIndex]);

        //================================================================================
        //CMD pos
        //================================================================================
        //Not Ros_CtrlGroup_GetPulsePosCmd. That is only for converting incoming absolute
        //positions. See https://github.com/Yaskawa-Global/motoros2/discussions/455
        //rad (or meter for linear track). If it can't be read, it is left at 0.
        ctrlGroup.sCtrlGrp = g_Ros_Controller.ctrlGroups[groupIndex]->groupId;
        if (mpGetPulsePos(&ctrlGroup, &cmdPulse) == OK)
            Ros_CtrlGroup_ConvertMotoUnitsToRosUnits(group, cmdPulse.lPos, reply->previousCommandPositionJoints[groupIndex]);

        if (Ros_CtrlGroup_IsRobot(group)) //is a robot and not an external axis
        {
            //Cart. If it can't be read, it is left at 0. It is computed from whole pulses, so it
            //can differ by about one pulse from the pose the absolute Cartesian mode keeps track
            //of: a client which sends it back as its target can move the robot that far.
            cartSendData.sRobotNo = groupIndex;
            cartSendData.sFrame = 1; //1 = RF
            cartSendData.sToolNo = command->toolIndex[groupIndex];
            if (mpGetCartPosEx(&cartSendData, &cartRespData) == OK)
            {
                Quaternion qCmd;
                QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(cartRespData.lPos[3], cartRespData.lPos[4], cartRespData.lPos[5], &qCmd);

                reply->previousCommandPositionCartesian[groupIndex][TCP_X] = MICROMETERS_TO_METERS(cartRespData.lPos[TCP_X]);
                reply->previousCommandPositionCartesian[groupIndex][TCP_Y] = MICROMETERS_TO_METERS(cartRespData.lPos[TCP_Y]);
                reply->previousCommandPositionCartesian[groupIndex][TCP_Z] = MICROMETERS_TO_METERS(cartRespData.lPos[TCP_Z]);

                reply->previousCommandPositionCartesian[groupIndex][TCP_Qx] = qCmd.x;
                reply->previousCommandPositionCartesian[groupIndex][TCP_Qy] = qCmd.y;
                reply->previousCommandPositionCartesian[groupIndex][TCP_Qz] = qCmd.z;
                reply->previousCommandPositionCartesian[groupIndex][TCP_Qw] = qCmd.w;
                //Other robots have no elbow angle
                if (group->numAxes >= 7)
                    reply->previousCommandPositionCartesian[groupIndex][TCP_Re] = DEG_0001_TO_RAD(cartRespData.lPos[6]);
            }
        }

        Ros_RtMotionControl_ToSessionJointOrder(group, reply->previousCommandPositionJoints[groupIndex]);
    }
}

//Cartesian mode: starts the expected pose from a pose read from the controller
static void Ros_RtMotionControl_SetExpectedPose(int groupNo, const LONG* pose, int tool)
{
    for (int i = 0; i < 3; i += 1)
        expectedTranslation[groupNo][i] = pose[TCP_X + i];
    QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(pose[3], pose[4], pose[5], &expectedOrientation[groupNo]);
    expectedElbow[groupNo] = pose[6];
    expectedTool[groupNo] = tool;
}

//Cartesian mode: whether a pose read from the controller differs from the expected pose by more
//than the thresholds
static bool Ros_RtMotionControl_PoseDeviates(int groupNo, const LONG* pose)
{
    Quaternion qRead;
    bool bDeviates = false;

    //Translation (micrometers)
    for (int i = 0; i < 3; i += 1)
    {
        if (fabs(pose[TCP_X + i] - expectedTranslation[groupNo][i]) > g_nodeConfigSettings.rt_motion_deviation_threshold_micrometers)
            bDeviates = true;
    }

    //Rotation (0.0001 degrees)
    //The orientation is in Euler angles, so a rotation about one axis can change all three of
    //them. The individual angles can't be compared. Instead, compare the angle between the
    //orientations, which also catches a rotation about the wrong axis.
    QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(pose[3], pose[4], pose[5], &qRead);
    if (fabs(RAD_TO_DEG_0001(Ros_RtMotionControl_RotationAngle(&expectedOrientation[groupNo], &qRead))) > g_nodeConfigSettings.rt_motion_deviation_threshold_deg_0001)
        bDeviates = true;

    //Elbow angle (0.0001 degrees)
    if (g_Ros_Controller.ctrlGroups[groupNo]->numAxes >= 7 &&
        fabs(Ros_RtMotionControl_ElbowDifference(pose[6], expectedElbow[groupNo])) > g_nodeConfigSettings.rt_motion_deviation_threshold_deg_0001)
        bDeviates = true;

    return bDeviates;
}

//Adds the increments which were just sent to the expected position
static void Ros_RtMotionControl_AddToExpectedPosition(MOTION_MODE mode, MP_EXPOS_DATA* moveData)
{
    for (int groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        LONG* sent = moveData->grp_pos_info[groupNo].pos;

        if (mode == MOTION_MODE_RT_JOINT)
        {
            for (int axis = 0; axis < MP_GRP_AXES_NUM; axis += 1)
                expectedPulses[groupNo][axis] += sent[axis];
        }
        else
        {
            Quaternion qSent;
            Quaternion* q = &expectedOrientation[groupNo];

            for (int i = 0; i < 3; i += 1)
                expectedTranslation[groupNo][i] += sent[TCP_X + i];

            //Robot frame increments compose as R_new = R_increment * R_old
            QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(sent[3], sent[4], sent[5], &qSent);
            Ros_RtMotionControl_QuatMultiply(&qSent, q, q);

            //Its length only differs from 1 by rounding errors, so this always normalizes it
            double norm;
            (void)Ros_RtMotionControl_NormalizeUnitQuaternion(q, &norm);

            expectedElbow[groupNo] += sent[6];
        }
    }
}

//Compares the position the robot has been commanded to with the expected position (see
//expectedPulses), and sets *pbDeviation if any group differs by more than the thresholds (ex: the
//FSU speed limit truncated an increment, or the controller changed it near a singularity).
//Returns false if the position can't be read.
static bool Ros_RtMotionControl_CheckForMotionDeviation(MOTION_MODE mode, int* tools, bool* pbDeviation)
{
    MP_CTRL_GRP_SEND_DATA ctrlGroup;
    MP_PULSE_POS_RSP_DATA cmdPulse;
    MP_CARTPOS_EX_SEND_DATA cartSendData;
    MP_CART_POS_RSP_DATA_EX cartRespData;

    *pbDeviation = false;

    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
    {
        bool bGroupDeviates = false;

        if (mode == MOTION_MODE_RT_JOINT)
        {
            //Not Ros_CtrlGroup_GetPulsePosCmd. That is only for converting incoming absolute
            //positions. See https://github.com/Yaskawa-Global/motoros2/discussions/455
            ctrlGroup.sCtrlGrp = g_Ros_Controller.ctrlGroups[groupIndex]->groupId;
            if (mpGetPulsePos(&ctrlGroup, &cmdPulse) != OK)
            {
                Ros_Debug_BroadcastMsg("ERROR: Group [%d] mpGetPulsePos failed", groupIndex);
                return false;
            }
            memcpy(prevRtCmdPosition[groupIndex], cmdPulse.lPos, sizeof(cmdPulse.lPos));

            //pulses. Nothing to check on the first command: it starts from the position read.
            for (int axis = 0; axis < MP_GRP_AXES_NUM && bExpectedPulsesRead; axis += 1)
            {
                if (abs(cmdPulse.lPos[axis] - expectedPulses[groupIndex][axis]) > g_nodeConfigSettings.rt_motion_deviation_threshold_pulses)
                    bGroupDeviates = true;
            }

            if (bGroupDeviates || !bExpectedPulsesRead)
                memcpy(expectedPulses[groupIndex], cmdPulse.lPos, sizeof(cmdPulse.lPos));
        }
        else if (mode == MOTION_MODE_RT_CARTESIAN)
        {
            cartSendData.sRobotNo = groupIndex;
            cartSendData.sFrame = 1; //1 = RF

            //Nothing to check before the first command
            if (expectedTool[groupIndex] >= 0)
            {
                //Read the pose with the tool the increments were commanded with
                cartSendData.sToolNo = expectedTool[groupIndex];
                if (mpGetCartPosEx(&cartSendData, &cartRespData) != OK)
                {
                    Ros_Debug_BroadcastMsg("ERROR: Group [%d] mpGetCartPosEx failed (tool [%d])", groupIndex, expectedTool[groupIndex]);
                    return false;
                }

                bGroupDeviates = Ros_RtMotionControl_PoseDeviates(groupIndex, cartRespData.lPos);
                if (bGroupDeviates)
                    Ros_RtMotionControl_SetExpectedPose(groupIndex, cartRespData.lPos, expectedTool[groupIndex]);
            }

            if (tools[groupIndex] != expectedTool[groupIndex])
            {
                //First command, or this command changes the tool. Start again from the pose
                //read with this command's tool, so its increment is checked against the same TCP.
                cartSendData.sToolNo = tools[groupIndex];
                if (mpGetCartPosEx(&cartSendData, &cartRespData) != OK)
                {
                    Ros_Debug_BroadcastMsg("ERROR: Group [%d] mpGetCartPosEx failed (tool [%d])", groupIndex, tools[groupIndex]);
                    return false;
                }
                Ros_RtMotionControl_SetExpectedPose(groupIndex, cartRespData.lPos, tools[groupIndex]);
            }
        }

        if (bGroupDeviates)
            *pbDeviation = true;
    }

    if (mode == MOTION_MODE_RT_JOINT)
        bExpectedPulsesRead = true;
    return true;
}

//Returns the angle (radians) of the rotation from 'from' to 'to'.
//If 'from' is NULL, returns the angle of the rotation 'to'.
static double Ros_RtMotionControl_RotationAngle(const Quaternion* from, const Quaternion* to)
{
    double x = to->x, y = to->y, z = to->z, w = to->w;

    if (from != NULL)
    {
        //conjugate(from) * to
        w = from->w * to->w + from->x * to->x + from->y * to->y + from->z * to->z;
        x = from->w * to->x - from->x * to->w - from->y * to->z + from->z * to->y;
        y = from->w * to->y - from->y * to->w - from->z * to->x + from->x * to->z;
        z = from->w * to->z - from->z * to->w - from->x * to->y + from->y * to->x;
    }

    return 2.0 * atan2(sqrt((x * x) + (y * y) + (z * z)), fabs(w));
}

static void Ros_RtMotionControl_PurgeBufferedPackets()
{
    struct fd_set fds;
    struct timeval tv;
    struct sockaddr_in client_addr;
    int client_addr_len = sizeof(client_addr);
    RtPacket incomingCommand;

    //----------------------------
    //mpIoctl(sockRtCommandListener, FIOFLUSH, 1);
    //UPDATE: mpIoctl isn't working! We'll manually purge the buffer with a draining loop.
    //----------------------------
    while (TRUE)
    {
        FD_ZERO(&fds);
        FD_SET(sockRtCommandListener, &fds);

        //no wait
        tv.tv_usec = 0;
        tv.tv_sec = 0;

        if (mpSelect(sockRtCommandListener + 1, &fds, NULL, NULL, &tv) <= 0)
            break;

        //Stop on an error, rather than finding the same packet waiting forever
        client_addr_len = sizeof(client_addr);
        if (mpRecvFrom(sockRtCommandListener, (char*)&incomingCommand, sizeof(RtPacket), 0, (struct sockaddr*)&client_addr, &client_addr_len) < 0)
            break;
    }
}

//Discards the packets which arrived during the current cycle (before its reply was sent),
//and checks whether any of them means the session must end:
//- a packet from another IP address (MultipleSources)
//- a newer command from the commander, which was sent without waiting for the reply.
//  Its increment would be lost, so the client must be told. (CommandTooEarly)
//A resend of a command which was already executed is harmless, and is ignored.
//
//Returns EndReason_None, or the reason the session must end (and sets endDetail).
static EndReason Ros_RtMotionControl_DiscardEarlyPackets(const struct sockaddr_in* commander, UINT32 executedSequenceId, int* endDetail)
{
    struct fd_set fds;
    struct timeval tv;
    struct sockaddr_in client_addr;
    int client_addr_len;
    char receiveBuffer[sizeof(RtPacket) + 1];
    RtPacket packet;
    int bytes_received;

    while (TRUE)
    {
        FD_ZERO(&fds);
        FD_SET(sockRtCommandListener, &fds);

        //no wait
        tv.tv_usec = 0;
        tv.tv_sec = 0;

        if (mpSelect(sockRtCommandListener + 1, &fds, NULL, NULL, &tv) <= 0)
            return EndReason_None;

        client_addr_len = sizeof(client_addr);
        bytes_received = mpRecvFrom(sockRtCommandListener, receiveBuffer, sizeof(receiveBuffer), 0, (struct sockaddr*)&client_addr, &client_addr_len);
        if (bytes_received < 0)
            return EndReason_None;

        if (client_addr.sin_addr.s_addr != commander->sin_addr.s_addr)
            return Ros_RtMotionControl_MultipleSources(commander, &client_addr, endDetail);

        //Only a well-formed packet can be judged. Anything else is just discarded.
        if (bytes_received != sizeof(RtPacket))
            continue;
        memcpy(&packet, receiveBuffer, sizeof(RtPacket));
        if (packet.version != VERSION_REAL_TIME_INTERFACE)
            continue;

        //wraparound safe
        if ((int32_t)(packet.sequenceId - executedSequenceId) > 0)
        {
            Ros_Debug_BroadcastMsg("ERROR: Command [%u] was sent before the reply to command [%u]. Its increment would be lost.",
                                    packet.sequenceId, executedSequenceId);
            *endDetail = (int)packet.sequenceId;
            return EndReason_CommandTooEarly;
        }
    }
}

//A packet came from another IP address than the commander's. Logs it, and returns the EndReason.
static EndReason Ros_RtMotionControl_MultipleSources(const struct sockaddr_in* commander, const struct sockaddr_in* sender, int* endDetail)
{
    Ros_Debug_BroadcastMsg("ERROR: Received command packets from multiple sources (0x%08X and 0x%08X)",
                            (UINT32)commander->sin_addr.s_addr,
                            (UINT32)sender->sin_addr.s_addr);
    *endDetail = (int)sender->sin_addr.s_addr;
    return EndReason_MultipleSources;
}

//Counts the cycles which lost a tick or replied late, and keeps the first ones to log when the
//session ends. A lost tick can also be the client's doing (ex: it sent its command late). That
//shows as a long wait for the command.
static void Ros_RtMotionControl_CheckCycleTiming(UINT32 sequenceId, const RtCycleTimes* prev, const RtCycleTimes* now, RtTimingStats* stats)
{
    const float msPerTick = (float)mpGetRtc();
    const float period = g_Ros_Controller.interpolPeriod;
    const bool bLostTick = ((now->tick - prev->tick) * msPerTick) > (period * 1.5f);
    const bool bLateReply = ((now->replied - now->tick) * msPerTick) > (period / 2.0f);

    stats->cycles += 1;
    if (bLostTick)
        stats->lostTickCycles += 1;
    if (bLateReply)
        stats->lateReplies += 1;

    if (!bLostTick && !bLateReply)
        return;

    stats->problemCycles += 1;
    if (stats->numEvents < RT_TIMING_MAX_EVENTS)
    {
        RtTimingEvent* event = &stats->events[stats->numEvents];
        event->sequenceId = sequenceId;
        event->sincePrevTick = now->tick - prev->tick;
        event->waitForCommand = now->received - prev->replied;
        event->processing = now->moved - now->received;
        event->waitForTick = now->tick - now->moved;
        event->buildingReply = now->replied - now->tick;
        event->populating = now->populated - now->tick;
        event->discarding = now->discarded - now->populated;
        event->sending = now->replied - now->discarded;
        stats->numEvents += 1;
    }
}

static void Ros_RtMotionControl_LogCycleTiming(const RtTimingStats* stats)
{
    const float msPerTick = (float)mpGetRtc();

    Ros_Debug_BroadcastMsg("R/T timing: %u cycles, %u started late (lost a tick), %u late replies",
        stats->cycles, stats->lostTickCycles, stats->lateReplies);

    for (int i = 0; i < stats->numEvents; i += 1)
    {
        const RtTimingEvent* event = &stats->events[i];
        Ros_Debug_BroadcastMsg("R/T timing: seq %u: %.1f ms between ticks. Previous reply to command: %.1f ms, processing: %.1f ms, wait for tick: %.1f ms, building reply: %.1f ms (populating %.1f, discarding %.1f, sending %.1f)",
            event->sequenceId, event->sincePrevTick * msPerTick, event->waitForCommand * msPerTick,
            event->processing * msPerTick, event->waitForTick * msPerTick, event->buildingReply * msPerTick,
            event->populating * msPerTick, event->discarding * msPerTick, event->sending * msPerTick);
    }

    if (stats->problemCycles > (UINT32)stats->numEvents)
        Ros_Debug_BroadcastMsg("R/T timing: only the first %d are listed", RT_TIMING_MAX_EVENTS);
}

//Reads the robot's state into a bitfield of RobotStatusBits
static UINT32 Ros_RtMotionControl_ReadRobotStatus()
{
    USHORT ioStatus[IO_ROBOTSTATUS_MAX];
    UINT32 status = 0;

    //Read into a local array. g_Ros_Controller.ioStatus is only updated while the agent is connected.
    if (!Ros_Controller_StatusRead(ioStatus))
        return 0;

    if (ioStatus[IO_ROBOTSTATUS_SERVO] != 0 && ioStatus[IO_ROBOTSTATUS_INECOMODE] == 0)
        status |= RobotStatus_DrivesPowered;

    if (ioStatus[IO_ROBOTSTATUS_ESTOP_EX] == 0 || ioStatus[IO_ROBOTSTATUS_ESTOP_PP] == 0 || ioStatus[IO_ROBOTSTATUS_ESTOP_CTRL] == 0)
        status |= RobotStatus_EStopped;

    if (ioStatus[IO_ROBOTSTATUS_HOLD] != 0)
        status |= RobotStatus_Hold;

    if (ioStatus[IO_ROBOTSTATUS_ALARM_MAJOR] != 0 || ioStatus[IO_ROBOTSTATUS_ALARM_MINOR] != 0 ||
        ioStatus[IO_ROBOTSTATUS_ALARM_SYSTEM] != 0 || ioStatus[IO_ROBOTSTATUS_ALARM_USER] != 0)
        status |= RobotStatus_Alarm;

    if (ioStatus[IO_ROBOTSTATUS_ERROR] != 0)
        status |= RobotStatus_Error;

    if (ioStatus[IO_ROBOTSTATUS_PLAY] != 0)
        status |= RobotStatus_PlayMode;

    if (ioStatus[IO_ROBOTSTATUS_REMOTE] != 0)
        status |= RobotStatus_RemoteMode;

    if (Ros_Controller_IsPflActiveInStatus(ioStatus))
        status |= RobotStatus_PflActive;

    return status;
}

//The motion API rejects an increment before the controller's status signals show why.
//(ex: HOLD takes a few cycles to appear.) Motion was accepted on the previous
//cycle, so none of the causes were present then. Read the status each cycle until one
//appears, so the final reply can report it.
//
//Returns the status. If no cause appears in time, returns the status as it is.
static UINT32 Ros_RtMotionControl_WaitForStopCause()
{
    //The same causes as Ros_RtMotionControl_CheckStatus, and an error, which can also explain
    //why an increment was rejected
    const UINT32 stopCauses = RT_STOPPED_STATUS | RobotStatus_Error;
    const UINT32 requiredForMotion = RobotStatus_DrivesPowered | RT_REQUIRED_MODES;
    UINT32 status = Ros_RtMotionControl_ReadRobotStatus();
    int waited_ms = 0;

    while (!(status & stopCauses) && ((status & requiredForMotion) == requiredForMotion) &&
           waited_ms < RT_STOP_CAUSE_WAIT_MS && !bRtStopRequested)
    {
        mpClkAnnounce(MP_INTERPOLATION_CLK);
        waited_ms += g_Ros_Controller.interpolPeriod;
        status = Ros_RtMotionControl_ReadRobotStatus();
    }

    return status;
}

//Returns whether the robot job is running (not held or stopped). If the signal can't be read,
//returns TRUE, so a read error doesn't end the session.
static BOOL Ros_RtMotionControl_IsJobRunning()
{
    USHORT operating = 0;

    if (mpReadIO(&g_Ros_Controller.ioStatusAddr[IO_ROBOTSTATUS_OPERATING], &operating, 1) != 0)
        return TRUE;

    return (operating != 0);
}
