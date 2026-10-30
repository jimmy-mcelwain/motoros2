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

static void Ros_RtMotionControl_InitJointSpace(MP_EXPOS_DATA* moveData);
static bool Ros_RtMotionControl_InitCartesian(MP_EXPOS_DATA* moveData);
static bool Ros_RtMotionControl_ParseJointSpace(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData);
static bool Ros_RtMotionControl_ParseCartesian(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData);
static void Ros_RtMotionControl_PopulateReplyMessage(MOTION_MODE mode, RtPacket* command, RtReply* reply);
static bool Ros_RtMotionControl_CheckForMotionDeviation(MOTION_MODE mode, int* tools);
static void Ros_RtMotionControl_PurgeBufferedPackets();
static UINT32 Ros_RtMotionControl_ReadRobotStatus();
static UINT32 Ros_RtMotionControl_WaitForStopCause();
static BOOL Ros_RtMotionControl_IsJobRunning();
static EndReason Ros_RtMotionControl_DiscardEarlyPackets(const struct sockaddr_in* commander, UINT32 executedSequenceId, int* endDetail);
static EndReason Ros_RtMotionControl_MultipleSources(const struct sockaddr_in* commander, const struct sockaddr_in* sender, int* endDetail);
static int Ros_RtMotionControl_WaitForPacket();
static double Ros_RtMotionControl_RotationAngle(const Quaternion* from, const Quaternion* to);

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

static int sockRtCommandListener = -1;

static LONG prevRtCmdPosition[MAX_GROUPS][MAX_AXES];
static LONG howMuchShouldIHaveMoved[MAX_GROUPS][MAX_AXES];

//Cartesian mode: the tool which prevRtCmdPosition was read with (-1 = not read yet).
//A TCP pose is only comparable with another pose of the same tool.
static int rtDeviationCheckTool[MAX_GROUPS];

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

    //=========================================================================================

    bzero(prevRtCmdPosition, MAX_GROUPS * MAX_AXES * sizeof(LONG));
    bzero(howMuchShouldIHaveMoved, MAX_GROUPS * MAX_AXES * sizeof(LONG));
    bzero(residualIncrement, sizeof(residualIncrement));

    if (mode == MOTION_MODE_RT_JOINT)
        Ros_RtMotionControl_InitJointSpace(&moveData);
    else
    {
        if (!Ros_RtMotionControl_InitCartesian(&moveData))
        {
            bRtSessionRunning = FALSE;
            bRtTaskRunning = FALSE;
            return; //abort rt session
        }
    }

    Ros_Debug_BroadcastMsg("Starting RT session");

    robotStatus = Ros_RtMotionControl_ReadRobotStatus();

    //=========================================================================================
    while (TRUE)
    {
        int waitResult = Ros_RtMotionControl_WaitForPacket();
        if (waitResult > 0)
        {
            client_addr_len = sizeof(client_addr);
            bytes_received = mpRecvFrom(sockRtCommandListener, receiveBuffer, sizeof(receiveBuffer), 0, (struct sockaddr*)&client_addr, &client_addr_len);
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
                if (incomingCommand.packetType != PacketType_Joint_Increments)
                {
                    packetTypeOK = false;
                    Ros_Debug_BroadcastMsg("ERROR: The packet type does not match the control_mode specified in start_rt_mode (Joint Increments)");
                }
                break;
            case MOTION_MODE_RT_CARTESIAN:
                if (incomingCommand.packetType != PacketType_Cart_Increments)
                {
                    packetTypeOK = false;
                    Ros_Debug_BroadcastMsg("ERROR: The packet type does not match the control_mode specified in start_rt_mode (Cartesian Increments)");
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

            bool bParsedOK;
            if (mode == MOTION_MODE_RT_JOINT)
                bParsedOK = Ros_RtMotionControl_ParseJointSpace(&incomingCommand, &moveData);
            else
                bParsedOK = Ros_RtMotionControl_ParseCartesian(&incomingCommand, &moveData);

            if (!bParsedOK)
            {
                endReason = EndReason_InvalidIncrement;
                bEndReplyToSender = true;
                break; //drop the connection
            }

            motionDeviationDetected = Ros_RtMotionControl_CheckForMotionDeviation(mode, incomingCommand.toolIndex);

            // Send increment to robot
            int ret = mpExRcsIncrementMove(&moveData);
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

            //send status back to the PC and notify it that I'm ready for another packet
            previousSequenceId = incomingCommand.sequenceId;
            Ros_RtMotionControl_PopulateReplyMessage(mode, &incomingCommand, &outgoingReply);
            outgoingReply.replyType = ReplyType_Feedback;
            outgoingReply.endReason = EndReason_None;
            outgoingReply.robotStatus = robotStatus;
            outgoingReply.motionDeviationDetected = motionDeviationDetected;

            //Discard anything which arrived while waiting for the interpolation clock. This must
            //happen before the reply is sent, so a client which waits for the reply can't be affected.
            EndReason discardReason = Ros_RtMotionControl_DiscardEarlyPackets(&previous_client_addr, incomingCommand.sequenceId, &endDetail);
            if (discardReason != EndReason_None)
            {
                endReason = discardReason;
                break; //drop the connection
            }

            mpSendTo(sockRtCommandListener, (char*)&outgoingReply, sizeof(RtReply), 0, (struct sockaddr*)&client_addr, client_addr_len);

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

            //track how big the increment SHOULD have been
            //we'll compare next cycle to see what actually happened
            bzero(howMuchShouldIHaveMoved, MAX_GROUPS * MAX_AXES * sizeof(LONG));
            for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
            {
                for (int axis = 0; axis < MAX_AXES; axis += 1)
                {
                    howMuchShouldIHaveMoved[groupIndex][axis] = moveData.grp_pos_info[groupIndex].pos[axis];
                }
            }
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

    //must be last, as Ros_RtMotionControl_StopSession waits for this
    bRtTaskRunning = FALSE;
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
    MP_CTRL_GRP_SEND_DATA ctrlGroup;
    MP_PULSE_POS_RSP_DATA cmdPulse;

    bzero(moveData, sizeof(MP_EXPOS_DATA));

    for (i = 0; i < g_Ros_Controller.numGroup; i++)
    {
        moveData->ctrl_grp |= (0x01 << i);
        moveData->grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(g_Ros_Controller.ctrlGroups[i]);
        moveData->grp_pos_info[i].pos_tag.data[3] = MP_INC_PULSE_DTYPE;


        ctrlGroup.sCtrlGrp = g_Ros_Controller.ctrlGroups[i]->groupId;
        mpGetPulsePos(&ctrlGroup, &cmdPulse);
        memcpy(prevRtCmdPosition[i], cmdPulse.lPos, sizeof(cmdPulse.lPos));
    }
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
        rtDeviationCheckTool[i] = -1;
    }

    return true;
}

//A joint may be commanded this much more than its maximum increment per cycle, so that a client
//which computes the maximum from rounded conversion factors isn't rejected. What is sent is
//still limited to the maximum; the excess is sent in a later cycle.
#define RT_MAX_INCREMENT_TOLERANCE_PULSES   0.001

//Converts an increment (already scaled to motoman units) to the nearest whole number of
//units, and carries the rest (up to half a unit, either way) over to the next cycle.
static LONG Ros_RtMotionControl_TakeWholeUnits(double increment, double* residual)
{
    double total = increment + *residual;
    LONG whole = (LONG)((total < 0.0) ? (total - 0.5) : (total + 0.5));

    *residual = total - whole;
    return whole;
}

static bool Ros_RtMotionControl_IsValidTool(int groupNo, int toolIndex)
{
    if (toolIndex < 0 || toolIndex > MAX_VALID_TOOL_INDEX)
    {
        Ros_Debug_BroadcastMsg("ERROR: Group [%d] Tool [%d] is invalid (must be 0 - %d)", groupNo, toolIndex, MAX_VALID_TOOL_INDEX);
        return false;
    }
    return true;
}

static bool Ros_RtMotionControl_ParseJointSpace(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    int axis, groupNo;
    double pulsesPerUnit;
    double pulses;
    double motoOrderDelta[MP_GRP_AXES_NUM];
    double const* delta;

    // For each control group, convert radians (or meters) to pulses and prepare moveData
    for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[groupNo];

        if (!Ros_RtMotionControl_IsValidTool(groupNo, incomingCommand->toolIndex[groupNo]))
            return false;

        //joints must be in moto-order
        if (sessionJointOrder == motoros2_interfaces__msg__JointOrderEnum__JOINT_STATES)
        {
            Ros_CtrlGroup_ConvertSequentialJointOrderToMotoJointOrder(ctrlGroup, incomingCommand->delta[groupNo], motoOrderDelta);
            delta = motoOrderDelta;
        }
        else
            delta = incomingCommand->delta[groupNo];

        //Check every axis, not just the first numAxes. Some robots have gaps in their
        //axis layout. (ex: a 4-axis palletizer (SLU--T-) has 4 axes, but T is at index 5)
        for (axis = 0; axis < MP_GRP_AXES_NUM; axis++)
        {
            if (Ros_CtrlGroup_IsInvalidAxis(ctrlGroup, axis))
            {
                moveData->grp_pos_info[groupNo].pos[axis] = 0;
                continue;
            }

            if (ctrlGroup->axisType.type[axis] == AXIS_ROTATION)
                pulsesPerUnit = ctrlGroup->pulseToRad.PtoR[axis];
            else if (ctrlGroup->axisType.type[axis] == AXIS_LINEAR)
                pulsesPerUnit = ctrlGroup->pulseToMeter.PtoM[axis];
            else
                pulsesPerUnit = 1.0;

            pulses = delta[axis] * pulsesPerUnit;

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

//No maximum speed is documented for the elbow angle, so the TCP rotation limit is used. Like the
//TCP limits, this only catches client errors (ex: wrong units). The controller limits the joints.
#define RT_MAX_ELBOW_SPEED_DEG_PER_MS   0.36

static bool Ros_RtMotionControl_ParseCartesian(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    int groupNo;
    double* delta;
    double magnitude;
    double norm;
    double rotation;
    double rx, ry, rz;
    Quaternion q;

    // 1500 mm/sec is 1.5 mm/ms
    const double max_speed_mm_per_ms = 1.5;

    // 360 deg/sec is 0.36 deg/ms (the controller's maximum TCP rotation speed)
    const double max_rotation_deg_per_ms = 0.36;

    // For each control group, convert incoming command and prepare moveData
    for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        delta = incomingCommand->delta[groupNo];

        if (!Ros_RtMotionControl_IsValidTool(groupNo, incomingCommand->toolIndex[groupNo]))
            return false;

        moveData->grp_pos_info[groupNo].pos_tag.data[2] = incomingCommand->toolIndex[groupNo];

        //=========================================================================================
        //Translation
        magnitude = METERS_TO_MILLIMETERS(sqrt(pow(delta[TCP_X], 2) + //x^2
                                               pow(delta[TCP_Y], 2) + //y^2
                                               pow(delta[TCP_Z], 2))); //z^2

        //Written so that NaN and infinity also fail this check
        if (!(magnitude <= (max_speed_mm_per_ms * g_Ros_Controller.interpolPeriod)))
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

        norm = sqrt((q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w));
        if (norm == 0.0)
        {
            //An all-zero packet is treated as "no rotation", so a zero-filled
            //packet can be used as a keep-alive
            q.w = 1.0;
        }
        else if (!(fabs(norm - 1.0) <= RT_QUATERNION_NORM_TOLERANCE)) //NaN also fails this check
        {
            Ros_Debug_BroadcastMsg("ERROR: Group [%d] The rotation must be a unit quaternion (length is [%f])", groupNo, norm);
            return false;
        }
        else
        {
            q.x /= norm;
            q.y /= norm;
            q.z /= norm;
            q.w /= norm;
        }

        rotation = Ros_RtMotionControl_RotationAngle(NULL, &q) * DEGREES_PER_RAD;
        if (rotation > (max_rotation_deg_per_ms * g_Ros_Controller.interpolPeriod))
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

void Ros_RtMotionControl_OpenSocket()
{
    struct sockaddr_in server_addr;

    sockRtCommandListener = mpSocket(AF_INET, SOCK_DGRAM, 0);
    if (sockRtCommandListener < 0)
    {
        Ros_Debug_BroadcastMsg("ERROR: Could not allocate socket for RT interface");
        motoRosAssert_withMsg(false, SUBCODE_FAIL_ALLOCATE_RT_CMD_SOCKET, "Failed to allocate RT socket");
    }

    // Bind socket to port
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = mpHtons(g_nodeConfigSettings.rt_listener_udp_port_number);

    if (mpBind(sockRtCommandListener, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
        Ros_Debug_BroadcastMsg("ERROR: Failed to bind UDP socket for real-time motion control");
        mpClose(sockRtCommandListener);
        sockRtCommandListener = -1; 
        motoRosAssert_withMsg(false, SUBCODE_FAIL_BIND_RT_SOCKET, "Failed to bind RT socket");
    }
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
    long degrees[MP_GRP_AXES_NUM];
    BITSTRING figure;
    MP_COORD coord;
    MP_CTRL_GRP_SEND_DATA ctrlGroup;
    MP_PULSE_POS_RSP_DATA cmdPulse;

    bzero(reply, sizeof(RtReply));
    bzero(degrees, sizeof(long) * MP_GRP_AXES_NUM);

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
            for (int axis = 0; axis < MP_GRP_AXES_NUM; axis += 1)
            {
                //if (group->axisType.type[axis] == AXIS_ROTATION)
                    degrees[axis] = RAD_TO_DEG_0001(reply->feedbackPositionJoints[groupIndex][axis]);
            }

            //Cart
            mpConvAxesToCartPos(groupIndex, degrees, command->toolIndex[groupIndex], &figure, &coord);

            Quaternion qFb;
            QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(coord.rx, coord.ry, coord.rz, &qFb);

            reply->feedbackPositionCartesian[groupIndex][TCP_X] = MICROMETERS_TO_METERS(coord.x);
            reply->feedbackPositionCartesian[groupIndex][TCP_Y] = MICROMETERS_TO_METERS(coord.y);
            reply->feedbackPositionCartesian[groupIndex][TCP_Z] = MICROMETERS_TO_METERS(coord.z);

            reply->feedbackPositionCartesian[groupIndex][TCP_Qx] = qFb.x;
            reply->feedbackPositionCartesian[groupIndex][TCP_Qy] = qFb.y;
            reply->feedbackPositionCartesian[groupIndex][TCP_Qz] = qFb.z;
            reply->feedbackPositionCartesian[groupIndex][TCP_Qw] = qFb.w;
            reply->feedbackPositionCartesian[groupIndex][TCP_Re] = DEG_0001_TO_RAD(coord.ex1);
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
            //deg
            for (int axis = 0; axis < MP_GRP_AXES_NUM; axis += 1)
                degrees[axis] = RAD_TO_DEG_0001(reply->previousCommandPositionJoints[groupIndex][axis]);

            //Cart
            mpConvAxesToCartPos(groupIndex, degrees, command->toolIndex[groupIndex], &figure, &coord);

            Quaternion qCmd;
            QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(coord.rx, coord.ry, coord.rz, &qCmd);

            reply->previousCommandPositionCartesian[groupIndex][TCP_X] = MICROMETERS_TO_METERS(coord.x);
            reply->previousCommandPositionCartesian[groupIndex][TCP_Y] = MICROMETERS_TO_METERS(coord.y);
            reply->previousCommandPositionCartesian[groupIndex][TCP_Z] = MICROMETERS_TO_METERS(coord.z);

            reply->previousCommandPositionCartesian[groupIndex][TCP_Qx] = qCmd.x;
            reply->previousCommandPositionCartesian[groupIndex][TCP_Qy] = qCmd.y;
            reply->previousCommandPositionCartesian[groupIndex][TCP_Qz] = qCmd.z;
            reply->previousCommandPositionCartesian[groupIndex][TCP_Qw] = qCmd.w;
            reply->previousCommandPositionCartesian[groupIndex][TCP_Re] = DEG_0001_TO_RAD(coord.ex1);
        }

        //after the FK, which needs moto-order
        Ros_RtMotionControl_ToSessionJointOrder(group, reply->previousCommandPositionJoints[groupIndex]);
    }
}

static bool Ros_RtMotionControl_CheckForMotionDeviation(MOTION_MODE mode, int* tools)
{
    MP_CTRL_GRP_SEND_DATA ctrlGroup;
    MP_PULSE_POS_RSP_DATA cmdPulse;
    MP_CARTPOS_EX_SEND_DATA cartSendData;
    MP_CART_POS_RSP_DATA_EX cartRespData;
    bool returnValue = FALSE;

    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
    {
        //================================================================================
        //Did the controller execute the previous increment as sent? (ex: the FSU speed limit)
        //================================================================================

        LONG difference;
        LONG howMuchDidIActuallyMove[MAX_AXES];
        LONG previousPosition[MAX_AXES];
        bzero(howMuchDidIActuallyMove, sizeof(LONG) * MAX_AXES);
        memcpy(previousPosition, prevRtCmdPosition[groupIndex], sizeof(previousPosition));

        if (mode == MOTION_MODE_RT_JOINT)
        {
            //Not Ros_CtrlGroup_GetPulsePosCmd. That is only for converting incoming absolute
            //positions. See https://github.com/Yaskawa-Global/motoros2/discussions/455
            ctrlGroup.sCtrlGrp = g_Ros_Controller.ctrlGroups[groupIndex]->groupId;
            mpGetPulsePos(&ctrlGroup, &cmdPulse);
        }
        else if (mode == MOTION_MODE_RT_CARTESIAN)
        {
            cartSendData.sRobotNo = groupIndex;
            cartSendData.sFrame = 1; //1 = RF

            if (rtDeviationCheckTool[groupIndex] < 0)
            {
                //First command. Nothing has been commanded yet, so there is nothing to check.
                //Read the starting pose with the tool this command uses.
                cartSendData.sToolNo = tools[groupIndex];
                mpGetCartPosEx(&cartSendData, &cartRespData);
                memcpy(prevRtCmdPosition[groupIndex], cartRespData.lPos, sizeof(LONG) * MAX_AXES);
                rtDeviationCheckTool[groupIndex] = tools[groupIndex];
                continue;
            }

            //Check the previous increment with the tool it was commanded with
            cartSendData.sToolNo = rtDeviationCheckTool[groupIndex];
            mpGetCartPosEx(&cartSendData, &cartRespData);
        }

        // Check if pulses (or mm's) are missing from last increment.
        // Get the current controller command position and substract the previous command position
        // and check if it matches the amount if increment sent last cycle
        for (int axis = 0; axis < MP_GRP_AXES_NUM; axis += 1)
        {
            if (mode == MOTION_MODE_RT_CARTESIAN)
            {
                howMuchDidIActuallyMove[axis] = cartRespData.lPos[axis] - prevRtCmdPosition[groupIndex][axis];
                prevRtCmdPosition[groupIndex][axis] = cartRespData.lPos[axis];
            }
			else if (mode == MOTION_MODE_RT_JOINT)
            {
                howMuchDidIActuallyMove[axis] = cmdPulse.lPos[axis] - prevRtCmdPosition[groupIndex][axis];
                prevRtCmdPosition[groupIndex][axis] = cmdPulse.lPos[axis];
            }
        }
		
        if (mode == MOTION_MODE_RT_JOINT)
        {
            //pulses
            for (int axis = 0; axis < MP_GRP_AXES_NUM; axis += 1)
            {
                difference = howMuchShouldIHaveMoved[groupIndex][axis] - howMuchDidIActuallyMove[axis];
                if (abs(difference) > g_nodeConfigSettings.rt_motion_deviation_threshold_pulses)
                    returnValue = TRUE;
            }
        }
        else if (mode == MOTION_MODE_RT_CARTESIAN)
        {
            //Translation (micrometers)
            for (int axis = TCP_X; axis <= TCP_Z; axis += 1)
            {
                difference = howMuchShouldIHaveMoved[groupIndex][axis] - howMuchDidIActuallyMove[axis];
                if (abs(difference) > g_nodeConfigSettings.rt_motion_deviation_threshold_micrometers)
                    returnValue = TRUE;
            }

            //Rotation (0.0001 degrees)
            //The orientation is in Euler angles, so a rotation about one axis can change all
            //three of them. The individual angles can't be compared. Instead, compare the size
            //of the rotation which was commanded with the size of the rotation which happened.
            Quaternion qCommanded, qPrevious, qCurrent;
            QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(howMuchShouldIHaveMoved[groupIndex][3],
                howMuchShouldIHaveMoved[groupIndex][4], howMuchShouldIHaveMoved[groupIndex][5], &qCommanded);
            QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(previousPosition[3], previousPosition[4], previousPosition[5], &qPrevious);
            QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(cartRespData.lPos[3], cartRespData.lPos[4], cartRespData.lPos[5], &qCurrent);

            double angleDifference = Ros_RtMotionControl_RotationAngle(NULL, &qCommanded) - Ros_RtMotionControl_RotationAngle(&qPrevious, &qCurrent);
            if (fabs(RAD_TO_DEG_0001(angleDifference)) > g_nodeConfigSettings.rt_motion_deviation_threshold_deg_0001)
                returnValue = TRUE;

            //Elbow angle (0.0001 degrees)
            if (g_Ros_Controller.ctrlGroups[groupIndex]->numAxes >= 7)
            {
                difference = howMuchShouldIHaveMoved[groupIndex][6] - howMuchDidIActuallyMove[6];
                if (abs(difference) > g_nodeConfigSettings.rt_motion_deviation_threshold_deg_0001)
                    returnValue = TRUE;
            }

            if (tools[groupIndex] != rtDeviationCheckTool[groupIndex])
            {
                //This command changes the tool. Re-read the pose with the new tool, so the
                //next cycle checks this command's increment against the same TCP.
                cartSendData.sToolNo = tools[groupIndex];
                mpGetCartPosEx(&cartSendData, &cartRespData);
                memcpy(prevRtCmdPosition[groupIndex], cartRespData.lPos, sizeof(LONG) * MAX_AXES);
                rtDeviationCheckTool[groupIndex] = tools[groupIndex];
            }
        }
    }
    return returnValue;
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
