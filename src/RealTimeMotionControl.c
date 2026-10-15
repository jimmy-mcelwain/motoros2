//RealTimeMotionControl.c

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

// Based loosely on @adv4ncr's modifications to MotoROS1 for real-time control through ROS2.
// https://github.com/adv4ncr/motoman_ROS2/blob/cdf63a592596ff711df842680a1e4c730fd547a7/controller_driver/RealTimeMotionServer.c

#include "MotoROS.h"

void Ros_RtMotionControl_InitJointSpace(MP_EXPOS_DATA* moveData);
bool Ros_RtMotionControl_InitCartesian(MP_EXPOS_DATA* moveData);
bool Ros_RtMotionControl_ParseJointSpace(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData);
bool Ros_RtMotionControl_ParseCartesian(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData);
void Ros_RtMotionControl_PopulateReplyMessage(MOTION_MODE mode, RtPacket* command, RtReply* reply);
bool Ros_RtMotionControl_CheckForMotionDeviation(MOTION_MODE mode, int* tools);
void Ros_RtMotionControl_PurgeBufferedPackets();
void Ros_RtMotionControl_SendRobotStatus();
static int Ros_RtMotionControl_WaitForPacket();

//How often the R/T task checks for a stop request while waiting for a command packet
#define RT_STOP_POLL_PERIOD_MS      100
//How long to wait for the R/T task to exit on its own before it gets deleted
#define RT_STOP_WAIT_TIMEOUT_MS     1000

static volatile BOOL bRtSessionRunning = FALSE;
static volatile BOOL bRtStopRequested = FALSE;

static int sockRtCommandListener = -1;
static int sockRtStatusSender = -1;
static int tidRobotStatus = -1;

static struct sockaddr_in client_addr_status_messages;

static LONG prevRtCmdPosition[MAX_GROUPS][MAX_AXES];
static LONG howMuchShouldIHaveMoved[MAX_GROUPS][MAX_AXES];

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

    RtPacket incomingCommand;
    RtPacket lastExecutedCommand;
    RtReply outgoingReply;

    EndReason endReason = EndReason_InternalError;
    int endDetail = 0;
    bool bEndReplyToSender = false; //the session ended because of the packet which was just received

    UINT32 previousSequenceId = 0;

    bool motionDeviationDetected;

    bool packetTypeOK;

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
            mpSuspendSelf;
            return;
        }
    }

    Ros_Debug_BroadcastMsg("Starting RT session");

    Ros_Debug_BroadcastMsg("Flushing stale packets from socket buffer...");
    Ros_RtMotionControl_PurgeBufferedPackets();

    //Spin up a separate normal-priorty thread to send out the robot status info
    tidRobotStatus = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE,
        (FUNCPTR)Ros_RtMotionControl_SendRobotStatus,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    //=========================================================================================
    while (TRUE)
    {
        int waitResult = Ros_RtMotionControl_WaitForPacket();
        if (waitResult > 0)
        {
            bzero(&incomingCommand, sizeof(incomingCommand));
            client_addr_len = sizeof(client_addr);
            bytes_received = mpRecvFrom(sockRtCommandListener, (char*)&incomingCommand, sizeof(RtPacket), 0, (struct sockaddr*)&client_addr, &client_addr_len);
            if (bytes_received < 0)
            {
                Ros_Debug_BroadcastMsg("ERROR: recvFrom returned an error");
                endReason = EndReason_InternalError;
                break;
            }

            //Only allow a single commander. A second sender means the system is misconfigured,
            //so the session is ended rather than letting two clients command the robot.
            if (!bFirstRecv && memcmp(&client_addr.sin_addr.s_addr, &previous_client_addr.sin_addr.s_addr, sizeof(UINT32)) != 0)
            {
                Ros_Debug_BroadcastMsg("ERROR: Received command packets from multiple sources (0x%08X and 0x%08X)",
                                        (UINT32)previous_client_addr.sin_addr.s_addr,
                                        (UINT32)client_addr.sin_addr.s_addr);
                endReason = EndReason_MultipleSources;
                endDetail = (int)client_addr.sin_addr.s_addr;
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

            if (bFirstRecv)
            {
                //flag is cleared down below

                //setup destination for status messages
                memset(&client_addr_status_messages, 0, sizeof(client_addr_status_messages));
                client_addr_status_messages.sin_family = AF_INET;
                client_addr_status_messages.sin_addr.s_addr = client_addr.sin_addr.s_addr;
                client_addr_status_messages.sin_port = mpHtons(atoi(g_nodeConfigSettings.rt_status_udp_port_number));
            }

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
            outgoingReply.motionDeviationDetected = motionDeviationDetected;

            Ros_RtMotionControl_PurgeBufferedPackets(); //in case user sent multiple commands while this was sleeping
            mpSendTo(sockRtCommandListener, (char*)&outgoingReply, sizeof(RtReply), 0, (struct sockaddr*)&client_addr, client_addr_len);

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

    mpDeleteTask(tidRobotStatus);

    //=========================================================================================
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

        mpSendTo(sockRtCommandListener, (char*)&outgoingReply, sizeof(RtReply), 0, (struct sockaddr*)endDestination, sizeof(struct sockaddr_in));
    }

    Ros_Debug_BroadcastMsg("Ending Rt Session (reason: %d)", endReason);

    //Ros_RtMotionControl_StopSession waits for this
    bRtSessionRunning = FALSE;
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

//Must be called before the R/T task is created
void Ros_RtMotionControl_PrepareSession()
{
    bRtStopRequested = FALSE;
    bRtSessionRunning = TRUE;
}

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

    for (waited_ms = 0; bRtSessionRunning && waited_ms < RT_STOP_WAIT_TIMEOUT_MS; waited_ms += 10)
        Ros_Sleep(10);

    return !bRtSessionRunning;
}

//Called after the R/T task has been forcibly deleted, or could not be created
void Ros_RtMotionControl_AbortSession()
{
    bRtSessionRunning = FALSE;
}


void Ros_RtMotionControl_InitJointSpace(MP_EXPOS_DATA* moveData)
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


        ctrlGroup.sCtrlGrp = g_Ros_Controller.ctrlGroups[i]->groupId;;
        mpGetPulsePos(&ctrlGroup, &cmdPulse);
        memcpy(prevRtCmdPosition[i], cmdPulse.lPos, sizeof(cmdPulse.lPos));
    }
}

bool Ros_RtMotionControl_InitCartesian(MP_EXPOS_DATA* moveData)
{
    int i;
    MP_CARTPOS_EX_SEND_DATA cartSendData;
    MP_CART_POS_RSP_DATA_EX cartRespData;
    MP_GET_TOOL_NO_RSP_DATA getToolResp;

    bzero(moveData, sizeof(MP_EXPOS_DATA));

    moveData->m_ctrl_grp = 0;
    moveData->s_ctrl_grp = 0;
    
    for (i = 0; i < g_Ros_Controller.numGroup; i++)
    {
        CtrlGroup* group = g_Ros_Controller.ctrlGroups[i];

        if (Ros_CtrlGroup_IsRobot(group)) //is a robot and not an external axis
            cartSendData.sFrame = 1; //1 = RF
        //TODO: Test on an actual track. I don't think you can get the cartesian of the track alone.
        //      I think that you can only get robot position, but in Base Frame.
//        else if (Ros_CtrlGroup_IsBase(group)) //is a base track
//            cartSendData.sFrame = 0; //0 = BF
        else
        {
            Ros_Debug_BroadcastMsg("ERROR: Group [%d] is an external positioner. Cartesian control mode is not supported for this group.", i);
            return false;
        }

        moveData->ctrl_grp |= (1 << i);
        moveData->grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(g_Ros_Controller.ctrlGroups[i]);
        moveData->grp_pos_info[i].pos_tag.data[3] = MP_INC_RF_DTYPE;

        //NOTE: This isn't the best method for this. During testing, I had tool #2 selected
        //      on the pendant, but I was commanding increments on tool #0. Because of this,
        //      the first motion on each axis would trigger the FSU detection mechanism. But
        //      it immediately recovers after one cycle.
        mpGetToolNo(group->groupId, &getToolResp);

        cartSendData.sRobotNo = i;

        cartSendData.sToolNo = getToolResp.sToolNo;
        mpGetCartPosEx(&cartSendData, &cartRespData);
        memcpy(prevRtCmdPosition[i], cartRespData.lPos, sizeof(LONG) * MAX_AXES);
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

bool Ros_RtMotionControl_ParseJointSpace(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    int axis, groupNo;
    double pulsesPerUnit;
    double pulses;

    // For each control group, convert radians (or meters) to pulses and prepare moveData
    for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        CtrlGroup* ctrlGroup = g_Ros_Controller.ctrlGroups[groupNo];

        //joints must be in moto-order
        //
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

            pulses = incomingCommand->delta[groupNo][axis] * pulsesPerUnit;

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

//Only meant to reject garbage (ex: NaN). The controller enforces the actual limits.
#define RT_MAX_ELBOW_INCREMENT_RAD      3.14159265358979

bool Ros_RtMotionControl_ParseCartesian(RtPacket* incomingCommand, MP_EXPOS_DATA* moveData)
{
    int groupNo;
    double* delta;
    double magnitude;
    double norm;
    double rx, ry, rz;
    Quaternion q;

    // 1500 mm/sec is 1.5 mm/ms
    const double max_speed_mm_per_ms = 1.5;

    // For each control group, convert incoming command and prepare moveData
    for (groupNo = 0; groupNo < g_Ros_Controller.numGroup; groupNo += 1)
    {
        delta = incomingCommand->delta[groupNo];

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

        QuatConversion_GeomMsgsQuaternion_To_EulerRad(&q, &rx, &ry, &rz);

        moveData->grp_pos_info[groupNo].pos[3] = Ros_RtMotionControl_TakeWholeUnits(RAD_TO_DEG_0001(rx), &residualIncrement[groupNo][3]);
        moveData->grp_pos_info[groupNo].pos[4] = Ros_RtMotionControl_TakeWholeUnits(RAD_TO_DEG_0001(ry), &residualIncrement[groupNo][4]);
        moveData->grp_pos_info[groupNo].pos[5] = Ros_RtMotionControl_TakeWholeUnits(RAD_TO_DEG_0001(rz), &residualIncrement[groupNo][5]);

        //=========================================================================================
        //Elbow angle (7-axis robots)
        if (!(fabs(delta[TCP_Re]) <= RT_MAX_ELBOW_INCREMENT_RAD))
        {
            Ros_Debug_BroadcastMsg("ERROR: Group [%d] The elbow angle increment [%f rad] is invalid", groupNo, delta[TCP_Re]);
            return false;
        }

        //Other robots have no elbow angle
        moveData->grp_pos_info[groupNo].pos[6] = 0;
        if (g_Ros_Controller.ctrlGroups[groupNo]->numAxes >= 7)
            moveData->grp_pos_info[groupNo].pos[6] = Ros_RtMotionControl_TakeWholeUnits(RAD_TO_DEG_0001(delta[TCP_Re]), &residualIncrement[groupNo][6]);

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
    server_addr.sin_port = mpHtons(atoi(g_nodeConfigSettings.rt_listener_udp_port_number));

    if (mpBind(sockRtCommandListener, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
        Ros_Debug_BroadcastMsg("ERROR: Failed to bind UDP socket for real-time motion control");
        mpClose(sockRtCommandListener);
        sockRtCommandListener = -1; 
        motoRosAssert_withMsg(false, SUBCODE_FAIL_BIND_RT_SOCKET, "Failed to bind RT socket");
    }

    //=========================================================================================
    sockRtStatusSender = mpSocket(AF_INET, SOCK_DGRAM, 0);
    if (sockRtStatusSender < 0)
    {
        Ros_Debug_BroadcastMsg("ERROR: Could not allocate Status socket for RT interface");
        motoRosAssert_withMsg(false, SUBCODE_FAIL_ALLOCATE_RT_FB_SOCKET, "Failed to allocate RT socket");
    }
}

void Ros_RtMotionControl_PopulateReplyMessage(MOTION_MODE mode, RtPacket* command, RtReply* reply)
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
        Ros_CtrlGroup_GetFBPulsePos(group, pulsePos_moto);

        //Angles (or meters for a linear track)
        Ros_CtrlGroup_ConvertMotoUnitsToRosUnits(group, pulsePos_moto, reply->feedbackPositionJoints[groupIndex]);

        if (Ros_CtrlGroup_IsRobot(group)) //is a robot and not an external axis
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

        //================================================================================
        //CMD pos
        //================================================================================
        //Should this be Ros_CtrlGroup_GetPulsePosCmd?
        //Answer: No, it should not. That should only be used when converting incoming
        //        positional commands that contain an absolute position.
        //        See https://github.com/Yaskawa-Global/motoros2/discussions/455
        ctrlGroup.sCtrlGrp = g_Ros_Controller.ctrlGroups[groupIndex]->groupId;
        mpGetPulsePos(&ctrlGroup, &cmdPulse);

        //rad (or meter for linear track)
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
    }
}

bool Ros_RtMotionControl_CheckForMotionDeviation(MOTION_MODE mode, int* tools)
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
        bzero(howMuchDidIActuallyMove, sizeof(LONG) * MAX_AXES);

        if (mode == MOTION_MODE_RT_JOINT)
        {
            //Should this be Ros_CtrlGroup_GetPulsePosCmd?
            //Answer: No, it should not. That should only be used when converting incoming
            //        positional commands that contain an absolute position.
            //        See https://github.com/Yaskawa-Global/motoros2/discussions/455
            ctrlGroup.sCtrlGrp = g_Ros_Controller.ctrlGroups[groupIndex]->groupId;
            mpGetPulsePos(&ctrlGroup, &cmdPulse);
        }
        else if (mode == MOTION_MODE_RT_CARTESIAN)
        {
            cartSendData.sRobotNo = groupIndex;
            cartSendData.sFrame = 1; //1 = RF
            cartSendData.sToolNo = tools[groupIndex];
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
		
        for (int axis = 0; axis < MP_GRP_AXES_NUM; axis += 1)
        {
            //When working in cartesian space, we're only going to monitor the translation.
            //1. There is no FSU speed limit for rotation. So it's moot.
            //2. When rotating by some increment, that rotation gets 'spread out' over multiple
            //   axes. Even if I put all of my commanded increment into a single axis, all 
            //   three of them are going to react. So, the cmd-value of my intended axis may
            //   not be the value I expect.
            if (mode == MOTION_MODE_RT_CARTESIAN && axis > TCP_Z)
            {
                break;
            }

            difference = howMuchShouldIHaveMoved[groupIndex][axis] - howMuchDidIActuallyMove[axis];
            if (abs(difference) > MAX_INCREMENT_DEVIATION_FOR_FSU_DETECTION)
            {
                //Ros_Debug_BroadcastMsg("howMuchShouldIHaveMoved[%d][%d] = %d", groupIndex, axis, howMuchShouldIHaveMoved[groupIndex][axis]);
                //Ros_Debug_BroadcastMsg("howMuchDidIActuallyMove[%d] = %d", axis, howMuchDidIActuallyMove[axis]);
                //Ros_Debug_BroadcastMsg("difference = %d", difference);
                //Ros_Debug_BroadcastMsg("---------");

                returnValue = TRUE;
            }
        }
    }
    return returnValue;
}

void Ros_RtMotionControl_PurgeBufferedPackets()
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

        if (mpSelect(sockRtCommandListener + 1, &fds, NULL, NULL, &tv) > 0)
        {
            mpRecvFrom(sockRtCommandListener, (char*)&incomingCommand, sizeof(RtPacket), 0, (struct sockaddr*)&client_addr, &client_addr_len);
        }
        else
            break;
    }
}

//Essentially a clone of the /robot_status topic. But decoupled from the industrial_msgs/RobotStatus type.
void Ros_RtMotionControl_SendRobotStatus()
{
    RobotState stateMsg;

    int client_addr_len = sizeof(client_addr_status_messages);

    //Duplicate logic of ControllerStatusIo
    MP_IO_INFO ioStatusAddr[IO_ROBOTSTATUS_MAX];            // Array of Specific Input Address representing the I/O status
    USHORT ioStatus[IO_ROBOTSTATUS_MAX];                    // Array storing the current status of the controller

    ioStatusAddr[IO_ROBOTSTATUS_ALARM_MINOR].ulAddr = 50011;       // Alarm
    ioStatusAddr[IO_ROBOTSTATUS_ALARM_SYSTEM].ulAddr = 50012;      // Alarm
    ioStatusAddr[IO_ROBOTSTATUS_ALARM_USER].ulAddr = 50013;        // Alarm
    ioStatusAddr[IO_ROBOTSTATUS_ERROR].ulAddr = 50014;             // Error
    ioStatusAddr[IO_ROBOTSTATUS_PLAY].ulAddr = 50054;              // Play
    ioStatusAddr[IO_ROBOTSTATUS_TEACH].ulAddr = 50053;             // Teach
    ioStatusAddr[IO_ROBOTSTATUS_REMOTE].ulAddr = 80011; //50056;   // Remote  // Modified E.M. 7/9/2013
    ioStatusAddr[IO_ROBOTSTATUS_OPERATING].ulAddr = 50070;         // Operating
    ioStatusAddr[IO_ROBOTSTATUS_HOLD].ulAddr = 50071;              // Hold
    ioStatusAddr[IO_ROBOTSTATUS_SERVO].ulAddr = 50073;             // Servo ON
    ioStatusAddr[IO_ROBOTSTATUS_ESTOP_EX].ulAddr = 80025;          // External E-Stop
    ioStatusAddr[IO_ROBOTSTATUS_ESTOP_PP].ulAddr = 80026;          // Pendant E-Stop
    ioStatusAddr[IO_ROBOTSTATUS_ESTOP_CTRL].ulAddr = 80027;        // Controller E-Stop
    ioStatusAddr[IO_ROBOTSTATUS_WAITING_ROS].ulAddr = IO_FEEDBACK_WAITING_MP_INCMOVE; // Job input signaling ready for external motion
    ioStatusAddr[IO_ROBOTSTATUS_INECOMODE].ulAddr = 50727;         // Energy Saving Mode
    ioStatusAddr[IO_ROBOTSTATUS_CONT_CYC_MODE].ulAddr = 50052;     // Continuous Cycle Mode
    ioStatusAddr[IO_ROBOTSTATUS_ALARM_MAJOR].ulAddr = 50010;       // Alarm
#if (YRC1000||YRC1000u)
    ioStatusAddr[IO_ROBOTSTATUS_PFL_STOP].ulAddr = 81702;          // PFL function stopped the motion
    ioStatusAddr[IO_ROBOTSTATUS_PFL_ESCAPE].ulAddr = 81703;        // PFL function escape from clamping motion
    ioStatusAddr[IO_ROBOTSTATUS_PFL_AVOIDING].ulAddr = 15120;      // PFL function avoidance operating
    ioStatusAddr[IO_ROBOTSTATUS_PFL_AVOID_JOINT].ulAddr = 15124;   // PFL function avoidance joint enabled
    ioStatusAddr[IO_ROBOTSTATUS_PFL_AVOID_TRANS].ulAddr = 15125;   // PFL function avoidance translation enabled
#endif

    //-------------------------------------------------------------------------------


    stateMsg.version = VERSION_OF_ROBOT_STATE_PACKET;

    while (TRUE)
    {
        Ros_Sleep(g_nodeConfigSettings.rt_status_sleep_period);

        //-------------------------------------------------------------------------------
        mpReadIO(ioStatusAddr, ioStatus, IO_ROBOTSTATUS_MAX);

        //-------------------------------------------------------------------------------
        stateMsg.drives_powered = (ioStatus[IO_ROBOTSTATUS_SERVO] != 0 && ioStatus[IO_ROBOTSTATUS_INECOMODE] != 0);
        stateMsg.e_stopped = (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ESTOP_EX] == 0)
                                || (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ESTOP_PP] == 0)
                                || (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_ESTOP_CTRL] == 0);
        stateMsg.play_mode = (g_Ros_Controller.ioStatus[IO_ROBOTSTATUS_PLAY] != 0);
        stateMsg.motion_possible = TRUE; //This task gets killed if the robot is in a state where motion isn't possible
        stateMsg.error = 0; //This task gets killed if the robot is in an error state
        stateMsg.error_code = 0;

        //-------------------------------------------------------------------------------
        mpSendTo(sockRtStatusSender, (char*)&stateMsg, sizeof(RobotState), 0, (struct sockaddr*)&client_addr_status_messages, client_addr_len);
    }
}
