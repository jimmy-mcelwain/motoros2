//mpMain.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

#if !(defined (DX200) || defined (YRC1000) || defined (YRC1000u))
#error MotoROS2 is only supported on DX2 and YRC1 generation controllers
#endif

void RosInitTask();

//Main entrypoint, called by M+ runtime
// cppcheck-suppress unusedFunction
void mpUsrRoot(int arg1, int arg2, int arg3, int arg4, int arg5, int arg6, int arg7, int arg8, int arg9, int arg10)
{
    //Creates and starts a new task in a seperate thread of execution.
    //All arguments will be passed to the new task if the function
    //prototype will accept them.
    int RosInitTaskID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)RosInitTask,
        arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
    if (RosInitTaskID == ERROR)
        mpSetAlarm(ALARM_TASK_CREATE_FAIL, APPLICATION_NAME " FAILED TO CREATE TASK", SUBCODE_INITIALIZATION);

    //Ends the initialization task.
    mpExitUsrRoot;
}

void Ros_Sleep(float milliseconds)
{
    mpTaskDelay(milliseconds / mpGetRtc()); //Tick length varies between controller models
}

//Report version info to display on pendant
void Ros_ReportVersionInfoToController()
{
    MP_APPINFO_SEND_DATA appInfoSendData;
    MP_STD_RSP_DATA stdResponseData;

    snprintf(appInfoSendData.AppName, MP_MAX_APP_NAME,
        "%s (%s)", APPLICATION_NAME, MOTOPLUS_LIBMICROROS_ROS2_CODENAME);
    snprintf(appInfoSendData.Version, MP_MAX_APP_VERSION,
        "%s", APPLICATION_VERSION);
    snprintf(appInfoSendData.Comment, MP_MAX_APP_COMMENT,
        "%s", "micro-ROS based ROS 2 interface");
    mpApplicationInfoNotify(&appInfoSendData, &stdResponseData); //don't care about return value
}

void RosInitTask()
{
    g_Ros_Controller.tidIncMoveThread = INVALID_TASK;

    //Check to see if another version of MotoROS2.out is running on this controller.
    motoRosAssert_withMsg(!Ros_IsOtherInstanceRunning(), SUBCODE_MULTIPLE_INSTANCES_DETECTED, "MotoROS2 - Multiple Instances");

    // init debug broadcast
    Ros_Debug_BroadcastMsg("---");
    Ros_Debug_BroadcastMsg(APPLICATION_NAME " %s - boot", APPLICATION_VERSION);
    Ros_Debug_BroadcastMsg("M+ libmicroros version: '%s'", MOTOPLUS_LIBMICROROS_VERSION_STR);
    Ros_Debug_BroadcastMsg("PlatformLib version: %u.%u.%u", MOTOROS_PLATFORM_LIB_MAJOR,
        MOTOROS_PLATFORM_LIB_MINOR, MOTOROS_PLATFORM_LIB_PATCH);

    //useful to see on debug-monitor
    Ros_Debug_LogToConsole("%s - boot", APPLICATION_VERSION);

#ifdef MOTOROS2_TESTING_ENABLE
    Ros_Debug_BroadcastMsg("===");
    Ros_Debug_BroadcastMsg("Performing unit tests");
    BOOL bTestResult = TRUE;
    bTestResult &= Ros_Testing_CtrlGroup();
    bTestResult &= Ros_Testing_RosMotoPlusConversionUtils();
    bTestResult &= Ros_Testing_ControllerStatusIO();
    bTestResult &= Ros_Testing_ActionServer_FJT();
    bTestResult ? Ros_Debug_BroadcastMsg("Testing SUCCESSFUL") : Ros_Debug_BroadcastMsg("!!! Testing FAILED !!!");
    Ros_Debug_BroadcastMsg("===");
#endif

    Ros_ConfigFile_Parse();

    Ros_ReportVersionInfoToController();

    //==================================
    Ros_Controller_SetIOState(IO_FEEDBACK_WAITING_MP_INCMOVE, FALSE);
    Ros_Controller_SetIOState(IO_FEEDBACK_MP_INCMOVE_DONE, FALSE);
    Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);
    Ros_Controller_SetIOState(IO_FEEDBACK_CONTROLLERRUNNING, TRUE);
    Ros_Controller_SetIOState(IO_FEEDBACK_AGENTCONNECTED, FALSE);

    Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, FALSE);

    Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_1, FALSE);
    Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_2, FALSE);
    Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_3, FALSE);
    Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_4, FALSE);
    Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_5, FALSE);
    Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_6, FALSE);
    Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_7, FALSE);
    Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_8, FALSE);

    //==================================
    FOREVER
    {
        MOTOROS2_MEM_TRACE_START(TOTAL_LOSS);
        MOTOROS2_MEM_TRACE_START(ONE_INIT);
        Ros_Controller_StatusInit();
        MOTOROS2_MEM_TRACE_REPORT(ONE_INIT);
        MOTOROS2_MEM_TRACE_START(TWO_INIT);
        Ros_Allocation_Initialize(&g_motoros2_Allocator);
        MOTOROS2_MEM_TRACE_REPORT(TWO_INIT);
        MOTOROS2_MEM_TRACE_START(TWO_POINT_FIVE_INIT);
        Ros_mpGetRobotCalibrationData_Initialize(); //must occur before Ros_Controller_Initialize
        MOTOROS2_MEM_TRACE_REPORT(TWO_POINT_FIVE_INIT);
        MOTOROS2_MEM_TRACE_START(THREE_INIT);
        Ros_Communication_ConnectToAgent();
        MOTOROS2_MEM_TRACE_REPORT(THREE_INIT);

        Ros_Controller_SetIOState(IO_FEEDBACK_AGENTCONNECTED, TRUE);
        MOTOROS2_MEM_TRACE_START(FOUR_INIT);
        Ros_Communication_Initialize();
        MOTOROS2_MEM_TRACE_REPORT(FOUR_INIT);
        MOTOROS2_MEM_TRACE_START(FIVE_INIT);
        // non-recoverable if this fails
        motoRosAssert(Ros_Controller_Initialize(), SUBCODE_FAIL_ROS_CONTROLLER_INIT);
        MOTOROS2_MEM_TRACE_REPORT(FIVE_INIT);
        MOTOROS2_MEM_TRACE_START(SIX_INIT);
        Ros_InformChecker_ValidateJob();
        MOTOROS2_MEM_TRACE_REPORT(SIX_INIT);
        MOTOROS2_MEM_TRACE_START(SEVEN_INIT);
        Ros_PositionMonitor_Initialize();
        MOTOROS2_MEM_TRACE_REPORT(SEVEN_INIT);
        MOTOROS2_MEM_TRACE_START(EIGHT_INIT);
        Ros_ActionServer_FJT_Initialize(); //initialize action server - FollowJointTrajectory
        MOTOROS2_MEM_TRACE_REPORT(EIGHT_INIT);
        MOTOROS2_MEM_TRACE_START(NINE_INIT);
        Ros_ServiceQueueTrajPoint_Initialize();
        MOTOROS2_MEM_TRACE_REPORT(NINE_INIT);
        MOTOROS2_MEM_TRACE_START(TEN_INIT);
        Ros_ServiceReadWriteIO_Initialize();
        MOTOROS2_MEM_TRACE_REPORT(TEN_INIT);
        MOTOROS2_MEM_TRACE_START(ELEVEN_INIT);
        Ros_ServiceResetError_Initialize();
        MOTOROS2_MEM_TRACE_REPORT(ELEVEN_INIT);
        MOTOROS2_MEM_TRACE_START(TWELVE_INIT);
        Ros_ServiceStartTrajMode_Initialize();
        MOTOROS2_MEM_TRACE_REPORT(TWELVE_INIT);
        MOTOROS2_MEM_TRACE_START(THIRTEEN_INIT);
        Ros_ServiceStartPointQueueMode_Initialize();
        MOTOROS2_MEM_TRACE_REPORT(THIRTEEN_INIT);
        MOTOROS2_MEM_TRACE_START(FOURTEEN_INIT);
        Ros_ServiceStopTrajMode_Initialize();
        MOTOROS2_MEM_TRACE_REPORT(FOURTEEN_INIT);
        MOTOROS2_MEM_TRACE_START(FIFTEEN_INIT);
        Ros_ServiceSelectMotionTool_Initialize();
        MOTOROS2_MEM_TRACE_REPORT(FIFTEEN_INIT);
        MOTOROS2_MEM_TRACE_START(SIXTEEN_INIT);

        // Start executor that performs all communication
        // (This task deletes itself when the agent disconnects.)
        SEM_ID semCommunicationExecutorStatus = mpSemBCreate(SEM_Q_FIFO, SEM_FULL);
        int tid = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE,
                                (FUNCPTR)Ros_Communication_StartExecutors,
                                (int)semCommunicationExecutorStatus, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        if (tid == ERROR)
            mpSetAlarm(ALARM_TASK_CREATE_FAIL, APPLICATION_NAME " FAILED TO CREATE TASK", SUBCODE_EXECUTOR);

        Ros_Debug_BroadcastMsg("Initialization complete. Memory available: (%d) bytes. Memory in use: (%d) bytes",
                       mpNumBytesFree(), MP_MEM_PART_SIZE - mpNumBytesFree());

        //==================================
        ULONG tickBefore = 0;

        while(g_Ros_Communication_AgentIsConnected)
        {
            //figure out how long to sleep to achieve the user-configured rate
            ULONG tickNow = tickGet();

            ULONG tickDiff = 0;
            if (tickNow > tickBefore)
                tickDiff = tickNow - tickBefore;
            else //unsigned rollover
                tickDiff = (ULONG_MAX - tickBefore) + tickNow;

            float elapsedMs = tickDiff * mpGetRtc(); //time it took to read and publish data

            if (elapsedMs < g_nodeConfigSettings.controller_status_monitor_period)
                Ros_Sleep(g_nodeConfigSettings.controller_status_monitor_period - elapsedMs);
            else
                Ros_Sleep(1); //don't hog the CPU from other tasks

            tickBefore = tickGet();

            //Check controller status.
            //This is being done on an independent thread (as opposed to being refreshed on-demand)
            //so that the motion thread can react as needed.
            if (!Ros_Controller_IoStatusUpdate())
            {
                Ros_Debug_BroadcastMsg("main: IoStatusUpdate failed, forcing disconnect/shutdown");
                // force a 'disconnection', so tasks can start shutting down
                g_Ros_Communication_AgentIsConnected = FALSE;
                // now assert
                motoRosAssert(FALSE, SUBCODE_FAIL_IO_STATUS_UPDATE);
            }

            //Update robot's feedback position and publish the topics
            Ros_PositionMonitor_UpdateLocation();
        }
        MOTOROS2_MEM_TRACE_REPORT(SIXTEEN_INIT);
        MOTOROS2_MEM_TRACE_START(SEVENTEEN_INIT);
        //==================================
        Ros_Controller_SetIOState(IO_FEEDBACK_AGENTCONNECTED, FALSE);
        Ros_Debug_BroadcastMsg("Micro-ROS PC Agent disconnected");
        //Also print to console, for easier debugging (but only if not logging to stdout already)
        if (!g_nodeConfigSettings.log_to_stdout)
        {
            Ros_Debug_LogToConsole("Micro-ROS PC Agent disconnected");
        }

        Ros_Debug_BroadcastMsg("Waiting for motion to stop before releasing memory");
        do
        {
            Ros_Sleep(1000);
        } while (Ros_Controller_IsInMotion()); //wait for motion to complete before terminating tasks
        MOTOROS2_MEM_TRACE_REPORT(SEVENTEEN_INIT);
        MOTOROS2_MEM_TRACE_START(EIGHTEEN_INIT);
        //wait for Ros_Communication_StartExecutors to finish
        mpSemTake(semCommunicationExecutorStatus, WAIT_FOREVER);
        MOTOROS2_MEM_TRACE_REPORT(EIGHTEEN_INIT);
        MOTOROS2_MEM_TRACE_START(NINETEEN_INIT);
        mpSemDelete(semCommunicationExecutorStatus);
        MOTOROS2_MEM_TRACE_REPORT(NINETEEN_INIT);
        MOTOROS2_MEM_TRACE_START(FIFTEEN_FINI);
        Ros_ServiceSelectMotionTool_Cleanup();
        MOTOROS2_MEM_TRACE_REPORT(FIFTEEN_FINI);
        MOTOROS2_MEM_TRACE_START(FOURTEEN_FINI);
        Ros_ServiceStopTrajMode_Cleanup();
        MOTOROS2_MEM_TRACE_REPORT(FOURTEEN_FINI);
        MOTOROS2_MEM_TRACE_START(THIRTEEN_FINI);
        Ros_ServiceStartPointQueueMode_Cleanup();
        MOTOROS2_MEM_TRACE_REPORT(THIRTEEN_FINI);
        MOTOROS2_MEM_TRACE_START(TWELVE_FINI);
        Ros_ServiceStartTrajMode_Cleanup();
        MOTOROS2_MEM_TRACE_REPORT(TWELVE_FINI);
        MOTOROS2_MEM_TRACE_START(ELEVEN_FINI);
        Ros_ServiceResetError_Cleanup();
        MOTOROS2_MEM_TRACE_REPORT(ELEVEN_FINI);
        MOTOROS2_MEM_TRACE_START(TEN_FINI);
        Ros_ServiceReadWriteIO_Cleanup();
        MOTOROS2_MEM_TRACE_REPORT(TEN_FINI);
        MOTOROS2_MEM_TRACE_START(NINE_FINI);
        Ros_ServiceQueueTrajPoint_Cleanup();
        MOTOROS2_MEM_TRACE_REPORT(NINE_FINI);

        MOTOROS2_MEM_TRACE_START(EIGHT_FINI);
        Ros_ActionServer_FJT_Cleanup();
        MOTOROS2_MEM_TRACE_REPORT(EIGHT_FINI);
        MOTOROS2_MEM_TRACE_START(SEVEN_FINI);
        Ros_PositionMonitor_Cleanup();
        MOTOROS2_MEM_TRACE_REPORT(SEVEN_FINI);
        MOTOROS2_MEM_TRACE_START(FIVE_FINI);
        Ros_Controller_Cleanup();
        MOTOROS2_MEM_TRACE_REPORT(FIVE_FINI);
        MOTOROS2_MEM_TRACE_START(FOUR_FINI);
        Ros_Communication_Cleanup(); 
        MOTOROS2_MEM_TRACE_REPORT(FOUR_FINI);
        MOTOROS2_MEM_TRACE_START(TWO_POINT_FIVE_FINI);
        Ros_mpGetRobotCalibrationData_Cleanup();
        MOTOROS2_MEM_TRACE_REPORT(TWO_POINT_FIVE_FINI);
        MOTOROS2_MEM_TRACE_START(ONE_FINI);
        //--------------------------------
        Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

        Ros_Sleep(2500);

        Ros_Debug_BroadcastMsg("Shutdown complete. Available memory: (%d) bytes", mpNumBytesFree());
        MOTOROS2_MEM_TRACE_REPORT(ONE_FINI);
        MOTOROS2_MEM_TRACE_REPORT(TOTAL_LOSS);
    }
}


