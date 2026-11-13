//ServiceStartRtMode.c

// SPDX-FileCopyrightText: 2025, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2025, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceStartRtMode;

ServiceStartRtMode_Messages g_messages_StartRtMode;

// shorten the typename a little, locally
typedef motoros2_interfaces__srv__StartRtMode_Request StartRtMode_Request;
typedef motoros2_interfaces__srv__StartRtMode_Response StartRtMode_Response;

void Ros_ServiceStartRtMode_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_start_rt_mode_init);

    rcl_ret_t ret = rclc_service_init_default(&g_serviceStartRtMode, &g_microRosNodeInfo.node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, StartRtMode),
        SERVICE_NAME_START_RT_MODE);
    motoRos_RCLAssertOK_withMsg(ret, SUBCODE_FAIL_INIT_SERVICE_START_RT_MODE, "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_StartRtMode.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_start_rt_mode_init);
}

void Ros_ServiceStartRtMode_Cleanup()
{
    MOTOROS2_MEM_TRACE_START(svc_start_rt_mode_fini);

    rcl_ret_t ret;

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_START_RT_MODE);
    ret = rcl_service_fini(&g_serviceStartRtMode, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_START_RT_MODE " service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_StartRtMode.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_start_rt_mode_fini);
}

//Cartesian increments are only supported for robots (no positioners or base tracks) with at
//least six axes. On robots with fewer (ex: SCARA, palletizers, MPL80/100), the controller
//doesn't turn the tool by the rotation it is sent, as reported by the controller (on
//palletizers, Rz turns it the opposite way, even when jogging on the pendant; on a SCARA, Rz
//does nothing). So the absolute mode would correct the orientation the wrong way, faster
//each cycle, or never.
//This is checked here, rather than in the R/T task, so the client is told why the
//session can't start.
static BOOL Ros_ServiceStartRtMode_IsCartesianSupported(rosidl_runtime_c__String* responseMessage)
{
    char output[MOTION_START_ERROR_MESSAGE_LENGTH] = { 0 };

    for (int groupIndex = 0; groupIndex < g_Ros_Controller.numGroup; groupIndex += 1)
    {
        if (!Ros_CtrlGroup_IsRobot(g_Ros_Controller.ctrlGroups[groupIndex]))
        {
            snprintf(output, MOTION_START_ERROR_MESSAGE_LENGTH,
                "%s: Cartesian control mode is only supported when all control groups are robots (group [%d] is not)",
                Ros_ErrorHandling_MotionNotReadyCode_ToString(MOTION_NOT_READY_INVALID_SELECTION), groupIndex);
            rosidl_runtime_c__String__assign(responseMessage, output);
            return FALSE;
        }

        if (g_Ros_Controller.ctrlGroups[groupIndex]->numAxes < 6)
        {
            snprintf(output, MOTION_START_ERROR_MESSAGE_LENGTH,
                "%s: Cartesian control mode is not supported for robots with fewer than six axes (group [%d] has %d)",
                Ros_ErrorHandling_MotionNotReadyCode_ToString(MOTION_NOT_READY_INVALID_SELECTION), groupIndex,
                g_Ros_Controller.ctrlGroups[groupIndex]->numAxes);
            rosidl_runtime_c__String__assign(responseMessage, output);
            return FALSE;
        }
    }

    return TRUE;
}

void Ros_ServiceStartRtMode_Trigger(const void* request_msg, void* response_msg)
{
    StartRtMode_Request* request = (StartRtMode_Request*)request_msg;
    StartRtMode_Response* response = (StartRtMode_Response*)response_msg;

    response->result_code.value = MOTION_READY;
    rosidl_runtime_c__String__assign(&response->message, "");
    response->period = g_Ros_Controller.interpolPeriod;
    response->timeout_for_rt_msg = g_nodeConfigSettings.timeout_for_rt_msg;
    response->max_sequence_diff_for_rt_msg = g_nodeConfigSettings.max_sequence_diff_for_rt_msg;

    MOTION_MODE mm = MOTION_MODE_INACTIVE;
    RtSessionSettings settings;

    settings.jointOrder = request->joint_order.value;
    settings.bAbsolute = FALSE;

    switch (request->control_mode.value)
    {
    case motoros2_interfaces__msg__ControlModeEnum__JOINT_INCREMENTS:
        mm = MOTION_MODE_RT_JOINT;
        break;
    case motoros2_interfaces__msg__ControlModeEnum__CARTESIAN_INCREMENTS:
        mm = MOTION_MODE_RT_CARTESIAN;
        break;
    case motoros2_interfaces__msg__ControlModeEnum__JOINT_ABSOLUTE:
        mm = MOTION_MODE_RT_JOINT;
        settings.bAbsolute = TRUE;
        break;
    case motoros2_interfaces__msg__ControlModeEnum__CARTESIAN_ABSOLUTE:
        mm = MOTION_MODE_RT_CARTESIAN;
        settings.bAbsolute = TRUE;
        break;
    }

    if (request->joint_order.value != motoros2_interfaces__msg__JointOrderEnum__JOINT_STATES &&
        request->joint_order.value != motoros2_interfaces__msg__JointOrderEnum__MOTOMAN)
        mm = MOTION_MODE_INACTIVE; //invalid selection

    if (mm != MOTION_MODE_INACTIVE && !Ros_RtMotionControl_IsSocketOpen())
    {
        char output[MOTION_START_ERROR_MESSAGE_LENGTH];
        snprintf(output, MOTION_START_ERROR_MESSAGE_LENGTH, "%s: The R/T port (%d) could not be opened. See alarm 8013[%d] or 8011[%d].",
            Ros_ErrorHandling_MotionNotReadyCode_ToString(MOTION_NOT_READY_ERROR),
            g_nodeConfigSettings.rt_listener_udp_port_number, SUBCODE_CONFIGURATION_RT_PORT_IN_USE,
            SUBCODE_FAIL_ALLOCATE_RT_CMD_SOCKET);
        rosidl_runtime_c__String__assign(&response->message, output);
        response->result_code.value = MOTION_NOT_READY_ERROR;
    }
    else if (mm == MOTION_MODE_RT_CARTESIAN && !Ros_ServiceStartRtMode_IsCartesianSupported(&response->message))
        response->result_code.value = MOTION_NOT_READY_INVALID_SELECTION;
    else if (mm != MOTION_MODE_INACTIVE)
        response->result_code.value = Ros_MotionControl_StartMotionMode(mm, &settings, &response->message);
    else
        response->result_code.value = MOTION_NOT_READY_INVALID_SELECTION;

    if (response->result_code.value != MOTION_READY)
    {
        // update response

        //Unless a more specific message was already written (ex: MOTION_NOT_READY_ERROR, see
        //Ros_MotionControl_StartMotionMode, or an unsupported Cartesian configuration)
        if (response->message.size == 0)
        {
            // map to human readable string
            rosidl_runtime_c__String__assign(&response->message,
                Ros_ErrorHandling_MotionNotReadyCode_ToString((MotionNotReadyCode)response->result_code.value));
        }

        Ros_Debug_BroadcastMsg("%s: %s (%d)", __func__,
            response->message.data, response->result_code.value);
    }
    else
    {
        Ros_Debug_BroadcastMsg("%s: activated", __func__);
    }
}
