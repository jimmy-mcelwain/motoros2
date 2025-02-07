//ServiceStartJointJogMode.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_services_StartJointJogMode;

ServiceStartJointJogMode_Messages g_messages_StartJointJogMode;

void Ros_ServiceStartJointJogMode_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_start_joint_jog_mode_init);

    const rosidl_service_type_support_t* type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);

    rcl_ret_t ret = rclc_service_init_default(&g_services_StartJointJogMode, &g_microRosNodeInfo.node, type_support, "start_joint_jog_mode");
    motoRosAssert_withMsg(ret == RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_STOP_TRAJ_MODE, "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_StartJointJogMode.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_start_joint_jog_mode_init);
}

void Ros_ServiceStartJointJogMode_Cleanup()
{
    MOTOROS2_MEM_TRACE_START(svc_start_joint_jog_mode_fini);

    rcl_ret_t ret;

    Ros_Debug_BroadcastMsg("Cleanup service start_joint_jog_mode");
    ret = rcl_service_fini(&g_services_StartJointJogMode, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up start_joint_jog_mode service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_StartJointJogMode.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_start_joint_jog_mode_fini);
}

void Ros_ServiceStartJointJogMode_Trigger(const void* request_msg, void* response_msg)
{
    Ros_Debug_BroadcastMsg("start_joint_jog_mode: attempting to start joint jog mode");

    std_srvs__srv__Trigger_Response* response = (std_srvs__srv__Trigger_Response*)response_msg;

    response->success = true;
    rosidl_runtime_c__String__assign(&response->message, "TODO");

    MotionNotReadyCode motion_result_code = Ros_MotionControl_StartMotionMode(MOTION_MODE_JOINTJOG, &response->message);
    if (motion_result_code != MOTION_READY)
    {
        // update response
        response->success = false;

        //If it is a MOTION_NOT_READY_ERROR, then the string was already populated in the Ros_MotionControl_StartMotionMode function
        if (motion_result_code != MOTION_NOT_READY_ERROR) {
            // map to human readable string
            rosidl_runtime_c__String__assign(&response->message,
                Ros_ErrorHandling_MotionNotReadyCode_ToString(motion_result_code));
        }

        Ros_Debug_BroadcastMsg("%s: %s (%d)", __func__,
            response->message.data, response->success);
    }
    else
    {
        Ros_Debug_BroadcastMsg("%s: activated", __func__);
    }
}
