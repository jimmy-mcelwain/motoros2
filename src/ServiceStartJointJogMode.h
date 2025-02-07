//ServiceStartJointJogMode.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_START_JOINT_JOG_MODE_H
#define MOTOROS2_SERVICE_START_JOINT_JOG_MODE_H


extern rcl_service_t g_services_StartJointJogMode;

typedef struct
{
    std_srvs__srv__Trigger_Request request;
    std_srvs__srv__Trigger_Response response;
} ServiceStartJointJogMode_Messages;
extern ServiceStartJointJogMode_Messages g_messages_StartJointJogMode;

extern void Ros_ServiceStartJointJogMode_Initialize();
extern void Ros_ServiceStartJointJogMode_Cleanup();

extern void Ros_ServiceStartJointJogMode_Trigger(const void* request_msg, void* response_msg);


#endif  // MOTOROS2_SERVICE_START_JOINT_JOG_MODE_H
