//SubscriptionJointJog.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_subscription_t g_subscriptions_JointJog;

control_msgs__msg__JointJog g_messages_JointJog;


void Ros_SubscriptionJointJog_Initialize()
{
    MOTOROS2_MEM_TRACE_START(sub_joint_jog_init);

    Ros_Debug_BroadcastMsg("Initializing JointJog subscription");

    const rmw_qos_profile_t* qos_profile = Ros_ConfigFile_To_Rmw_Qos_Profile(ROS_QOS_PROFILE_SENSOR_DATA);

    rclc_subscription_init(
        &g_subscriptions_JointJog,
        &g_microRosNodeInfo.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
        "servo_node/delta_joint_cmds",
        qos_profile
        );

    control_msgs__msg__JointJog__init(&g_messages_JointJog);

    rosidl_runtime_c__String__Sequence__init(&g_messages_JointJog.joint_names, 6);
    rosidl_runtime_c__double__Sequence__init(&g_messages_JointJog.displacements, 6);
    rosidl_runtime_c__double__Sequence__init(&g_messages_JointJog.velocities, 6);

    for (int i = 0; i < 6; i += 1)
        rosidl_runtime_c__String__assign(&g_messages_JointJog.joint_names.data[i], "012345678901234567890123456789012");

    MOTOROS2_MEM_TRACE_REPORT(sub_joint_jog_init);
}

void Ros_SubscriptionJointJog_Cleanup()
{
    rcl_ret_t ret;
    MOTOROS2_MEM_TRACE_START(sub_joint_jog_fini);

    Ros_Debug_BroadcastMsg("Cleanup JointJog subscription");

    rosidl_runtime_c__double__Sequence__fini(&g_messages_JointJog.velocities);
    rosidl_runtime_c__double__Sequence__fini(&g_messages_JointJog.displacements);
    rosidl_runtime_c__String__Sequence__fini(&g_messages_JointJog.joint_names);

    control_msgs__msg__JointJog__fini(&g_messages_JointJog);

    ret = rcl_subscription_fini(&g_subscriptions_JointJog, &g_microRosNodeInfo.node);

    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up JointJog subscription: %d", ret);
    control_msgs__msg__JointJog__fini(&g_messages_JointJog);

    MOTOROS2_MEM_TRACE_REPORT(sub_joint_jog_fini);
}

void Ros_SubscriptionJointJog_Callback(const void* message)
{
    const control_msgs__msg__JointJog * msg = (control_msgs__msg__JointJog*) message;

    Ros_Debug_BroadcastMsg("Does debug broadcasting even work?");

}
