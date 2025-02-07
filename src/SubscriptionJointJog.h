//SubscriptionJointJog.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SUBSCRIPTION_JOINT_JOG_H
#define MOTOROS2_SUBSCRIPTION_JOINT_JOG_H

extern rcl_subscription_t g_subscriptions_JointJog;
extern control_msgs__msg__JointJog g_messages_JointJog;

extern void Ros_SubscriptionJointJog_Initialize();
extern void Ros_SubscriptionJointJog_Cleanup();
extern void Ros_SubscriptionJointJog_Callback(const void* message);

#endif //MOTOROS2_SUBSCRIPTION_JOINT_JOG_H
