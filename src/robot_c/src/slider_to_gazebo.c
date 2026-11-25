#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "rcl/rcl.h"
#include "rcl/time.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"

#include "sensor_msgs/msg/joint_state.h"
#include "std_msgs/msg/float64.h"
#include "trajectory_msgs/msg/joint_trajectory.h"
#include "trajectory_msgs/msg/joint_trajectory_point.h"

#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#define JOINT_COUNT 7
#define GROUP_COUNT 2

static volatile sig_atomic_t running = 1;

/* Existing Gazebo publishers */
static rcl_publisher_t gazebo_publishers[JOINT_COUNT];

/* New ROS 2 Control publishers */
static rcl_publisher_t trajectory_publishers[GROUP_COUNT];

static const char *joint_names[JOINT_COUNT] = {
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7"
};

static const char *gazebo_topics[JOINT_COUNT] = {
    "/joint_1/cmd_pos",
    "/joint_2/cmd_pos",
    "/joint_3/cmd_pos",
    "/joint_4/cmd_pos",
    "/joint_5/cmd_pos",
    "/joint_6/cmd_pos",
    "/joint_7/cmd_pos"
};

static const char *trajectory_topics[GROUP_COUNT] = {
    "/arm_group_controller/joint_trajectory",
    "/hand_group_controller/joint_trajectory"
};

static const double lower_limits[JOINT_COUNT] = {
    -3.1416,
    -1.5708,
    -2.0944,
    -3.1416,
    -1.5708,
     0.0,
    -0.5236
};

static const double upper_limits[JOINT_COUNT] = {
     3.1416,
     1.5708,
     2.0944,
     3.1416,
     1.5708,
     0.5236,
     0.0
};

/* Latest requested position for every joint */
static double requested_positions[JOINT_COUNT] = {0};

static void signal_handler(int signal_number)
{
    (void)signal_number;
    running = 0;
}

/*
 * Publish a complete joint trajectory for one controller.
 *
 * group = 0: joints 1-5
 * group = 1: joints 6-7
 */
static void publish_group_trajectory(
    int group,
    size_t first_joint,
    size_t joint_count)
{
    trajectory_msgs__msg__JointTrajectory trajectory = {0};

    if (!trajectory_msgs__msg__JointTrajectory__init(
            &trajectory)) {
        fprintf(stderr, "Failed to initialize trajectory\n");
        return;
    }

    bool ok = rosidl_runtime_c__String__Sequence__init(
        &trajectory.joint_names,
        joint_count
    );

    if (ok) {
        ok = trajectory_msgs__msg__JointTrajectoryPoint__Sequence__init(
            &trajectory.points,
            1
        );
    }

    if (ok) {
        ok = rosidl_runtime_c__double__Sequence__init(
            &trajectory.points.data[0].positions,
            joint_count
        );
    }

    if (ok) {
        for (size_t i = 0; i < joint_count; i++) {
            size_t joint_index = first_joint + i;

            if (!rosidl_runtime_c__String__assign(
                    &trajectory.joint_names.data[i],
                    joint_names[joint_index])) {
                ok = false;
                break;
            }

            trajectory.points.data[0].positions.data[i] =
                requested_positions[joint_index];
        }
    }

    if (ok) {
    /* Start trajectory immediately */
    trajectory.header.stamp.sec = 0;
    trajectory.header.stamp.nanosec = 0;

    /* Target should be reached after 1 second */
    trajectory.points.data[0].time_from_start.sec = 1;
    trajectory.points.data[0].time_from_start.nanosec = 0;

    rcl_ret_t result = rcl_publish(
        &trajectory_publishers[group],
        &trajectory,
        NULL
    );

        if (result != RCL_RET_OK) {
            fprintf(
                stderr,
                "Failed to publish trajectory for group %d\n",
                group
            );
        }
    } else {
        fprintf(stderr, "Failed to allocate trajectory\n");
    }

    trajectory_msgs__msg__JointTrajectory__fini(&trajectory);
}

/*
 * Receive targets from the synchronized slider system.
 */
static void joint_target_callback(const void *message)
{
    if (message == NULL) {
        return;
    }

    const sensor_msgs__msg__JointState *targets =
        (const sensor_msgs__msg__JointState *)message;

    if (targets->name.size != targets->position.size) {
        return;
    }

    bool valid_update = false;

    for (size_t i = 0; i < targets->name.size; i++) {
        const char *name = targets->name.data[i].data;
        double position = targets->position.data[i];

        if (name == NULL || !isfinite(position)) {
            continue;
        }

        for (size_t j = 0; j < JOINT_COUNT; j++) {
            if (strcmp(name, joint_names[j]) == 0) {

                if (position < lower_limits[j]) {
                    position = lower_limits[j];
                }

                if (position > upper_limits[j]) {
                    position = upper_limits[j];
                }

                requested_positions[j] = position;
                valid_update = true;

                /*
                 * Preserve existing Gazebo control.
                 * These publishers will remain available
                 * during migration to ROS 2 Control.
                 */
                std_msgs__msg__Float64 command = {0};
                command.data = position;

                (void)rcl_publish(
                    &gazebo_publishers[j],
                    &command,
                    NULL
                );

                break;
            }
        }
    }

    if (!valid_update) {
        return;
    }

    /* ROS 2 Control: arm group */
    publish_group_trajectory(0, 0, 5);

    /* ROS 2 Control: gripper group */
    publish_group_trajectory(1, 5, 2);
}

int main(int argc, const char *argv[])
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    rcl_allocator_t allocator =
        rcl_get_default_allocator();

    rclc_support_t support;

    if (rclc_support_init(
            &support,
            argc,
            argv,
            &allocator) != RCL_RET_OK) {
        fprintf(stderr, "Failed to initialize ROS 2\n");
        return 1;
    }

    rcl_node_t node = rcl_get_zero_initialized_node();

    if (rclc_node_init_default(
            &node,
            "slider_to_gazebo_c",
            "",
            &support) != RCL_RET_OK) {
        fprintf(stderr, "Failed to create node\n");
        return 1;
    }

    /* Initialize seven existing Gazebo publishers */
    for (size_t i = 0; i < JOINT_COUNT; i++) {
        gazebo_publishers[i] =
            rcl_get_zero_initialized_publisher();

        if (rclc_publisher_init_default(
                &gazebo_publishers[i],
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(
                    std_msgs, msg, Float64),
                gazebo_topics[i]) != RCL_RET_OK) {
            fprintf(stderr, "Failed to create Gazebo publisher\n");
            return 1;
        }
    }

    /* Initialize the two new trajectory publishers */
    for (size_t i = 0; i < GROUP_COUNT; i++) {
        trajectory_publishers[i] =
            rcl_get_zero_initialized_publisher();

        if (rclc_publisher_init_default(
                &trajectory_publishers[i],
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(
                    trajectory_msgs, msg, JointTrajectory),
                trajectory_topics[i]) != RCL_RET_OK) {
            fprintf(stderr, "Failed to create trajectory publisher\n");
            return 1;
        }
    }

    rcl_subscription_t subscription =
        rcl_get_zero_initialized_subscription();

    if (rclc_subscription_init_default(
            &subscription,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                sensor_msgs, msg, JointState),
            "/slider_joint_targets") != RCL_RET_OK) {
        fprintf(stderr, "Failed to create target subscription\n");
        return 1;
    }

    sensor_msgs__msg__JointState input_message;

    if (!sensor_msgs__msg__JointState__init(
            &input_message)) {
        fprintf(stderr, "Failed to initialize input message\n");
        return 1;
    }

    rclc_executor_t executor =
        rclc_executor_get_zero_initialized_executor();

    if (rclc_executor_init(
            &executor,
            &support.context,
            1,
            &allocator) != RCL_RET_OK) {
        fprintf(stderr, "Failed to initialize executor\n");
        return 1;
    }

    if (rclc_executor_add_subscription(
            &executor,
            &subscription,
            &input_message,
            joint_target_callback,
            ON_NEW_DATA) != RCL_RET_OK) {
        fprintf(stderr, "Failed to add subscription\n");
        return 1;
    }

    printf("C joint controller started.\n");
    printf("Listening on /slider_joint_targets\n");
    printf("Legacy Gazebo commands enabled.\n");
    printf("ROS 2 trajectory commands enabled.\n");
    fflush(stdout);

    while (running &&
           rcl_context_is_valid(&support.context)) {
        rclc_executor_spin_some(
            &executor,
            RCL_MS_TO_NS(100)
        );
    }

    rclc_executor_fini(&executor);

    rcl_subscription_fini(&subscription, &node);

    for (size_t i = 0; i < JOINT_COUNT; i++) {
        rcl_publisher_fini(
            &gazebo_publishers[i],
            &node
        );
    }

    for (size_t i = 0; i < GROUP_COUNT; i++) {
        rcl_publisher_fini(
            &trajectory_publishers[i],
            &node
        );
    }

    sensor_msgs__msg__JointState__fini(&input_message);

    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
