#include <stdio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

// Called when a message arrives
void subscription_callback(const void *message)
{
    const std_msgs__msg__String *msg =
        (const std_msgs__msg__String *)message;

    if (msg != NULL && msg->data.data != NULL) {
        printf("Hearing: \"%s\"\n", msg->data.data);
        fflush(stdout);
    }
}

int main(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Initialize ROS 2
    rclc_support_t support;

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        fprintf(stderr, "Failed to initialize ROS 2\n");
        return 1;
    }

    // Create subscriber node
    rcl_node_t node = rcl_get_zero_initialized_node();

    if (rclc_node_init_default(
            &node,
            "Subscriber_Node",
            "",
            &support) != RCL_RET_OK) {
        fprintf(stderr, "Failed to create subscriber node\n");
        return 1;
    }

    // Create subscriber
    rcl_subscription_t subscriber =
        rcl_get_zero_initialized_subscription();

    if (rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "Greeting_topic") != RCL_RET_OK) {
        fprintf(stderr, "Failed to create subscriber\n");
        return 1;
    }

    // Initialize receiving message
    std_msgs__msg__String message;

    if (!std_msgs__msg__String__init(&message)) {
        fprintf(stderr, "Failed to initialize message\n");
        return 1;
    }

    // Create executor
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

    // Connect subscriber to callback
    if (rclc_executor_add_subscription(
            &executor,
            &subscriber,
            &message,
            subscription_callback,
            ON_NEW_DATA) != RCL_RET_OK) {
        fprintf(stderr, "Failed to add subscription\n");
        return 1;
    }

    printf("Subscriber_Node started\n");

    // Wait for messages
    rclc_executor_spin(&executor);

    // Cleanup
    rclc_executor_fini(&executor);
    rcl_subscription_fini(&subscriber, &node);
    std_msgs__msg__String__fini(&message);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
