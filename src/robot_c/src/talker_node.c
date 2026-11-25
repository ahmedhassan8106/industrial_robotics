#include <stdio.h>
#include <stdint.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

static rcl_publisher_t publisher;
static std_msgs__msg__String message;
static int counter = 0;

// Called every 0.5 seconds
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;

    if (timer == NULL) {
        return;
    }

    char text[100];

    snprintf(text, sizeof(text), "Hello World %d", counter++);

    if (!rosidl_runtime_c__String__assign(&message.data, text)) {
        fprintf(stderr, "Failed to assign message string\n");
        return;
    }

    if (rcl_publish(&publisher, &message, NULL) == RCL_RET_OK) {
        printf("Publishing: \"%s\"\n", message.data.data);
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

    // Create the publisher node
    rcl_node_t node = rcl_get_zero_initialized_node();

    if (rclc_node_init_default(
            &node,
            "Publisher_Node",
            "",
            &support) != RCL_RET_OK) {
        fprintf(stderr, "Failed to create publisher node\n");
        return 1;
    }

    // Create publisher for Greeting_topic
    publisher = rcl_get_zero_initialized_publisher();

    if (rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "Greeting_topic") != RCL_RET_OK) {
        fprintf(stderr, "Failed to create publisher\n");
        return 1;
    }

    // Initialize the message
    if (!std_msgs__msg__String__init(&message)) {
        fprintf(stderr, "Failed to initialize message\n");
        return 1;
    }

    // Create a timer running every 500 milliseconds
    rcl_timer_t timer = rcl_get_zero_initialized_timer();

    if (rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(500),
            timer_callback) != RCL_RET_OK) {
        fprintf(stderr, "Failed to create timer\n");
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

    if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) {
        fprintf(stderr, "Failed to add timer\n");
        return 1;
    }

    printf("Publisher_Node started\n");

    // Keep the node running
    rclc_executor_spin(&executor);

    // Cleanup
    rclc_executor_fini(&executor);
    rcl_timer_fini(&timer);
    rcl_publisher_fini(&publisher, &node);
    std_msgs__msg__String__fini(&message);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
