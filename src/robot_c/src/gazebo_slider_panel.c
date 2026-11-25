#include <gtk/gtk.h>

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#define JOINT_COUNT 7
#define PENDING_TIMEOUT_US 500000

static const char *joint_names[JOINT_COUNT] = {
    "joint_1", "joint_2", "joint_3", "joint_4",
    "joint_5", "joint_6", "joint_7"
};

static const double lower_limits[JOINT_COUNT] = {
    -1.57079632679, -1.57079632679, -1.57079632679, -3.14159265359, 0.0, 0.0, -0.5236
};

static const double upper_limits[JOINT_COUNT] = {
    1.57079632679, 1.57079632679, 1.57079632679, 3.14159265359, 3.14159265359, 0.5236, 0.0
};

static const double demo_pose[JOINT_COUNT] = {
    0.31, -0.24, 0.39, 0.21,
     0.15, 0.15, -0.15
};

/* GTK widgets */
static GtkWidget *sliders[JOINT_COUNT];
static GtkWidget *measured_labels[JOINT_COUNT];

static GtkWidget *status_label;
static GtkWidget *demo_button;
static GtkWidget *zero_button;

/* Synchronization state */
static bool updating_from_ros = false;
static bool ready = false;
static bool seen_targets[JOINT_COUNT] = {false};

static double pending_values[JOINT_COUNT] = {0};
static gint64 pending_times[JOINT_COUNT] = {0};

static gint64 last_targets_time = 0;
static gint64 feedback_times[JOINT_COUNT] = {0};

/* ROS objects */
static rcl_node_t node;
static rcl_publisher_t target_publisher;

static rcl_subscription_t targets_subscriber;
static rcl_subscription_t feedback_subscriber;

static rclc_executor_t executor;

static sensor_msgs__msg__JointState targets_message;
static sensor_msgs__msg__JointState feedback_message;


/* Find joint index by name */
static int find_joint(const char *name)
{
    for (int i = 0; i < JOINT_COUNT; i++) {
        if (strcmp(name, joint_names[i]) == 0) {
            return i;
        }
    }

    return -1;
}


/* Publish one joint or all seven joints */
static void publish_targets(int selected_joint)
{
    int count = selected_joint < 0 ? JOINT_COUNT : 1;
    sensor_msgs__msg__JointState *msg =
    sensor_msgs__msg__JointState__create();

    if (msg == NULL) {
        fprintf(stderr, "Could not create target message\n");
        return;
    }

    bool names_ok =
        rosidl_runtime_c__String__Sequence__init(
            &msg->name, count
        );

    bool positions_ok =
        rosidl_runtime_c__double__Sequence__init(
            &msg->position, count
        );

    if (!names_ok || !positions_ok) {
        fprintf(stderr, "Could not allocate joint targets\n");
        sensor_msgs__msg__JointState__destroy(msg);
        return;
    }

    for (int k = 0; k < count; k++) {
        int i = selected_joint < 0 ? k : selected_joint;

        rosidl_runtime_c__String__assign(
            &msg->name.data[k],
            joint_names[i]
        );

        msg->position.data[k] =
            gtk_range_get_value(GTK_RANGE(sliders[i]));
    }

    rcl_ret_t result = rcl_publish(
        &target_publisher,
        msg,
        NULL
    );

    if (result == RCL_RET_OK) {
        gint64 now = g_get_monotonic_time();

        for (int k = 0; k < count; k++) {
            int i = selected_joint < 0 ? k : selected_joint;

            pending_values[i] = msg->position.data[k];
            pending_times[i] = now;
        }
    } else {
        fprintf(stderr, "Failed to publish slider targets\n");
    }

    sensor_msgs__msg__JointState__destroy(msg);
}


/* Called when a Gazebo slider moves */
static void slider_changed(GtkRange *range, gpointer user_data)
{
    (void)range;

    if (!ready || updating_from_ros) {
        return;
    }

    int joint_index = GPOINTER_TO_INT(user_data);

    publish_targets(joint_index);
}


/* Apply a pose to all seven joints */
static void apply_pose(const double pose[JOINT_COUNT])
{
    if (!ready) {
        return;
    }

    updating_from_ros = true;

    for (int i = 0; i < JOINT_COUNT; i++) {
        gtk_range_set_value(
            GTK_RANGE(sliders[i]),
            pose[i]
        );
    }

    updating_from_ros = false;

    publish_targets(-1);
}


/* Demo Pose button */
static void demo_clicked(GtkButton *button, gpointer user_data)
{
    (void)button;
    (void)user_data;

    apply_pose(demo_pose);
}


/* Return to Zero button */
static void zero_clicked(GtkButton *button, gpointer user_data)
{
    (void)button;
    (void)user_data;

    const double zero_pose[JOINT_COUNT] = {
        0, 0, 0, 0, 0, 0, 0
    };

    apply_pose(zero_pose);
}


/* Receive targets from original RViz slider GUI */
static void targets_callback(const void *received)
{
    const sensor_msgs__msg__JointState *msg = received;

    gint64 now = g_get_monotonic_time();

    updating_from_ros = true;

    size_t count = msg->name.size;

    if (msg->position.size < count) {
        count = msg->position.size;
    }

    for (size_t k = 0; k < count; k++) {
        int i = find_joint(msg->name.data[k].data);
        double value = msg->position.data[k];

        if (i < 0 || !isfinite(value)) {
            continue;
        }

        seen_targets[i] = true;
        last_targets_time = now;

        /* Ignore older GUI echoes after a local edit */
        if (pending_times[i] > 0) {
            gint64 age = now - pending_times[i];

            if (age < PENDING_TIMEOUT_US &&
                fabs(value - pending_values[i]) > 0.011) {
                continue;
            }

            pending_times[i] = 0;
        }

        gtk_range_set_value(
            GTK_RANGE(sliders[i]),
            value
        );
    }

    updating_from_ros = false;

    /* Enable controls after all seven targets are known */
    if (!ready) {
        bool all_seen = true;

        for (int i = 0; i < JOINT_COUNT; i++) {
            if (!seen_targets[i]) {
                all_seen = false;
            }
        }

        if (all_seen) {
            ready = true;

            for (int i = 0; i < JOINT_COUNT; i++) {
                gtk_widget_set_sensitive(sliders[i], TRUE);
            }

            gtk_widget_set_sensitive(demo_button, TRUE);
            gtk_widget_set_sensitive(zero_button, TRUE);
        }
    }
}


/* Receive actual measured positions from Gazebo */
static void feedback_callback(const void *received)
{
    const sensor_msgs__msg__JointState *msg = received;

    size_t count = msg->name.size;

    if (msg->position.size < count) {
        count = msg->position.size;
    }

    for (size_t k = 0; k < count; k++) {
        int i = find_joint(msg->name.data[k].data);
        double value = msg->position.data[k];

        if (i < 0 || !isfinite(value)) {
            continue;
        }

        char label[80];

        snprintf(
            label,
            sizeof(label),
            "measured: %+.3f rad",
            value
        );

        gtk_label_set_text(
            GTK_LABEL(measured_labels[i]),
            label
        );

        feedback_times[i] = g_get_monotonic_time();
    }
}


/* Process ROS messages inside GTK's event loop */
static gboolean update_ros(gpointer user_data)
{
    (void)user_data;

    rclc_executor_spin_some(
        &executor,
        RCL_MS_TO_NS(0)
    );

    gint64 now = g_get_monotonic_time();

    if (!ready) {
        gtk_label_set_text(
            GTK_LABEL(status_label),
            "Waiting for original RViz slider GUI..."
        );

        return G_SOURCE_CONTINUE;
    }

    if (now - last_targets_time > 1000000) {
        gtk_label_set_text(
            GTK_LABEL(status_label),
            "Waiting for fresh slider targets..."
        );

        return G_SOURCE_CONTINUE;
    }

    bool fresh_feedback = true;

    for (int i = 0; i < JOINT_COUNT; i++) {
        if (now - feedback_times[i] > 1000000) {
            fresh_feedback = false;
        }
    }

    if (fresh_feedback) {
        gtk_label_set_text(
            GTK_LABEL(status_label),
            "Sliders synchronized | Gazebo feedback active"
        );
    } else {
        gtk_label_set_text(
            GTK_LABEL(status_label),
            "Sliders synchronized | waiting for Gazebo feedback"
        );
    }

    return G_SOURCE_CONTINUE;
}


/* Construct the C slider window */
static void create_gui(void)
{
    GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    gtk_window_set_title(
        GTK_WINDOW(window),
        "Gazebo joint controls | C"
    );

    gtk_window_set_default_size(
        GTK_WINDOW(window),
        650,
        520
    );

    g_signal_connect(
        window,
        "destroy",
        G_CALLBACK(gtk_main_quit),
        NULL
    );

    GtkWidget *main_box = gtk_box_new(
        GTK_ORIENTATION_VERTICAL,
        8
    );

    gtk_container_set_border_width(
        GTK_CONTAINER(main_box),
        12
    );

    gtk_container_add(
        GTK_CONTAINER(window),
        main_box
    );

    GtkWidget *heading = gtk_label_new(
        "Gazebo controls — synchronized with RViz sliders"
    );

    gtk_box_pack_start(
        GTK_BOX(main_box),
        heading,
        FALSE,
        FALSE,
        0
    );

    /* Create seven rows */
    for (int i = 0; i < JOINT_COUNT; i++) {
        GtkWidget *row = gtk_box_new(
            GTK_ORIENTATION_HORIZONTAL,
            8
        );

        gtk_box_pack_start(
            GTK_BOX(main_box),
            row,
            FALSE,
            FALSE,
            0
        );

        GtkWidget *name = gtk_label_new(joint_names[i]);

        gtk_widget_set_size_request(name, 65, -1);

        gtk_box_pack_start(
            GTK_BOX(row),
            name,
            FALSE,
            FALSE,
            0
        );

        sliders[i] = gtk_scale_new_with_range(
            GTK_ORIENTATION_HORIZONTAL,
            lower_limits[i],
            upper_limits[i],
            0.01
        );

        gtk_scale_set_digits(GTK_SCALE(sliders[i]), 3);

        gtk_widget_set_size_request(
            sliders[i],
            320,
            -1
        );

        gtk_range_set_value(
            GTK_RANGE(sliders[i]),
            0.0
        );

        gtk_widget_set_sensitive(sliders[i], FALSE);

        gtk_box_pack_start(
            GTK_BOX(row),
            sliders[i],
            TRUE,
            TRUE,
            0
        );

        g_signal_connect(
            sliders[i],
            "value-changed",
            G_CALLBACK(slider_changed),
            GINT_TO_POINTER(i)
        );

        measured_labels[i] = gtk_label_new(
            "measured: --"
        );

        gtk_widget_set_size_request(
            measured_labels[i],
            160,
            -1
        );

        gtk_box_pack_start(
            GTK_BOX(row),
            measured_labels[i],
            FALSE,
            FALSE,
            0
        );
    }

    /* Demo Pose button */
    demo_button = gtk_button_new_with_label(
        "Move all seven joints to demo pose"
    );

    gtk_widget_set_sensitive(demo_button, FALSE);

    g_signal_connect(
        demo_button,
        "clicked",
        G_CALLBACK(demo_clicked),
        NULL
    );

    gtk_box_pack_start(
        GTK_BOX(main_box),
        demo_button,
        FALSE,
        FALSE,
        0
    );

    /* Return to Zero button */
    zero_button = gtk_button_new_with_label(
        "Return all targets to zero"
    );

    gtk_widget_set_sensitive(zero_button, FALSE);

    g_signal_connect(
        zero_button,
        "clicked",
        G_CALLBACK(zero_clicked),
        NULL
    );

    gtk_box_pack_start(
        GTK_BOX(main_box),
        zero_button,
        FALSE,
        FALSE,
        0
    );

    /* Status */
    status_label = gtk_label_new(
        "Waiting for ROS 2..."
    );

    gtk_box_pack_start(
        GTK_BOX(main_box),
        status_label,
        FALSE,
        FALSE,
        0
    );

    gtk_widget_show_all(window);
}


int main(int argc, char **argv)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rclc_support_t support;

    if (rclc_support_init(
            &support,
            argc,
            (const char * const *)argv,
            &allocator
        ) != RCL_RET_OK) {
        fprintf(stderr, "Failed to initialize ROS\n");
        return 1;
    }

    if (rclc_node_init_default(
            &node,
            "gazebo_slider_panel_c",
            "",
            &support
        ) != RCL_RET_OK) {
        fprintf(stderr, "Failed to initialize node\n");
        return 1;
    }

    if (rclc_publisher_init_default(
            &target_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                sensor_msgs, msg, JointState
            ),
            "/gazebo_slider_targets"
        ) != RCL_RET_OK) {
        fprintf(stderr, "Failed to create publisher\n");
        return 1;
    }

    if (rclc_subscription_init_default(
            &targets_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                sensor_msgs, msg, JointState
            ),
            "/slider_joint_targets"
        ) != RCL_RET_OK) {
        fprintf(stderr, "Failed to subscribe to targets\n");
        return 1;
    }

    if (rclc_subscription_init_default(
            &feedback_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                sensor_msgs, msg, JointState
            ),
            "/joint_states"
        ) != RCL_RET_OK) {
        fprintf(stderr, "Failed to subscribe to feedback\n");
        return 1;
    }

    sensor_msgs__msg__JointState__init(&targets_message);
    sensor_msgs__msg__JointState__init(&feedback_message);

    if (rclc_executor_init(
            &executor,
            &support.context,
            2,
            &allocator
        ) != RCL_RET_OK) {
        fprintf(stderr, "Failed to create executor\n");
        return 1;
    }

    rclc_executor_add_subscription(
        &executor,
        &targets_subscriber,
        &targets_message,
        targets_callback,
        ON_NEW_DATA
    );

    rclc_executor_add_subscription(
        &executor,
        &feedback_subscriber,
        &feedback_message,
        feedback_callback,
        ON_NEW_DATA
    );

    /* Start GTK without passing ROS-specific arguments */
    gtk_init(NULL, NULL);

    create_gui();

    /* Service ROS callbacks every 30 milliseconds */
    g_timeout_add(30, update_ros, NULL);

    printf("Gazebo C slider panel started.\n");
    printf("Demo Pose and Return to Zero are available.\n");
    fflush(stdout);

    gtk_main();

    /* Cleanup */
    rclc_executor_fini(&executor);

    rcl_subscription_fini(&targets_subscriber, &node);
    rcl_subscription_fini(&feedback_subscriber, &node);

    rcl_publisher_fini(&target_publisher, &node);

    sensor_msgs__msg__JointState__fini(&targets_message);
    sensor_msgs__msg__JointState__fini(&feedback_message);

    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
