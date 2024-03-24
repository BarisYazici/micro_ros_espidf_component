
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"


#include <uros_network_interfaces.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "example_interfaces/srv/add_two_ints.h"
#include "umi_robot_msgs/srv/control_servo.h"
#include "umi_robot_msgs/msg/joint_states.h"


#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char *TAG = "servo_drive";

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE -90         // Minimum angle
#define SERVO_MAX_DEGREE 90          // Maximum angle

#define SERVO_PULSE_GPIO_1 2  // GPIO connects to the PWM signal line
#define SERVO_PULSE_GPIO_2 4  // GPIO connects to the PWM signal line
#define SERVO_PULSE_GPIO_3 17 // GPIO connects to the PWM signal line
#define SERVO_PULSE_GPIO_4 18 // GPIO connects to the PWM signal line

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms
#define RMW_UXRCE_TRANSPORT_CUSTOM 1

#include "esp32_serial_transport.h"

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

#define DOMAIN_ID 100

mcpwm_cmpr_handle_t comparator[4]; // make the comparator a global variable so it can be accessed in the service callback
mcpwm_gen_handle_t generator[4];
mcpwm_timer_handle_t timer[2];
mcpwm_oper_handle_t oper[4]; // Create an array of 2 operator handles

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void servo_control_service_callback(const void *req, void *res)
{
    umi_robot_msgs__srv__ControlServo_Request *req_in = (umi_robot_msgs__srv__ControlServo_Request *)req;
    umi_robot_msgs__srv__ControlServo_Response *res_in = (umi_robot_msgs__srv__ControlServo_Response *)res;
    ESP_LOGI(TAG, "Entered the callback function:");

    float motor_positions[4] = {req_in->motor_1, req_in->motor_2, req_in->motor_3, req_in->motor_4};

    // Loop over the servo_positions array
    for (int i = 0; i < 4; i++) {
        int servo_position = motor_positions[i];

        ESP_LOGI(TAG, "Servo Position: %d", servo_position);
        uint32_t compare_value = example_angle_to_compare(servo_position);
        ESP_LOGI(TAG, "Compare value: %lu", compare_value);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], compare_value));
    }
    res_in->success = true;
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));

// #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
//     rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

//     // Static Agent IP and port can be used instead of autodisvery.
//     RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
//     // RCCHECK(rmw_uros_discover_agent(rmw_options));
// #endif

    // Setup support structure.
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "servo_drive_client_rclc", "", &support));

    // create service
    rcl_service_t service;
    RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(umi_robot_msgs, srv, ControlServo), "/set_servo_angle"));

    // // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));


    umi_robot_msgs__srv__ControlServo_Response res;
    umi_robot_msgs__srv__ControlServo_Request req;
    RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, servo_control_service_callback));
   
    // Spin forever
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

        usleep(1000);
    }

    // Free resources
    RCCHECK(rcl_service_fini(&service, &node));
    RCCHECK(rcl_node_fini(&node));
}

static size_t uart_port = UART_NUM_0;

void app_main(void)
{

// #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
//     ESP_ERROR_CHECK(uros_network_interface_initialize());
// #endif
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
	rmw_uros_set_custom_transport(
		true,
		(void *) &uart_port,
		esp32_serial_open,
		esp32_serial_close,
		esp32_serial_write,
		esp32_serial_read
	);
#else
#error micro-ROS transports misconfigured
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM
    mcpwm_timer_config_t timer_config = {
        .group_id = 0, // Use the same group for all timers
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_timer_config_t timer_config1 = {
        .group_id = 1, // Use the same group for all timers
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };

    mcpwm_operator_config_t operator_config1 = {
        .group_id = 1, // operator must be in the same group to the timer
    };
    for (int i = 0; i < 4; i++)
    {
        mcpwm_generator_config_t generator_config;
        // Use the same timer for servos 1 and 2, and a different timer for servos 3 and 4
        if (i < 2)
        {
            ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer[0]));
            ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper[i]));
        }
        else
        {
            ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config1, &timer[1]));
            ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config1, &oper[i]));
        }

        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper[i], timer[i / 2]));


        ESP_LOGI(TAG, "Create comparator and generator from the operator");

        mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper[i], &comparator_config, &comparator[i]));

        if (i == 0)
        {
            generator_config.gen_gpio_num = SERVO_PULSE_GPIO_1;
        }
        else if (i == 1)
        {
            generator_config.gen_gpio_num = SERVO_PULSE_GPIO_2;
        }
        else if (i == 2)
        {
            generator_config.gen_gpio_num = SERVO_PULSE_GPIO_3;
        }
        else
        {
            generator_config.gen_gpio_num = SERVO_PULSE_GPIO_4;
        }
        ESP_ERROR_CHECK(mcpwm_new_generator(oper[i], &generator_config, &generator[i]));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], example_angle_to_compare(0)));

        // go high on counter empty
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator[i], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

        // go low on compare threshold
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator[i],
                                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator[i], MCPWM_GEN_ACTION_LOW)));

        ESP_LOGI(TAG, "Enable and start timer");
        ESP_ERROR_CHECK(mcpwm_timer_enable(timer[i / 2]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer[i / 2], MCPWM_TIMER_START_NO_STOP));
    }

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
                "uros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}