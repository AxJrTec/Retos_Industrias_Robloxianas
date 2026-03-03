// Micro-ROS and ESP32 libraries
#include <micro_ros_arduino.h>           // Micro-ROS library for ESP32
#include <rcl/rcl.h>                     // ROS 2 client library (Core)
#include <rclc/rclc.h>                   // Micro-ROS client library
#include <rclc/executor.h>               // Executor to handle callbacks
#include <std_msgs/msg/float32.h>        // Float message type for LED brightness
#include <stdio.h>                       // Standard I/O for debugging

// Micro-ROS Entity Declarations 
rclc_support_t support;       // Micro-ROS execution context
rclc_executor_t executor;     // Manages execution of tasks (timers, subscribers)
rcl_allocator_t allocator;    // Handles memory allocation

rcl_node_t node;              // Defines the ROS 2 node

rcl_subscription_t cmd_subscriber;

std_msgs__msg__Float32 cmd_msg;

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}} //Executes fn and returns false if it fails.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} // Executes fn, but ignores failures.

// Hardware Pin Definitions
#define IN1_PIN 25
#define IN2_PIN 26
#define PWM_PIN 27

#define PWM_FRQ 980 // Hz
#define PWM_RES 8 // Bits 0-255
#define PWM_CHNL 0

#define CMD_MIN -1 
#define CMD_MAX 1

// Executes a statement (X) every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// Micro-ROS Connection State Machine
enum states {
  WAITING_AGENT,        // Waiting for ROS 2 agent connection
  AGENT_AVAILABLE,      // Agent detected
  AGENT_CONNECTED,      // Successfully connected
  AGENT_DISCONNECTED    // Connection lost
} state;

// Function Prototypes
bool create_entities();
void destroy_entities();

// Motor Control
void set_motor(float cmd)
{
  // Limitar rango
  if (cmd > CMD_MAX) cmd = CMD_MAX;
  if (cmd < CMD_MIN) cmd = CMD_MIN;

  // Zona muerta
  if (fabs(cmd) < 0.05) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(PWM_CHNL, 0);
    return;
  }

  // Dirección
  if (cmd > 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }

  // Convertir cmd (0–1) a PWM (0–255)
  int duty = (int)(fabs(cmd) * 255.0);
  ledcWrite(PWM_CHNL, duty);
}

// Subscriber Callback 
void subscription_callback(const void * msgin) {  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  set_motor(msg->data);
}

// Setup Function
void setup() {
  set_microros_transports();  // Initialize Micro-ROS communication
  
  // Inicializar pines
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  // Configurar PWM ESP32
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);
}

// ======== Main Loop Function ========
void loop() {
  switch (state) {

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
      
    default:
      break;
  }
}

// ROS 2 Entity Creation and Cleanup Functions
bool create_entities()
{
  // Initialize Micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_elian", "", &support));

  // Initialize Subscriber
  RCCHECK(rclc_subscription_init_default(
      &cmd_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "cmd_pwm_axel"));

  // Initialize Executor
  // create zero initialised executor (no configured) to avoid memory problems
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_subscriber, &cmd_msg, &subscription_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&cmd_subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}