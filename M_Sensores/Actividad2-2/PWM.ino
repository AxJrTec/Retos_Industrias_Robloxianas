// Librerias
#include <micro_ros_arduino.h>           // Micro-ROS library for ESP32
#include <rcl/rcl.h>                     // ROS 2 client library (Core)
#include <rclc/rclc.h>                   // Micro-ROS client library
#include <rclc/executor.h>               // Executor to handle callbacks
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <string.h>

// Hardware Pin Definitions
#define VAR_PIN 14
#define IN1_PIN 25
#define IN2_PIN 26
#define PWM_PIN 27

#define PWM_FRQ 5000 // Hz
#define PWM_RES 8 // Bits 0-255
#define PWM_CHNL 0

// Micro-ROS Entity Declarations 
rclc_support_t support;       // Micro-ROS execution context
rclc_executor_t executor;     // Manages execution of tasks (timers, subscribers)
rcl_allocator_t allocator;    // Handles memory allocation

rcl_node_t node;              // Defines the ROS 2 node

rcl_subscription_t command_sub;

rcl_publisher_t duty_pub;
rcl_publisher_t voltaje_pub;
rcl_publisher_t state_pub;

std_msgs__msg__String command_msg;
std_msgs__msg__Float32 duty_msg;
std_msgs__msg__Float32 voltaje_msg;
std_msgs__msg__String state_msg;

// Variables del motor
bool motorEnMovimiento = false;
int pot = 0;
int pwm = 0;
float voltaje = 0;
float duty = 0;
float Vcc = 3.3;

// Buffer para strings
char state_buffer[20];
char command_buffer[20];

void subscription_callback(const void *msgin)
{
    std_msgs__msg__String *msg = (std_msgs__msg__String *)msgin;
    char opcion = msg->data.data[0];

    switch (opcion) {
        case 'D':
            motorEnMovimiento = true;
            derecha();
            break;
        case 'I':
            motorEnMovimiento = true;
            izquierda();
            break;
        case 'S':
            motorEnMovimiento = false;
            detener();
            break;
        default:
            break; // Comando no válido
    }
}

void leerPWM()
{
  pot = analogRead(VAR_PIN);

  voltaje = pot * (3.3 / 4095.0);
  duty = (voltaje / 3.3) * 100.0;

  pwm = map(pot, 0, 4095, 0, 255);
  ledcWrite(PWM_CHNL, pwm);
}

void detener()
{
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  ledcWrite(PWM_CHNL, 0);
  strcpy(state_buffer, "Detenido");
}

void derecha()
{
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  strcpy(state_buffer, "Derecha");
}

void izquierda()
{
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  strcpy(state_buffer, "Izquierda");
}

void setup()
{
  set_microros_transports();

  pinMode(VAR_PIN, INPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);

  detener();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_Industrias_Robloxianas", "", &support);

  // Publishers
  rclc_publisher_init_default(
    &duty_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/duty_cycle");

  rclc_publisher_init_default(
    &voltaje_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/voltaje");

  rclc_publisher_init_default(
    &state_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/motor_state");

  // Subscriber
  rclc_subscription_init_default(
    &command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/motor_command");

  // Configurar memoria de strings
  state_msg.data.data = state_buffer;
  state_msg.data.size = 0;
  state_msg.data.capacity = sizeof(state_buffer);

  command_msg.data.data = command_buffer;
  command_msg.data.size = 0;
  command_msg.data.capacity = sizeof(command_buffer);

  // Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor,
    &command_sub,
    &command_msg,
    &subscription_callback,
    ON_NEW_DATA);
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  if (motorEnMovimiento) {
    leerPWM();
    duty_msg.data = duty;
    voltaje_msg.data = voltaje;

    rcl_publish(&duty_pub, &duty_msg, NULL);
    rcl_publish(&voltaje_pub, &voltaje_msg, NULL);
  }

  state_msg.data.size = strlen(state_buffer);
  rcl_publish(&state_pub, &state_msg, NULL);

  delay(100);
  
}