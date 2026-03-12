// Micro-ROS y ESP32
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <Arduino.h>
#include <math.h>

// --- Pines motor y encoder ---
#define EnA  13  // PWM
#define In1  27
#define In2  14
#define EncA 32
#define EncB 35

#define freq 980
#define resolution 8
#define PWM1_Ch 0

// --- Variables micro-ROS ---
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t motor_node;

// Publisher
rcl_publisher_t velocity_pub;
rcl_publisher_t control_pub;
std_msgs__msg__Float32 control_msg;
std_msgs__msg__Float32 velocity_msg;

// Suscription
rcl_subscription_t setpoint_sub;
std_msgs__msg__Float32 setpoint_msg;

// PID
float Kp = 0.119193823031844, Ki = 0.94258078365982, Kd = 0.00124321749106575;
float error_prev = 0, integral = 0;
float Ts = 0.01; // 10ms

float set_point = 0.0;
float velocity = 0.0;

// Encoder
volatile int32_t delta_tiempo = 2000000; 
volatile uint32_t tiempo_ant = 0;
volatile bool encoderDirection = true;
const float pulsos_por_vuelta = 919.0;

// --- Interrupción encoder ---
void IRAM_ATTR Encoder() {
  encoderDirection = (digitalRead(EncA) != digitalRead(EncB));
  
  uint32_t tiempo_act = micros();
  delta_tiempo = tiempo_act - tiempo_ant;
  tiempo_ant = tiempo_act;
}

// --- Función para actualizar velocidad desde encoder ---
void update_velocity(){
  uint32_t tiempo_desde_ultimo = micros() - tiempo_ant;
  
  if(tiempo_desde_ultimo > 100000) {
    velocity = 0.0;
    return;
  }

  if(delta_tiempo <= 0) return;
  
  float rad_s = 6283185.3 / (pulsos_por_vuelta * (float)delta_tiempo);
  if(encoderDirection) {
    velocity = rad_s;
  } else {
    velocity = -rad_s;
  }

  //float rpm = 60000000.0 / (pulsos_por_vuelta * (float)delta_tiempo);
  //velocity = encoderDirection ? rpm : -rpm;
}
// --- Callbacks micro-ROS ---
void set_motor(float output){
  // Saturación de seguridad [-1.0, 1.0]
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;

  if(output > 0){
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
  } 
  else if (output < 0) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
  } 
  else {
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
  }

  int duty = (int)(fabs(output) * 255.0);
  ledcWrite(PWM1_Ch, duty);
}

void setpoint_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  set_point = msg->data;
}

// --- PID ---
void pid_control(){
  float error = set_point - velocity;

  // Integral con Anti-Windup
  float integral_limit = 1.0 / Ki;
  integral += error * Ts;
  integral = max(min(integral, integral_limit), -integral_limit);

  float derivative = (error - error_prev) / Ts;
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  error_prev = error;

  set_motor(output);
  control_msg.data = output;
  rcl_publish(&control_pub, &control_msg, NULL);
}

// Crear entidades (nodo, 2 publicadores, 1 suscripción)
bool create_entities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Nodo único
  rclc_node_init_default(&motor_node, "motor_driver_node", "", &support);

  rclc_publisher_init_default(
    &velocity_pub,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32),
    "velocity");

  rclc_publisher_init_default(
    &control_pub,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32),
    "control_signal");

  rclc_subscription_init_default(
    &setpoint_sub,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32),
    "set_point");

  // Executor único
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &setpoint_sub, &setpoint_msg, &setpoint_callback, ON_NEW_DATA);

  return true;
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  set_microros_transports();

  // Pines motor
  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  ledcSetup(PWM1_Ch, freq, resolution);
  ledcAttachPin(EnA, PWM1_Ch);

  // Pines encoder
  pinMode(EncA, INPUT_PULLUP);
  pinMode(EncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE);
}

// --- Loop ---
void loop() {
  static bool connected = false;
  if(!connected) {
    connected = create_entities();
    if(!connected) {
      delay(1000);
      return;
    }
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  // Publicar velocidad y ejecutar PID
  static uint32_t last_time = 0;
  uint32_t now = millis();
  if(now - last_time >= (uint32_t)(Ts*1000)) {
    update_velocity();
    velocity_msg.data = velocity;
    rcl_publish(&velocity_pub, &velocity_msg, NULL);
    pid_control();
    last_time = now;
  }
}