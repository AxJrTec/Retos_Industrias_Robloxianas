import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        #Parametros
        self.declare_parameter('kp', 0.609)
        self.declare_parameter('ki', 62.952)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('Ts', 0.01) #Tiempo de muestreo 

        #Variables de estado
        self.motor = 0.0 #Estado actual del motor (velocidad)
        self.ref = 0.0 #Referencia actual del set point
        self.error_prev = 0.0
        self.integral = 0.0

        #Subscribers
        self.set_point_sub = self.create_subscription(Float32, 'set_point', self.setpoint_callback, 10)
        self.motor_speed_sub = self.create_subscription(Float32, 'motor_speed_y', self.output_callback, 10)

        #Publisher
        self.motor_input_pub = self.create_publisher(Float32, 'motor_input_u', 10)

        #Control loop
        self.timer = self.create_timer(self.get_parameter('Ts').value, self.control_loop)
        self.get_logger().info('Controller node started')

    #Callbacks
    def setpoint_callback(self, msg):
        self.ref = msg.data

    def output_callback(self, msg):
        self.motor = msg.data

    #PID
    def control_loop(self):
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        Ts = self.get_parameter('Ts').value

        #Diferencia entre la referencia y la velocidad del motor
        error = self.ref - self.motor

        self.integral += error * Ts
        derivative = (error - self.error_prev) / Ts

        #Calculo de la señal de control
        u = kp * error + ki * self.integral + kd * derivative

        #Publicar la entrada de control
        msg = Float32()
        msg.data = u
        self.motor_input_pub.publish(msg)

        #Actualizar variables de estado
        self.error_prev = error

def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()