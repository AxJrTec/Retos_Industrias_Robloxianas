import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class Motor_Input(Node):
    def __init__(self):
        super().__init__("motor_input")

        #Parametros a declarar
        self.declare_parameter('signal_type', 'sine') #square, step, sine, triangle
        self.declare_parameter('amplitude', 10.0) #velocidad en rad/s
        self.declare_parameter('omega', 1.0) #frecuencia en Hz

        #Leer parametros
        self.signal_type = self.get_parameter ("signal_type").value
        self.amplitude = self.get_parameter ("amplitude").value
        self.omega = self.get_parameter ("omega").value

        #Publisher 
        self.publisher = self.create_publisher(Float32, "/set_point", 10)
        self.timer_period = 0.1 #segundos
        self.timer = self.create_timer(self .timer_period, self.publish_signal)

        self.start_time = self.get_clock().now()
        self.get_logger().info("Motor_Input node started")

        
    def publish_signal(self):
        #Leemos los parametros
        self.signal_type = self.get_parameter('signal_type').get_parameter_value().string_value
        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.omega = self.get_parameter('omega').get_parameter_value().double_value
        
        self.amplitude = np.clip(self.amplitude, -14.66, 14.66) # Limite de velocidad en rad/s
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        msg = Float32()

        # Señales posibles a generar
        if self.signal_type == 'sine':
            msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        elif self.signal_type == 'square':
            msg.data = self.amplitude * (1.0 if np.sin(self.omega * elapsed_time) >= 0 else -1.0)
        elif self.signal_type == 'triangle':
            msg.data = self.amplitude * (2/np.pi) * np.arcsin(np.sin(self.omega * elapsed_time))
        elif self.signal_type == 'step':
            msg.data = self.amplitude if elapsed_time >= 1 else 0.0
        else:
            self.get_logger().warn(f"Señal desconocida '{self.signal_type}'. Señal senoidal usada por defecto.")
            msg.data = self.amplitude * np.sin(self.omega * elapsed_time)

        self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    set_point = Motor_Input()
    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
