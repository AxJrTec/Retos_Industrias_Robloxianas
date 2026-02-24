# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32

#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        self.declare_parameter('signal_type')
        self.declare_parameter('amplitude')
        self.declare_parameter('omega')
        
        # Retrieve sine wave parameters
        self.amplitude = 2.0
        self.omega  = 1.0
        self.timer_period = 0.1 #seconds

        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        
        #Create a messages and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.get_logger().info("SetPoint Node Started \U0001F680")

    def timer_cb(self):
        # Get the signal type parameter
        self.omega = self.get_parameter('omega').get_parameter_value().double_value
        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        signal_type = self.get_parameter('signal_type').get_parameter_value().string_value
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Generate selected signal
        if signal_type == 'sine':
            self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        elif signal_type == 'square':
            self.signal_msg.data = self.amplitude * (1.0 if np.sin(self.omega * elapsed_time) >= 0 else -1.0)
        elif signal_type == 'triangle':
            self.signal_msg.data = self.amplitude * (2 * abs((elapsed_time / (2 * np.pi)) % 1 - 0.5) - 0.5)
        else:
            self.get_logger().warn(f"Unknown signal type '{signal_type}'. Defaulting to sine wave.")
            self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)

        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)
    

#Main
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
