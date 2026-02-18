#Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

#Class Definition
class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.publisher_ = self.create_publisher(Float32, 'signal', 10)
        timer_period = 0.5 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.i = 0

    def timer_callback(self):
        msg = Float32()
        msg.data = float(self.i)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args = None):
    rclpy.init(args = args)
    signal_generator = SignalGenerator()

    try:
        rclpy.spin(signal_generator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        signal_generator.destroy_node()        

#Execute Node
if __name__ == '__main__':
    main()