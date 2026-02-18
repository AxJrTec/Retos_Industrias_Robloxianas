#Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

# Class Definition
class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_gen')
        self.publisher_time = self.create_publisher(Float32, 'time', 10)
        self.publisher_signal = self.create_publisher(Float32, 'signal', 10)
        
        # Parámetros
        self.f = 10.0 # Hz
        self.A = 1.0 # Amplitud
        self.omega = 2 * np.pi * self.f
        self.t = 0.0 #

        #Timer
        timer_period = 0.00001 # Segundos
        self.timer = self.create_timer(timer_period, self.timer_cb)


    # Timer Callback
    def timer_cb(self):

        # Publicar tiempo
        msgT = Float32()
        msgT.data = self.t
        self.publisher_time.publish(msgT)
        self.get_logger().info(f"Tiempo: {msgT.data:.2f}")

        # Publicar señal
        msgS = Float32()
        msgS.data = self.A * np.sin(self.omega * self.t)
        self.publisher_signal.publish(msgS)
        self.get_logger().info(f"Señal: {msgS.data:.2f}")
        self.t += 0.00001


def main(args = None):
    rclpy.init(args = args)
    signal_gen = SignalGenerator()

    try:
        rclpy.spin(signal_gen)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        signal_gen.destroy_node()        

# Execute Node
if __name__ == '__main__':
    main()