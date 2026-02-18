#Imports
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

# Class Definition
class SignalProcessing(Node):
    def __init__(self):
        super().__init__('signal_proc')        
        self.subscription_time = self.create_subscription(Float32, 'time', self.time_cb, 10)
        self.subscription_signal = self.create_subscription(Float32, 'signal', self.signal_cb, 10)        
        self.publisher_proc_signal = self.create_publisher(Float32, 'proc_signal', 10)
    
        # Parámetros de procesamiento 
        self.f = 10.0 # Hz
        self.omega = 2 * np.pi * self.f
        self.theta = np.pi # Desplazamiento de fase
        self.scale = 0.5 # Escala de amplitud
        self.offset = 1.0 # Offset >= 0.5
        self.t = 0.0 # Tiempo Actual

    def signal_cb(self, msgS):
        self.get_logger().info(f'Señal recibida: {msgS.data:.2f}')

        # Procesar señal
        x = msgS.data
        dx = self.omega * np.cos(self.omega * self.t)
        y = x * np.cos(self.theta) + (dx / self.omega) * np.sin(self.theta) # Desplazamiento
        y = self.scale * y + self.offset # Escala y offset
        
        # Publicar señal procesada
        msgP = Float32()
        msgP.data = y
        self.publisher_proc_signal.publish(msgP)
        self.get_logger().info(f'Señal procesada: {msgP.data:.2f}')

    def time_cb(self, msgT):
        self.get_logger().info(f'Tiempo recibido: {msgT.data:.2f}')
        self.t = msgT.data
        

def main(args=None):
    rclpy.init(args = args)
    signal_proc = SignalProcessing()

    try:
        rclpy.spin(signal_proc)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        signal_proc.destroy_node()

# Execute Node
if __name__ == '__main__':
    main()