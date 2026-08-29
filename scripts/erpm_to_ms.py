import rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped
import math

class VescSpeedConverter(Node):
    def __init__(self):
        super().__init__('vesc_speed_converter')
        
        # Sabitler
        self.poles = 14
        self.pole_pairs = self.poles / 2
        self.gear_ratio = 2.769
        self.wheel_radius = 0.055  # metre
        
        # Subscriber kurulumu
        self.subscription = self.create_subscription(
            VescStateStamped,
            '/sensors/core',
            self.listener_callback,
            10)
        
        self.get_logger().info('VESC Speed Converter başlatıldı...')

    def listener_callback(self, msg):
        # Mesaj içindeki erpm (speed) verisini al
        erpm = msg.state.speed
        
        # ERPM -> m/s Dönüşümü
        # 1. Motor RPM bul (ERPM / Kutup Çifti)
        motor_rpm = erpm / self.pole_pairs
        
        # 2. Tekerlek RPM bul (Motor RPM / Dişli Oranı)
        wheel_rpm = motor_rpm / self.gear_ratio
        
        # 3. m/s hesapla (RPM * 2 * PI * r / 60)
        speed_mps = (wheel_rpm * 2 * math.pi * self.wheel_radius) / 60
        
        # Terminale yazdır
        self.get_logger().info(f'ERPM: {erpm:.0f} | Speed: {speed_mps:.3f} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = VescSpeedConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
