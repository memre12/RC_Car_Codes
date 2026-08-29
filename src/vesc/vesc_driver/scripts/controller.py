import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial

class SerialPWMReader(Node):
    def __init__(self):
        super().__init__('serial_pwm_reader')

        self.declare_parameter('serial_port', '/dev/USB_REMOTE_CONTROLLER')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened serial port {port} at {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            self.ser = None

        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def map_pwm(self, value, in_min, in_max, out_min, out_max):
        # Clamp value within range and map
        value = max(min(value, in_max), in_min)
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def timer_callback(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port not open")
            return

        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split(',')
                if len(parts) == 4:
                    try:
                        pwm = [int(x) for x in parts]

                        # Mapping based on clarified input
                        servo_val = self.map_pwm(pwm[0], 1000, 2000, 0.8, 0.2)
                        speed_val = self.map_pwm(pwm[2], 1000, 2000, -5000.0, 5000.0)

                        self.servo_pub.publish(Float64(data=servo_val))
                        self.speed_pub.publish(Float64(data=speed_val))

                        # self.get_logger().info(f"Servo: {servo_val:.2f}, Speed: {speed_val:.2f}")
                    except ValueError:
                        self.get_logger().warn(f"Non-integer values received: {parts}")
                else:
                    self.get_logger().warn(f"Unexpected number of values: {line}")
        except Exception as e:
            self.get_logger().warn(f"Failed to read/parse serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialPWMReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
