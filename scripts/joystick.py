import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

# Try importing pynput for keyboard support
try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False

class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')

        # Parameters
        self.declare_parameter('use_keyboard', False)
        # Joystick Axis Parameters
        self.declare_parameter('servo_axis', 0)
        self.declare_parameter('forward_axis', 5)
        self.declare_parameter('reverse_axis', 2)
        # settings
        self.declare_parameter('keyboard_speed', 5000.0) # Speed when using keyboard

        # Load parameter values
        self.use_keyboard = False
        self.servo_axis   = self.get_parameter('servo_axis').get_parameter_value().integer_value
        self.forward_axis = self.get_parameter('forward_axis').get_parameter_value().integer_value
        self.reverse_axis = self.get_parameter('reverse_axis').get_parameter_value().integer_value
        self.kb_max_speed = self.get_parameter('keyboard_speed').get_parameter_value().double_value

        # Publishers
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed',    10)

        # State Variables
        self.current_servo = 0.5  # Neutral (0.5)
        self.current_speed = 0.0  # Stopped (0.0)

        # Mode Selection
        if self.use_keyboard:
            if not PYNPUT_AVAILABLE:
                self.get_logger().error("pynput library not found! Cannot use keyboard mode. Please install pynput.")
            else:
                self.get_logger().info("Keyboard Mode Enabled. Controls: W (Forward), S (Reverse), A (Left), D (Right)")
                self.keys_pressed = set()
                # Start non-blocking keyboard listener
                self.listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
                self.listener.start()
                # Timer to update logic from keys
                self.create_timer(0.05, self.update_keyboard_logic)
        else:
            self.get_logger().info(f"Joystick Mode Enabled. Axes: Servo={self.servo_axis}, Fwd={self.forward_axis}, Rev={self.reverse_axis}")
            self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Optimization: Publish at a fixed rate (20Hz) instead of on every callback/event
        # This prevents spamming the simulation if inputs are noisy or frequent
        self.create_timer(0.05, self.publish_loop)

    def map_val(self, x, in_min, in_max, out_min, out_max):
        # Constrain input
        x = max(min(x, in_max), in_min)
        # Map
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def joy_callback(self, msg: Joy):
        """Callback for Joystick messages"""
        # Steering
        try:
            raw_steer = msg.axes[self.servo_axis]
            a_fwd = msg.axes[self.forward_axis]
            a_rev = msg.axes[self.reverse_axis]
        except IndexError:
            # self.get_logger().warn("Joystick index error")
            return

        # Trigger processing (usually -1 to 1, or 0 to 1 depending on driver, but assuming -1 is pressed or 1 is pressed)
        # Original code assumed: Rest at 1.0, Pull to -1.0 ?
        # Original: fwd_norm = (1.0 - a_fwd) / 2.0  -> If a_fwd=1 (rest), norm=0. If a_fwd=-1 (pressed), norm=1.
        
        fwd_norm = (1.0 - a_fwd) / 2.0
        rev_norm = (1.0 - a_rev) / 2.0
        
        # Combined Speed: +1 = Full Forward, -1 = Full Reverse
        raw_speed = fwd_norm - rev_norm

        # Update State (Don't publish here)
        # Servo: Joystick [-1, 1] -> Servo [0.2, 0.8]
        self.current_servo = self.map_val(raw_steer, -1.0, 1.0, 0.2, 0.8)
        
        # Speed: Combined [-1, 1] -> Speed [-10000, 10000]
        self.current_speed = self.map_val(raw_speed, -1.0, 1.0, -10000.0, 10000.0)

    # --- Keyboard Handlers ---
    def on_key_press(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                self.keys_pressed.add(key.char.lower())
        except AttributeError:
            pass

    def on_key_release(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                char = key.char.lower()
                if char in self.keys_pressed:
                    self.keys_pressed.remove(char)
        except AttributeError:
            pass

    def update_keyboard_logic(self):
        """Update speed and servo based on currently pressed keys"""
        target_servo = 0.5
        target_speed = 0.0

        # Steering
        if 'a' in self.keys_pressed:
            target_servo = 0.8 # Left (assuming similar mapping to joystick +1 raw)
        elif 'd' in self.keys_pressed:
            target_servo = 0.2 # Right

        # Speed
        if 'w' in self.keys_pressed:
            target_speed = self.kb_max_speed
        elif 's' in self.keys_pressed:
            target_speed = -self.kb_max_speed

        self.current_servo = target_servo
        self.current_speed = target_speed

    def publish_loop(self):
        """Publish the current state at fixed rate"""
        msg_servo = Float64()
        msg_servo.data = float(self.current_servo)
        
        msg_speed = Float64()
        msg_speed.data = float(self.current_speed)

        self.servo_pub.publish(msg_servo)
        self.speed_pub.publish(msg_speed)

def main(args=None):
    rclpy.init(args=args)
    node = JoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.use_keyboard and hasattr(node, 'listener'):
            node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
