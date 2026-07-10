import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import pandas as pd
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class CSVPathPublisher(Node):
    def __init__(self):
        super().__init__('csv_path_publisher')
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.publisher_ = self.create_publisher(Path, '/path', qos)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.flag = False
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        self.load_path_from_csv('/home/xavier/race_car_ws/rc_pure_pursuit/big_hallway_adjusted.csv')

    def load_path_from_csv(self, filename):
        try:
            df = pd.read_csv(filename)
            if 'x' not in df.columns or 'y' not in df.columns:
                self.get_logger().error("At the beginning of the CSV file, 'x' and 'y' columns are required.")
                return

            for _, row in df.iterrows():
                pose = PoseStamped()
                pose.header.frame_id = "odom"
                pose.pose.position.x = float(row['x'])
                pose.pose.position.y = float(row['y'])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                self.path_msg.poses.append(pose)

            self.get_logger().info(f"{len(self.path_msg.poses)} waypoints are loaded from {filename}.")

        except Exception as e:
            self.get_logger().error(f"CSV could not be read: {e}")

    def timer_callback(self):

        if not self.flag:
            now = self.get_clock().now().to_msg()
            self.path_msg.header.stamp = now
            for pose in self.path_msg.poses:
                pose.header.stamp = now
            self.publisher_.publish(self.path_msg)
            self.get_logger().info("Path published.")
            self.flag = True

def main(args=None):
    rclpy.init(args=args)
    node = CSVPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

