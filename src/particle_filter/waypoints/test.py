import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import pandas as pd

class CSVPathPublisher(Node):
    def __init__(self):
        super().__init__('csv_path_publisher')
        self.publisher_ = self.create_publisher(Path, 'path', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        # CSV dosyasından verileri yükle
        self.load_path_from_csv('/home/xavier/race_car_ws/f1tenth_ws/src/particle_filter/waypoints/waypoints.csv')

    def load_path_from_csv(self, filename):
        try:
            df = pd.read_csv(filename)
            if 'x' not in df.columns or 'y' not in df.columns:
                self.get_logger().error("CSV dosyasında 'x' ve 'y' sütunları olmalı.")
                return

            for _, row in df.iterrows():
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(row['x'])
                pose.pose.position.y = float(row['y'])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0  # yönelim sabit (başlangıç)
                self.path_msg.poses.append(pose)

            self.get_logger().info(f"{len(self.path_msg.poses)} nokta yüklendi.")

        except Exception as e:
            self.get_logger().error(f"CSV okunamadı: {e}")

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        for pose in self.path_msg.poses:
            pose.header.stamp = now
        self.publisher_.publish(self.path_msg)
        self.get_logger().info("Path yayınlandı.")

def main(args=None):
    rclpy.init(args=args)
    node = CSVPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
