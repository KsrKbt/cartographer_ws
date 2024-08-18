import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Twist, TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import websocket
import json
import threading
import time

class DataTransferNode(Node):
    def __init__(self):
        super().__init__('data_transfer_node')
        
        # QoSプロファイルの設定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # tf_staticのためのQoSプロファイル
        qos_profile_tf_static = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, qos_profile)
        self.tf_static_sub = self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, qos_profile_tf_static)
        
        self.ws = None
        self.connect_websocket()

    def scan_callback(self, msg):
        self.send_to_pc2('/scan', msg)

    def odom_callback(self, msg):
        self.send_to_pc2('/odom', msg)

    def tf_callback(self, msg):
        self.send_to_pc2('/tf', msg)

    def tf_static_callback(self, msg):
        self.send_to_pc2('/tf_static', msg)

    def send_to_pc2(self, topic, msg):
        data = {
            "op": "publish",
            "topic": topic,
            "msg": self.ros_msg_to_dict(msg)
        }
        try:
            if self.ws and self.ws.sock and self.ws.sock.connected:
                self.ws.send(json.dumps(data))
            else:
                self.get_logger().warn("WebSocket is not connected. Attempting to reconnect...")
                self.connect_websocket()
        except Exception as e:
            self.get_logger().error(f"Error sending data: {e}")

    def on_message(self, ws, message):
        # This method is not used in this implementation
        pass

    def on_open(self, ws):
        self.get_logger().info("WebSocket connection opened")

    def on_error(self, ws, error):
        self.get_logger().error(f"WebSocket error: {error}")
    
    def on_close(self, ws, close_status_code, close_msg):
        self.get_logger().info(f"WebSocket connection closed: {close_status_code} - {close_msg}")
        self.connect_websocket()  # 接続が閉じられたら再接続を試みる

    def connect_websocket(self):
        while rclpy.ok():
            try:
                self.ws = websocket.WebSocketApp("ws://192.168.1.29:9090",
                                                 on_open=self.on_open,
                                                 on_message=self.on_message,
                                                 on_error=self.on_error,
                                                 on_close=self.on_close)
                self.ws_thread = threading.Thread(target=self.ws.run_forever)
                self.ws_thread.start()
                break
            except Exception as e:
                self.get_logger().error(f"Failed to connect to WebSocket: {e}")
                time.sleep(5)  # 5秒待ってから再試行

    def ros_msg_to_dict(self, msg):
        if isinstance(msg, LaserScan):
            return {
                "header": self.header_to_dict(msg.header),
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "angle_increment": msg.angle_increment,
                "time_increment": msg.time_increment,
                "scan_time": msg.scan_time,
                "range_min": msg.range_min,
                "range_max": msg.range_max,
                "ranges": list(msg.ranges),
                "intensities": list(msg.intensities)
            }
        elif isinstance(msg, Odometry):
            return {
                "header": self.header_to_dict(msg.header),
                "child_frame_id": msg.child_frame_id,
                "pose": self.pose_with_covariance_to_dict(msg.pose),
                "twist": self.twist_with_covariance_to_dict(msg.twist)
            }
        elif isinstance(msg, TFMessage):
            return {
                "transforms": [self.transform_stamped_to_dict(transform) for transform in msg.transforms]
            }
        else:
            self.get_logger().warn(f"Unsupported message type: {type(msg)}")
            return {}

    def header_to_dict(self, header):
        return {
            "stamp": {
                "sec": header.stamp.sec,
                "nanosec": header.stamp.nanosec
            },
            "frame_id": header.frame_id
        }

    def pose_with_covariance_to_dict(self, pose_with_cov):
        return {
            "pose": self.pose_to_dict(pose_with_cov.pose),
            "covariance": list(pose_with_cov.covariance)
        }

    def twist_with_covariance_to_dict(self, twist_with_cov):
        return {
            "twist": self.twist_to_dict(twist_with_cov.twist),
            "covariance": list(twist_with_cov.covariance)
        }

    def pose_to_dict(self, pose):
        return {
            "position": self.point_to_dict(pose.position),
            "orientation": self.quaternion_to_dict(pose.orientation)
        }

    def twist_to_dict(self, twist):
        return {
            "linear": self.vector3_to_dict(twist.linear),
            "angular": self.vector3_to_dict(twist.angular)
        }

    def point_to_dict(self, point):
        return {"x": point.x, "y": point.y, "z": point.z}

    def quaternion_to_dict(self, quaternion):
        return {"x": quaternion.x, "y": quaternion.y, "z": quaternion.z, "w": quaternion.w}

    def vector3_to_dict(self, vector3):
        return {"x": vector3.x, "y": vector3.y, "z": vector3.z}

    def transform_stamped_to_dict(self, transform_stamped):
        return {
            "header": self.header_to_dict(transform_stamped.header),
            "child_frame_id": transform_stamped.child_frame_id,
            "transform": {
                "translation": self.vector3_to_dict(transform_stamped.transform.translation),
                "rotation": self.quaternion_to_dict(transform_stamped.transform.rotation)
            }
        }

def main():
    rclpy.init()
    node = DataTransferNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()