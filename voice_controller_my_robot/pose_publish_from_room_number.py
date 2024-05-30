import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header

import json

package_name = "voice_controller_my_robot"
room_data_path = os.path.join(get_package_share_directory(package_name), 'data','rooms_data.json')

class PosePublishFromRoomNumber(Node):
    def __init__(self):
        super().__init__('pose_publish_from_room_number')
        self.rooms_data = self.read_rooms_data()
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.subscription = self.create_subscription(
            Int16,
            'room_number',  
            self.listener_callback,
            10)
        self.subscription
        self.get_logger().info("pose_publish_from_room_number has started")
        
    def listener_callback(self, msg):
        self.get_logger().info("Room number received")
        room_number = msg.data
        point = self.select_room(room_number)
        if point is not None: 
            self.publish_pose(point)
        
    def select_room(self, room_number):
        try:
            point = self.rooms_data[str(room_number)]
            self.get_logger().info(f"La salle {room_number} est à la position : {point}")
            return point
        except KeyError:
            self.get_logger().info(f"La salle {room_number} n'existe pas")
            return None
        
    def publish_pose(self, point):
        X = float(point['x'])
        Y = float(point['y'])
        
        msg = PoseStamped()
        # Set the header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg() 
        msg.header.frame_id = 'map'
        
        # Set the pose
        msg.pose.position = Point(x=X, y=Y, z=0.0)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pose_publisher_.publish(msg)
        self.get_logger().info("Goal pose published")
        
    
    def read_rooms_data(self):
        with open(room_data_path, 'r') as f:
            return json.load(f)
    
    
    
def main(args=None):
    rclpy.init(args=args)
    node = PosePublishFromRoomNumber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
