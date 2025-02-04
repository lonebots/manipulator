from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')
        
        qos_profile = QoSProfile(depth = 10)
        self.join_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        
        
    
    