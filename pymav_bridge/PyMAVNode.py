#!/usr/bin/python3
"""
A node for interfacing with the MAVlink messages from ArduPilot without needing to spin up a whole MAVROS instance.
Note: this is experimental and should only be used in lieu of support for the SERVO_OUTPUT_RAW message in MAVROS. 
More research is needed for optimization as PyMAVlink is slow and has some issues.

Date Created: 05/06/2023
Date Modified: 05/06/2023

CHANGELOG:
v1.0.0 - Initial Release
v1.0.1 - Updated docs
"""

from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16, Int32
from seacat_msg.msg import Throttle

__author__ = "Braidan Duffy"
__copyright__ = "Copyright 2023, PANTHER Boat Team"
__credits__ = ["Braidan Duffy"]
__license__ = "MIT"
__version__ = "1.0.1"
__maintainer__ = "Braidan Duffy"
__email__ = "bduffy2018@my.fit.edu"
__status__ = "Experimental"


class PyMAVNode(Node):
    """
    A node for interfacing more simply with MAVlink messages from ArduPilot without needing to spin up a MAVROS instance.
    Note: this is experimental and should only be used in lieu of support for the SERVO_OUTPUT_RAW message in MAVROS. 
    More research is needed for optimization as PyMAVlink is slow and has some issues.
    
    Parameters:
        conn_type (str): the connection type that PyMAVlink should use to establish a connection to ArduPilot. Default: 'udpin'
        address (str): the IP address PyMAVlink should use to establish a connection to ArduPilot. Default: '0.0.0.0' (localhost)
        port (str): the port that PyMAVlink should listen on for MAVlink messages. Default: '14550'
    """
    MAV_CONN_TYPE: str
    MAV_ADDR: str
    MAV_PORT: str
    MAV_CONNECTION: str
    master: mavutil.mavlink_connection
    
    def __init__(self):
        super().__init__('PyMAVNode')
        
        self.declare_parameter('conn_type', 'udpin')
        self.declare_parameter('address', '0.0.0.0')
        self.declare_parameter('port', '14550')
        
        # Create MAVLink connection
        self.MAV_CONN_TYPE = self.get_parameter("conn_type").get_parameter_value().string_value
        self.MAV_ADDR = self.get_parameter('address').get_parameter_value().string_value
        self.MAV_PORT = self.get_parameter('port').get_parameter_value().string_value
        self.MAV_CONNECTION = f"{self.MAV_CONN_TYPE}:{self.MAV_ADDR}:{self.MAV_PORT}"
        self.get_logger().info(f"Listening for MAVlink connections at: {self.MAV_CONNECTION}")
        
        self.master = mavutil.mavlink_connection(self.MAV_CONNECTION)
        if not self.master:
            self.get_logger().fatal(f"Unable to establish a MAVlink connection at: {self.MAV_CONNECTION}")
            raise Exception("Unable to establish MAVlink connection!")
        else:
            self.get_logger().info("Established a MAVlink connection!")
        
        # Set up topic names
        self.topicname_throttle_prefix = "throttle"
        self.topicname_throttle_port = f"{self.topicname_throttle_prefix}/port"
        self.topicname_throttle_stbd = f"{self.topicname_throttle_prefix}/stbd"
        self.topicname_armed = "armed"
        
        # Set up publishers/subscribers
        self.publisher_throttle_port = self.create_publisher(Int16,
                                                             self.topicname_throttle_port,
                                                             10)
        self.publisher_throttle_stbd = self.create_publisher(Int16,
                                                             self.topicname_throttle_stbd,
                                                             10)
        self.get_logger().info(f"Publishing throttle data to: {self.topicname_throttle_port} and {self.topicname_throttle_stbd}")
        
        # Set up timers
        self.timer_pymav_heart = self.create_timer(0.1, self.handle_pymav_heartbeat)
        self.timer_pymav_servo = self.create_timer(0.1, self.handle_pymav_servo)
        self.get_logger().info("Created a timer to check for new heartbeat messages at 10 Hz")
        
    
    # =======================
    # === TIMER CALLBACKS ===
    # =======================
    
        
    def handle_pymav_heartbeat(self):
        """
        Receive a heartbeat message from ArduPilot and publish system status to the appropriate ROS topics
        """
        mav_msg = self.master.recv_match(type="HEARTBEAT")
        if not mav_msg: # Check if HEARTBEAT message received
            return
        self.get_logger().debug(f"Got heartbeat message: {mav_msg.to_dict()}")
        self.get_logger().debug(f"System status: {mav_msg.system_status}")
        
        # TODO: Fill out PyMAVStatus with mav_msg parameters.
    
    def handle_pymav_servo(self):
        """
        Receive a servo output message from ArduPilot and publish the port and starboard throttles to the appropriate ROS topics
        """
        mav_msg = self.master.recv_match(type="SERVO_OUTPUT_RAW")
        if not mav_msg: # Check if SERVO_OUTPUT_RAW message received
            return
        _dict = mav_msg.to_dict()
        self.get_logger().debug(f"Got servo message: {_dict}")
        self.get_logger().debug(f"Throttle port: {_dict['servo1_raw']} us")
        self.get_logger().debug(f"Throttle stbd: {_dict['servo3_raw']} us")
        
        self.publisher_throttle_port.publish(Int16(data=_dict["servo1_raw"]))
        self.publisher_throttle_stbd.publish(Int16(data=_dict["servo3_raw"]))
        self.get_logger().info(f"Published {_dict['servo1_raw']} us to {self.topicname_throttle_port}")
        self.get_logger().info(f"Published {_dict['servo3_raw']} us to {self.topicname_throttle_stbd}")


def main(args=None):
    rclpy.init(args=args)
    node = PyMAVNode()
    rclpy.spin(node)		#Spin the ROS2 node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()