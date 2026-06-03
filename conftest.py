import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'tests'))

# Initialize ROS 2 once for the entire test session
import rclpy
rclpy.init()
