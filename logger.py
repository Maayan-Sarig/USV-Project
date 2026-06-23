# logs the nodes behavior for excel or MATLAB files
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import tkinter as tk
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import csv
from datetime import datetime

class Logger(Node):
    def __init__(self):
        super().__init__('display_data')

        self.tension = 0.0
        self.angle = 0.0
        self.loc = Float32MultiArray()
        self.loc.data = [0.0, 0.0, 0.0]
        self.dir = 0.0
        self.t200_sp = Float32MultiArray()
        self.t200_sp.data = [0.0, 0.0]
        self.length = 0.0
        self.start_time = self.get_clock().now().to_msg()
        self.spins = 0.0
        self.dep = 0.0
        self.c = Float32MultiArray()
        self.c.data =[0.0, 0.0]
        self.g = Float32MultiArray()
        self.g.data =[0.0,0.0,0.0,0.0]

        # Creates the logger
        self.log_file = open('node_log.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['Time', 'Tension', 'Angle', 'X', 'Y', 'Z', 'Direction', 'T200_Forward', 'T200_Reverse'])

        self.create_timer(0.05, self.log_to_file)  # 5 Hz logging

        # ROS subscriptions
        self.subscription1 = self.create_subscription(Float32, 'tension', self.listener_callback1, 10)
        self.get_logger().info("Subscribed to 'tension'")

        self.subscription2 = self.create_subscription(Float32, 'encoder_angle', self.listener_callback2, 10)
        self.get_logger().info("Subscribed to 'encoder_angle'")

        self.subscription3 = self.create_subscription(Float32MultiArray, 'location', self.listener_callback3, 10)
        self.get_logger().info("Subscribed to 'imu'")

        self.subscription4 = self.create_subscription(Float32, 'stepper', self.listener_callback4, 10)
        self.get_logger().info("Subscribed to 'Stepper'")

        self.subscription5 = self.create_subscription(Float32MultiArray, 't200_speed', self.listener_callback5, 10)
        self.get_logger().info("Subscribed to 'T200'")

        self.subscription6 = self.create_subscription(Float32, 'subm', self.depth, 10)
        self.get_logger().info("Subscribed to Tester")

        self.subscription7 = self.create_subscription(Float32MultiArray, 'check', self.check, 10)
        self.get_logger().info("Subscribed to check")

        self.subscription8 = self.create_subscription(Float64MultiArray, 'gps', self.gps, 10)
        self.get_logger().info("Subscribed to gps")

    def listener_callback1(self, msg):
        self.tension = msg.data

    def listener_callback2(self, msg):
        self.angle = msg.data

    def listener_callback3(self, msg):
        x, y, z = msg.data
        self.loc.data = [x,y,z]

    def listener_callback4(self, msg):
        self.dir = msg.data

    def listener_callback5(self, msg):
        forward, reverse = msg.data
        self.t200_sp.data = [forward, reverse]
    
    def gps (self, msg):
        f,x,y,z = msg.data
        self.g.data = [f,x,y,z]
    
    def depth(self,msg):
        self.dep = -msg.data

    def check(self,msg):
        x,y = msg.data
        self.c.data = [x,y]

    def log_to_file(self):
        now = self.get_clock().now().to_msg()
        # Convert time to float seconds
        t_now = now.sec + now.nanosec * 1e-9
        t_start = self.start_time.sec + self.start_time.nanosec * 1e-9
        t_elapsed = t_now - t_start
        time_str = f"{now.sec}.{str(now.nanosec).zfill(9)}"

        if self.dir:
            if self.spins <= 0:
                self.spins = 0
                
            self.spins += (self.dir/60.0) * 0.2
            if abs(self.spins) > 23.125:
                n = int(self.spins/23.125)
            else:
                 n = 0

            self.length =  2.0 * (self.spins % 23.125) * 3.14 * (0.095 + 0.0076/2.0 + (5 - n) *0.0076)
            if n > 0:
                for i in range(n):
                    self.length += 2.0 * 23.125 * 3.14 * (0.095 + 0.0076/2.0 + (5 - i) *0.0076)
        row = [
            t_elapsed,
            #self.tension,
            #self.angle,
            self.loc.data[0],
            self.loc.data[1],
            self.g.data[1],
            self.g.data[2],
            self.g.data[3],
            self.g.data[0],
            #self.loc.data[2],
            #self.dir,
            #self.length,
            #self.spins,
            #self.t200_sp.data[0],
            #self.t200_sp.data[1],
            #self.dep
        ]
        self.csv_writer.writerow(row)