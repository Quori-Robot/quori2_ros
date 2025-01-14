#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import Joy
import math
import time

class HeadTeleop(Node):

    def __init__(self):

        super().__init__('head_teleop')

        self.neck=0.0
        self.pan =0.0
        self.tilt=0.0

        self.neck_rate = 0.25
        self.pan_rate = 6.0
        self.tilt_rate = 6.0

        self.publisher_ = self.create_publisher(Float64MultiArray, 'vel_ctrl/commands', 10)
        #self.publisher_ = self.create_publisher(MultiDOFCommand, 'pid_controller/reference', 10)
      
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.last_time = None

    def joy_callback(self, msg):

        t = self.get_clock().now()

        if( self.last_time is None):
            self.last_time = t
            return
        
        dt = (t-self.last_time).nanoseconds/ 1e9 
        self.last_time = t

        neck = msg.axes[5]-msg.axes[2]
        pan = msg.axes[6]
        tilt = msg.axes[7]

        
        self.neck = self.neck_rate  * neck
        self.pan  = self.pan_rate   * pan
        self.tilt = self.tilt_rate  * tilt

        if self.neck  < -0.25: #0.0:
            self.neck = -0.25 #0.0

        if self.neck  > 0.25: # 0.5:
            self.neck = 0.25  #0.5

        if self.pan  < -math.pi/2:
            self.pan = -math.pi/2

        if self.pan  > math.pi/2:
            self.pan = math.pi/2

        if self.tilt  < -math.pi/4:
            self.tilt = -math.pi/4

        if self.tilt  > math.pi/4:
            self.tilt = math.pi/4


        #cmd_msg = MultiDOFCommand()
        cmd_msg = Float64MultiArray()

        #cmd_msg.dof_names = [ 'neck_joint', 'pan_joint', 'tilt_joint' ]
        cmd_msg.data = [ -self.neck, self.pan, -self.tilt ]
        #cmd_msg.values_dot = [ self.neck_rate, self.pan_rate, self.tilt_rate ]

        self.publisher_.publish(cmd_msg)
        
        #self.get_logger().info('I heard: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)

    head_teleop = HeadTeleop()

    rclpy.spin(head_teleop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    head_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
  