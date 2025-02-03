#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray,Float64
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

        self.neck_rate = 0.3
        self.pan_rate = 4.0
        self.tilt_rate = 0.5

        self.neck_pub = self.create_publisher(Float64, 'sim/neck_joint_pos_cmd', 10)
        self.tilt_pub = self.create_publisher(Float64, 'sim/tilt_joint_pos_cmd', 10)
        self.pan_pub = self.create_publisher(Float64, 'sim/pan_joint_pos_cmd', 10)
        #self.publisher_ = self.create_publisher(MultiDOFCommand, 'pid_controller/reference', 10)
        self.joint_pos_pub = self.create_publisher(Float64MultiArray, 'head_joint_pos_cmds', 10)
      
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

        neck = (msg.axes[5]-msg.axes[4])/2
        pan  = msg.axes[6]
        tilt = msg.axes[7]

        
        self.neck += self.neck_rate  * neck * dt
        self.pan  += self.pan_rate   * pan  * dt
        self.tilt += self.tilt_rate  * tilt * dt

        if self.neck  < -0.2: #0.0:
            self.neck = -0.2 #0.0

        if self.neck  > 0.2: # 0.5:
            self.neck = 0.2  #0.5

        if self.pan  < -math.pi/2:
            self.pan = -math.pi/2

        if self.pan  > math.pi/2:
            self.pan = math.pi/2

        if self.tilt  < -math.pi/4:
            self.tilt = -math.pi/4

        if self.tilt  > math.pi/4:
            self.tilt = math.pi/4


        cmd_msg_group = Float64MultiArray()
        cmd_msg = Float64()

        cmd_msg.data =  -self.neck
        self.neck_pub.publish(cmd_msg) 

        cmd_msg.data = self.pan
        self.pan_pub.publish(cmd_msg) 
        
        cmd_msg.data = -self.tilt 
        self.tilt_pub.publish(cmd_msg) 


        cmd_msg_group.data = [ -self.neck, self.pan, -self.tilt ]
        self.joint_pos_pub.publish(cmd_msg_group) 


        
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
  