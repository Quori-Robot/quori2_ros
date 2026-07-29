#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray,Float64
from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import Joy
import math
import time

JS_DEADBAND = 0.1

class HeadTeleop(Node):

    def __init__(self):

        super().__init__('head_teleop')

        self.neck=0.0
        self.pan =0.0
        self.tilt=0.0

        self.neck_rate = 0.15
        self.pan_rate = 1.5
        self.tilt_rate = 1.0

        self.declare_parameter("axis_neck",[4,5])
        self.declare_parameter("axis_pan",6)
        self.declare_parameter("axis_tilt",7)

       

        [self.neck_axis_p,self.neck_axis_n] = self.get_parameter('axis_neck').get_parameter_value().integer_array_value
        # = [int(n) for n in self.neck_axis.split(",")]

        self.axis_pan = self.get_parameter('axis_pan').get_parameter_value().integer_value
        self.axis_tilt = self.get_parameter('axis_tilt').get_parameter_value().integer_value

        self.get_logger().info(f'Head teleop Node got params:\n\taxis_neck [{self.neck_axis_p}-{self.neck_axis_n}]\n\taxis_pan {self.axis_pan}\n\taxis_tilt {self.axis_tilt}')



        #self.neck_pub = self.create_publisher(Float64, 'sim/neck_joint_pos_cmd', 10)
        #self.tilt_pub = self.create_publisher(Float64, 'sim/tilt_joint_pos_cmd', 10)
        #self.pan_pub = self.create_publisher(Float64, 'sim/pan_joint_pos_cmd', 10)
        
        self.neck_pub = self.create_publisher(Float64, 'sim/neck_joint_vel_cmd', 10)
        self.tilt_pub = self.create_publisher(Float64, 'sim/tilt_joint_vel_cmd', 10)
        self.pan_pub = self.create_publisher(Float64, 'sim/pan_joint_vel_cmd', 10)
        
        #self.publisher_ = self.create_publisher(MultiDOFCommand, 'pid_controller/reference', 10)
        self.joint_pos_pub = self.create_publisher(Float64MultiArray, 'head_joint_pos_cmds', 10)
      
        self.joint_vel_pub = self.create_publisher(Float64MultiArray, 'head_joint_vel_cmds', 10)
      
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.last_time = None
        
    def db(self,v):
        if abs(v) < JS_DEADBAND:
            return 0.0
        
        return (v - (JS_DEADBAND if v >=0.0 else - JS_DEADBAND)) / (1.0-JS_DEADBAND)

    def joy_callback(self, msg):

        t = self.get_clock().now()

        if( self.last_time is None):
            self.last_time = t
            return
        
        dt = (t-self.last_time).nanoseconds/ 1e9 
        self.last_time = t

        neck = (msg.axes[self.neck_axis_p]-msg.axes[self.neck_axis_n])/2
        pan  = -msg.axes[self.axis_pan]
        tilt = msg.axes[self.axis_tilt]
        
        neck = self.db(neck)
        pan = self.db(pan)
        tilt = self.db(tilt)

        
        self.neck = self.neck_rate  * neck 
        self.pan  = self.pan_rate   * pan 
        self.tilt = self.tilt_rate  * tilt 

        """ 
        if self.neck  < -0.05: #0.0:
            self.neck = -0.05 #0.0

        if self.neck  > 0.05: # 0.5:
            self.neck = 0.05  #0.5

        if self.pan  < -2.0*math.pi/3.0:
            self.pan = -2.0*math.pi/3.0

        if self.pan  > 2.0*math.pi/3.0:
            self.pan = 2.0*math.pi/3.0

        if self.tilt  < -math.pi/6.5:
            self.tilt = -math.pi/6.5

        if self.tilt  > math.pi/6.5:
            self.tilt = math.pi/6.5
        """
        #self.get_logger().info(f"Pan: {self.pan}, Tilt: {self.tilt}, Neck: {self.neck}")
        
        cmd_msg_group = Float64MultiArray()
        cmd_msg = Float64()

        cmd_msg.data =  -self.neck
        self.neck_pub.publish(cmd_msg) 

        cmd_msg.data = self.pan
        self.pan_pub.publish(cmd_msg) 
        
        cmd_msg.data = self.tilt 
        self.tilt_pub.publish(cmd_msg) 


        cmd_msg_group.data = [ -self.neck, self.pan, self.tilt ]
        #self.joint_pos_pub.publish(cmd_msg_group) 
        self.joint_vel_pub.publish(cmd_msg_group) 


        
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
  
