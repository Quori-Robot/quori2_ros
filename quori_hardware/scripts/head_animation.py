#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray,Float64
from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import Joy
import math
import time
import csv
from threading import Thread
from rclpy.qos import QoSProfile, ReliabilityPolicy

JS_DEADBAND = 0.1


# cyclic position command interface values
CP_CMD_STOP=0 # send stop command
CP_CMD_CONT=1 # set final velocity = velocity
CP_CMD_ZERO=2 # set final velocity to 0
PP_CMD_POS = 3 # set profile position 
PV_CMD_VEL = 4 # set profile velocity 

class HeadAnimation(Node):

    def __init__(self):

        super().__init__('head_animation')

        
        self.declare_parameter("anim_file","neck_anim.csv")

        self.data_file = self.get_parameter("anim_file").get_parameter_value().string_value
        

        self.get_logger().info(f'Head Animation Node got params:\n\tanim_file {self.data_file}')
        
        qos_profile = QoSProfile(
            depth=1, # Depth of the message queue
            reliability=ReliabilityPolicy.RELIABLE # Set reliability to RELIABLE
        )
           
        #self.neck_sim_pub = self.create_publisher(Float64, 'sim/neck_joint_vel_cmd', qos_profile)
        #self.tilt_sim_pub = self.create_publisher(Float64, 'sim/tilt_joint_vel_cmd', qos_profile)
        #self.pan_sim_pub = self.create_publisher(qos_profile)
        
        self.neck_pub = self.create_publisher(Float64MultiArray, '/neck_cyclic_pos_ctrl/commands', qos_profile,)
        self.tilt_pub = self.create_publisher(Float64MultiArray, '/tilt_cyclic_pos_ctrl/commands', qos_profile)
        self.pan_pub = self.create_publisher(Float64MultiArray, '/pan_cyclic_pos_ctrl/commands', qos_profile)
        
        #self.joint_pos_pub = self.create_publisher(Float64MultiArray, 'head_joint_pos_cmds', 10)
        #self.joint_vel_pub = self.create_publisher(Float64MultiArray, 'head_joint_vel_cmds', 10)

        self.last_time = None
        
        self.spin_thread = None
        
        self.max_time=-1.0
        
        #while self.neck_pub.get_subscription_count() <  1 and self.pan_pub.get_subscription_count() <  1 and self.tilt_pub.get_subscription_count() <  1:
        #    rclpy.spin_once(self)
            
        print("Got all topics subscribers\n")
        
        with open(self.data_file,newline='') as anim_fh:
            reader = csv.DictReader(anim_fh, ["Joint","Time","Frame","Pos","Vel","Accel"])
            self.parse_data(reader)
                
        
    def parse_data(self,data):
        

        self.pan =  {}
        self.neck = {}
        self.tilt = {}
        
        joint_lookup = {"Head_LR" : self.pan, "Head_UD" : self.tilt, "Head_Translate" : self.neck}
    
        skip_headers=True
        
        for line in data:
            if skip_headers:
                skip_headers = False
                continue
            
            data_list = joint_lookup[line["Joint"]]
            
            step_time = line["Time"]
            
            t = float(step_time)
            self.max_time = max(t,self.max_time)
            
            data_list[step_time] = (float(line["Pos"]) ,float(line["Vel"]), float(line["Accel"]))
     
        #print(self.pan)
        #print(self.neck)
        #print(self.tilt)
        
    def run(self):

        # Spin in a separate thread to process subscription callbacks concurrently
        self.spin_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

        SampleRate = 100 # 100 Hz
        rate = self.create_rate(SampleRate) 
        
        try:

            times = self.pan.keys()
            
            do_startup=True
        
            for t in times:
                
                if not rclpy.ok():
                    break
                
                if do_startup:
                    rate.sleep()
                   
                    init_pos_pan=self.pan[t][0]
                    init_pos_neck=self.neck[t][0]
                    init_pos_tilt=self.tilt[t][0]
                    print(f"Do Start up: Moving to Pan={init_pos_pan}, Neck={init_pos_neck}, Tilt={init_pos_tilt}\n")
                    # move to initial position
                    
                    for i in range(100):
                        if not rclpy.ok():
                            break
                        rate.sleep()
                        self.send_cmd((0.0,0.0,math.nan),(0.0,0.0,math.nan),(0.0,0.0,math.nan), CP_CMD_ZERO)
                      
                    rate.sleep()
                    self.send_cmd((init_pos_pan,0.5,2.0), (init_pos_neck,20.0,10.0),(init_pos_tilt,0.5,2.0), PP_CMD_POS)
                   
                    start_delay = 15 * SampleRate # wait for initial move
                    
                    while start_delay>0:
                        if not rclpy.ok():
                            break
                        rate.sleep()
                        self.send_cmd((init_pos_pan,0.5,2.0), (init_pos_neck,20.0,10.0),(init_pos_tilt,0.5,2.0), PP_CMD_POS)
                        
                        start_delay = start_delay-1
                        
                    do_startup = False
                    
                    print("Done Delay\n")
                    
                
                if not rclpy.ok():
                    break
                
                tf = float(t)      
                rate.sleep()
                self.send_cmd(self.pan[t],self.neck[t],self.tilt[t], CP_CMD_CONT if tf < self.max_time else CP_CMD_ZERO)
               
                
                
                #break

                    
                
        except KeyboardInterrupt:
            pass
        
        for _ in range(5):
            rate.sleep()
            self.send_cmd(self.pan[t],self.neck[t],self.tilt[t],CP_CMD_ZERO)
                
        for _ in range(5):
            self.send_cmd((math.nan,0.0,math.nan),(math.nan,0.0,math.nan),(math.nan,0.0,math.nan), PV_CMD_VEL)
            rate.sleep()
       
    def send_cmd(self, pan,neck,tilt, cmd):


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
        
        cmd_msg = Float64MultiArray()
        #cmd_msg = Float64()

        #cmd_msg.data = -self.neck
        
        cmd_msg.data = [ 0.5*(neck[0]/1000.0-0.07), 0.5* neck[1]/1000.0,  0.5*neck[2]/1000.0, float(cmd)]
        self.neck_pub.publish(cmd_msg) 

        #cmd_msg.data = self.pan
        cmd_msg.data = [ pan[0], pan[1] , pan[2] , float(cmd)]
        self.pan_pub.publish(cmd_msg)
         
        #cmd_msg.data = self.tilt 
        cmd_msg.data = [ tilt[0], tilt[1], tilt[2], float(cmd) ]
        self.tilt_pub.publish(cmd_msg) 

        #cmd_msg_group.data = [ -self.neck, self.pan, self.tilt ]
        
        
        #self.joint_pos_pub.publish(cmd_msg_group) 
        #self.joint_vel_pub.publish(cmd_msg_group) 


        
        #self.get_logger().info('I heard: "%s"' % msg)

def main(args=None):
    
    rclpy.init(args=args)

    head_animation = HeadAnimation()

    head_animation.run()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    head_animation.destroy_node()
    rclpy.shutdown()
    if head_animation.spin_thread is not None:
        head_animation.spin_thread.join()


if __name__ == '__main__':
    main()
  
