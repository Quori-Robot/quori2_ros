#ifndef _QUORI_NODE_HPP_
#define _QUARI_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "quori2_interface/quori2_interface.h"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "quori_odom.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "quori_msgs/msg/motor_controller_status.hpp"
#include "quori_msgs/msg/base_vel_cmd.hpp"
#include <eigen3/Eigen/Dense>
#include "quori_node_types.hpp"

using std::placeholders::_1;

#define MK_PACKET_CB(func) [] (NET_PACKET & packet, short unsigned int stream_id, void * arg) { ((QuoriNode*)arg)->func(packet,stream_id,arg); }
class QuoriNode : public rclcpp::Node 
{ 

    private:
        QuoriContext context;

        PARAMS params;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_main_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_head_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_base_mtr_pub;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;

        rclcpp::Publisher<quori_msgs::msg::MotorControllerStatus>::SharedPtr mtr_ctrl_status_pub;

        rclcpp::Publisher<quori_msgs::msg::BaseVelCmd>::SharedPtr base_vel_cmd_pub;

        std::shared_ptr<Odom> odom_helper;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc;

        POSE pose;
        rclcpp::Time last_timestamp;

        Eigen::Matrix3d holonomic_invk;
       
        void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
            RCLCPP_DEBUG(get_logger(),"Got cmd_vel message");

            float vx=msg->linear.x;
            float vy=msg->linear.y;
            float w=msg->angular.z;

 //   #ifdef DIFFERENTIAL_DRIVE
//            float vl = vx;
  //          float vw = w;
   //         float v1 = mtr1_dir * (vl - vw * wheelbase/2.0)/wheelradius;
    //        float v2 = mtr2_dir * (vl + vw * wheelbase/2.0)/wheelradius;
//    #endif

 //   #ifdef HOLONOMIC_DRIVE

            double st = sin(-pose.th_body);
            double ct = cos(-pose.th_body);
            double rs2 = 2.0*params.xy_offset*params.wheelradius;

            holonomic_invk <<
                ct/params.wheelradius - params.wheelbase*st/rs2,    st/params.wheelradius + params.wheelbase*ct/rs2,    0.0,
                ct/params.wheelradius + params.wheelbase*st/rs2,    st/params.wheelradius - params.wheelbase*ct/rs2,    0.0,
                st/params.xy_offset,                               -ct/params.xy_offset,                                1.0;

            Eigen::Vector3d x3;
            x3 << vx,vy,w;

            Eigen::Vector3d w3 = holonomic_invk * x3;

            double v1 = w3[0];
            double v2 = w3[1];
            double v3 = w3[2];

  //  #endif
            QUORI2_INTERFACE_STS sts;

            if(params.swap_left_right) {
                sts=quori_send_base_mtr_cmd_packet(context, params.mtr1_dir*v2,params.mtr2_dir*v1, params.mtr3_dir*v3,MTR_CMD_FLAGS::MTR_CMD_SET_VEL);
            }
            else {
                sts=quori_send_base_mtr_cmd_packet(context, params.mtr1_dir*v1,params.mtr2_dir*v2, params.mtr3_dir*v3,MTR_CMD_FLAGS::MTR_CMD_SET_VEL);
            }

            if(sts != QUORI2_INTERFACE_STS::STS_OK) {
                RCLCPP_DEBUG(get_logger(),"Got send base mtr cmd Error: sts =%d",(int)sts);
            }

            quori_msgs::msg::BaseVelCmd base_vel_cmd_msg;
            base_vel_cmd_msg.header.stamp = get_clock()->now();
            base_vel_cmd_msg.header.frame_id="diff_base_link";

            base_vel_cmd_msg.velocity[0] = w3[0];
            base_vel_cmd_msg.velocity[1] = w3[1];
            base_vel_cmd_msg.velocity[2] = w3[2];

            base_vel_cmd_pub->publish(base_vel_cmd_msg);
        }


    public:

        QuoriNode() : Node("quori_node") {

            last_timestamp = {0,0};

            declare_parameter("robot_ip_address",params.lib_params.robot_ip_address);
            declare_parameter("ethernet_interface",params.lib_params.ethernet_interface);
            declare_parameter("wheelbase",params.wheelbase);
            declare_parameter("wheelradius",params.wheelradius);
            declare_parameter("publish_odom_tf",params.publish_odom_tf);
            declare_parameter("publish_wheel_joint_states",params.publish_wheel_joints);
            declare_parameter("swap_left_right",params.swap_left_right);
            declare_parameter("mtr1_dir",params.mtr1_dir);
            declare_parameter("mtr2_dir",params.mtr2_dir);
            declare_parameter("mtr3_dir",params.mtr3_dir);

            params.lib_params.robot_ip_address = get_parameter("robot_ip_address").as_string();
            params.lib_params.ethernet_interface = get_parameter("ethernet_interface").as_string();
            params.wheelbase = get_parameter("wheelbase").as_double();
            params.wheelradius = get_parameter("wheelradius").as_double();
            params.publish_odom_tf = get_parameter("publish_odom_tf").as_bool();
            params.publish_wheel_joints = get_parameter("publish_wheel_joint_states").as_bool();
            params.swap_left_right = get_parameter("swap_left_right").as_bool();
            params.mtr1_dir = get_parameter("mtr1_dir").as_int();
            params.mtr2_dir = get_parameter("mtr2_dir").as_int();
            params.mtr3_dir = get_parameter("mtr3_dir").as_int();


            RCLCPP_INFO(get_logger(),"Param: robot_ip_address=%s",params.lib_params.robot_ip_address.c_str());
            RCLCPP_INFO(get_logger(),"Param: ethernet_interface=%s",params.lib_params.ethernet_interface.c_str());
            RCLCPP_INFO(get_logger(),"Param: wheelbase=%5.2f",params.wheelbase);
            RCLCPP_INFO(get_logger(),"Param: wheelradius=%5.2f",params.wheelradius);
            RCLCPP_INFO(get_logger(),"Param: publish_odom_tf=%s",params.publish_odom_tf?"True":"False");
            RCLCPP_INFO(get_logger(),"Param: publish_wheel_joint_states=%s",params.publish_wheel_joints?"True":"False");
            RCLCPP_INFO(get_logger(),"Param: swap_left_right=%s",params.swap_left_right?"True":"False");
            RCLCPP_INFO(get_logger(),"Param: mtr1_dir=%d",params.mtr1_dir);
            RCLCPP_INFO(get_logger(),"Param: mtr2_dir=%d",params.mtr2_dir);
            RCLCPP_INFO(get_logger(),"Param: mtr3_dir=%d",params.mtr3_dir);

            QUORI2_INTERFACE_STS sts = quori_open(context,params.lib_params.robot_ip_address.c_str(),params.lib_params.ethernet_interface.c_str());
       
            RCLCPP_INFO(get_logger(),"Quori Node opened quoriLib: sts=%d",(uint32_t)sts);  
            if (sts != QUORI2_INTERFACE_STS::STS_OK) {
               int qerr = quori_get_errno();
               RCLCPP_INFO(get_logger(),"QuoriLib open error: %d",qerr); 
            }

            open_quori_stream(QUORI_BASE_MTR_STS_STREAM, MK_PACKET_CB(mtr_sts_packet_cb));
            open_quori_stream(QUORI_MTR_CTRL_STS_STREAM, MK_PACKET_CB(mtr_ctrl_sts_packet_cb));
            open_quori_stream(QUORI_IMU_STREAM, MK_PACKET_CB(imu_packet_cb));

            cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10, std::bind(&QuoriNode::cmd_vel_cb,this,_1));
            odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom_raw",10);
            imu_main_pub = create_publisher<sensor_msgs::msg::Imu>("imu_main_raw",10);
            imu_head_pub = create_publisher<sensor_msgs::msg::Imu>("imu_head_raw",10);
            imu_base_mtr_pub = create_publisher<sensor_msgs::msg::Imu>("imu_base_mtr_raw",10);

            mtr_ctrl_status_pub = create_publisher<quori_msgs::msg::MotorControllerStatus>("quori_motor_ctrl_status",10);

            joint_state_pub = create_publisher<sensor_msgs::msg::JointState>("quori_joint_states",10);
            
            base_vel_cmd_pub = create_publisher<quori_msgs::msg::BaseVelCmd>("base_vel_cmd",10);

            tf_bc = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            odom_helper = std::make_shared<Odom>(this,odom_pub,tf_bc,params,pose);


            sts = quori_start_streaming(context);
            RCLCPP_INFO(get_logger(),"Quori Node streaming started: sts=%d",(uint32_t)sts); 
            if(sts != QUORI2_INTERFACE_STS::STS_OK) {
                int qerr = quori_get_errno();
                RCLCPP_INFO(get_logger(),"QuoriLib start stream error: %d",qerr); 
            }

            RCLCPP_INFO(get_logger(),"Quori Node Init Ok");

        };

        void open_quori_stream(QuoriStreamID stream_id, QuoriStreamCallback cb) {
            QUORI2_INTERFACE_STS sts = quori_open_stream(context,stream_id,cb,(void*)this);
            RCLCPP_INFO(get_logger(),"Quori Node opened stream %u: sts=%d",stream_id,(uint32_t)sts); 
            if(sts != QUORI2_INTERFACE_STS::STS_OK) {
                int qerr = quori_get_errno();
                RCLCPP_INFO(get_logger(),"QuoriLib open stream error: %d",qerr); 
            }

        }

        void mtr_sts_packet_cb(NET_PACKET & packet, QuoriStreamID stream_id, void* arg) {
            (void)arg;
            (void)stream_id;

            if(packet.header.type == NET_PACKET_TYPE::PT_MTR_STS) {

                RCLCPP_DEBUG(get_logger(),"Got motor sts packet: mtr_id=%d",(int)packet.data.mtr_sts.mtr_id);

                if(packet.data.mtr_sts.mtr_id == MOTOR_ID::BASE_MOTOR_12) {
                    odom_helper->publish(packet);
                    if(params.publish_wheel_joints) {
                        odom_helper->publish_wheel_joint_state(joint_state_pub);
                    }
                } else {

                    double vel = packet.data.mtr_sts.vel_pos[0];
                    double pos = packet.data.mtr_sts.vel_pos[1];
                    rclcpp::Time timestamp = get_clock()->now();

                    switch(packet.data.mtr_sts.mtr_id) {
                        case MOTOR_ID::TWIST_MOTOR:

                            vel *= params.mtr3_dir;
                            pos *= params.mtr3_dir;

                            pose.th_body = pos;
                            pose.w_body = vel;

                     
                            publish_twist_joint_state(timestamp);
                            break;

                        case MOTOR_ID::NECK_MOTOR:
                            pose.pos_neck = pos;
                            pose.vel_neck = vel;

                            break;

                        case MOTOR_ID::PAN_MOTOR:
                            pose.th_pan = pos;
                            pose.w_pan = vel;

                            break;

                        case MOTOR_ID::TILT_MOTOR:
                            pose.th_tilt = pos;
                            pose.w_tilt = vel;

                            break;

                        default:
                            break;

                    }
                }

            }

        }

        void mtr_ctrl_sts_packet_cb(NET_PACKET & packet, QuoriStreamID stream_id, void* arg) {
            (void)arg;
            (void)stream_id;
            quori_msgs::msg::MotorControllerStatus msg;
            MTR_CTRL_STS_PACKET & mtr_sts = packet.data.mtr_ctrl_sts;

            if(packet.header.type == NET_PACKET_TYPE::PT_MTR_CTRL_STS) {

                RCLCPP_DEBUG(get_logger(),"Got motor ctrl sts packet: mtr_id=%d",(int)packet.data.mtr_ctrl_sts.mtr_id);

                if(mtr_sts.mtr_id == MOTOR_ID::BASE_MOTOR_12) {


                    msg.temp[0] = mtr_sts.temp[0];
                    msg.temp[1] = mtr_sts.temp[1];
                    msg.temp[2] = mtr_sts.temp[2];

                    msg.state = mtr_sts.state;

                    msg.volts[0] = mtr_sts.volts[0];
                    msg.volts[1] = mtr_sts.volts[1];
                    msg.volts[2] = mtr_sts.volts[2];

                    msg.amps[0] = mtr_sts.amps[0];
                    msg.amps[1] = mtr_sts.amps[1];

                    msg.din = mtr_sts.din;

                    msg.fm_faults[0] = mtr_sts.fm_faults[0];
                    msg.fm_faults[1] = mtr_sts.fm_faults[1];

                    msg.fs_flags = mtr_sts.fs_flags;
                    msg.ff_faults = mtr_sts.ff_faults;

                    msg.coms_error_count = mtr_sts.coms_error_count;

                    msg.controller_name = "Base_controller";

                    mtr_ctrl_status_pub->publish(msg);
                }
                else {

                    msg.temp[0] = mtr_sts.temp[0];
                 
                    msg.state = mtr_sts.state;

                    msg.volts[0] = mtr_sts.volts[0];
                 
                    msg.amps[0] = mtr_sts.amps[0];
                    msg.amps[1] = mtr_sts.amps[1];

                    msg.fm_faults[0] = mtr_sts.fm_faults[0];
                    msg.fm_faults[1] = mtr_sts.fm_faults[1];

                    msg.fs_flags = mtr_sts.fs_flags;

                    msg.ff_faults = mtr_sts.ff_faults;

                    msg.coms_error_count = mtr_sts.coms_error_count;

                    switch(mtr_sts.mtr_id) {
                        case MOTOR_ID::TWIST_MOTOR:
                            msg.controller_name = "Twist_controller";
                            mtr_ctrl_status_pub->publish(msg);;
                            break;

                        case MOTOR_ID::NECK_MOTOR:
                            msg.controller_name = "Neck_controller";
                            mtr_ctrl_status_pub->publish(msg);
                            break;

                        case MOTOR_ID::PAN_MOTOR:
                            msg.controller_name = "Pan_controller";
                            mtr_ctrl_status_pub->publish(msg);
                            break;

                        case MOTOR_ID::TILT_MOTOR:
                            msg.controller_name = "Tilt_controller";
                            mtr_ctrl_status_pub->publish(msg);
                            break;

                        default:
                            break;
                    }
                }

            }

        }

        
        void imu_packet_cb(NET_PACKET & packet, QuoriStreamID stream_id, void* arg) {
            (void)arg;

            RCLCPP_DEBUG(get_logger(),"Got imu sts packet on stream %d",stream_id);

            sensor_msgs::msg::Imu imu;
            IMU_PACKET & imu_data = packet.data.imu;

            std::array<double,9> cv= { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };

            rclcpp::Time timestamp = get_clock()->now();

            //timestamp = rclcpp::Time( imu_data.timestamp.sec, imu_data.timestamp.nanosec); 
 
            imu.header.stamp = timestamp;
     
            imu.orientation.x = imu_data.orientation[0]; 
            imu.orientation.y = imu_data.orientation[1];
            imu.orientation.z = imu_data.orientation[2];
            imu.orientation.w = imu_data.orientation[3];

            imu.orientation_covariance = cv;

            imu.angular_velocity.x = imu_data.angular_vel[0];
            imu.angular_velocity.y = imu_data.angular_vel[1];
            imu.angular_velocity.z = imu_data.angular_vel[2];

            imu.angular_velocity_covariance = cv;

            imu.linear_acceleration.x = imu_data.linear_accel[0];
            imu.linear_acceleration.y = imu_data.linear_accel[1];
            imu.linear_acceleration.z = imu_data.linear_accel[2];

            imu.linear_acceleration_covariance = cv;

            switch(imu_data.imu_id) {

                case IMU_ID::MAIN_BOARD_IMU:
                    imu.header.frame_id = "imu_main_link";
                    imu_main_pub->publish(imu);
                    break;

                case IMU_ID::HEAD_BOARD_IMU:
                    imu.header.frame_id = "imu_head_link";
                    imu_head_pub->publish(imu);
                    break;

                case IMU_ID::BASE_MOTOR_BOARD_IMU:
                    imu.header.frame_id = "imu_base_mtr_link";
                    imu_base_mtr_pub->publish(imu);
                    break;

                default:
                    break;
            }
           
        }



        void  publish_twist_joint_state(rclcpp::Time & stamp) {
   
                sensor_msgs::msg::JointState joint_state;

                joint_state.name = {"twist_joint"};
                joint_state.position = {pose.th_body};
                joint_state.velocity = {pose.w_body};

                joint_state.header.stamp = stamp;
                joint_state.header.frame_id = "base_link";

                joint_state_pub->publish(joint_state);
        };



};
#endif // _QUARI_NODE_HPP_


