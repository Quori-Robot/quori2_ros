#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "quorilib/quori_lib.h"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "quori_odom.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "quori_msgs/msg/motor_status.hpp"

using std::placeholders::_1;

#define MK_PACKET_CB(func) [] (NET_PACKET & packet, short unsigned int stream_id, void * arg) { ((QuoriNode*)arg)->func(packet,stream_id,arg); }
class QuoriNode : public rclcpp::Node 
{ 

    private:
        QuoriContext context;

        std::string robot_ip_address="10.2.99.111";
        std::string ethernet_interface="eth0";

        float wheelbase=0.310;
        float wheelradius=0.065;

        bool publish_odom_tf=true;
        bool publish_wheel_joints=true;
        bool swap_left_right=false;

        float bot_x=0.0;
        float bot_y=0.0;
        float bot_w=0.0;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_main_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_head_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_base_mtr_pub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joint_pub;
        rclcpp::Publisher<quori_msgs::msg::MotorStatus>::SharedPtr mtr_status_pub;
        
        std::shared_ptr<Odom> odom_helper;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc;
       
        void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
            //RCLCPP_INFO(get_logger(),"Got cmd_vel message");

            float v1;
            float v2;

            v1 =  (msg->linear.x - msg->angular.z * wheelbase/2.0)/wheelradius;
            v2 =  (msg->linear.x + msg->angular.z * wheelbase/2.0)/wheelradius;

            if(swap_left_right) {
                quori_send_mtr_cmd_packet(context,v2,v1,MTR_CMD_FLAGS::MTR_CMD_SET_VEL);
            }
            else {
                quori_send_mtr_cmd_packet(context,v1,v2,MTR_CMD_FLAGS::MTR_CMD_SET_VEL);
            }
        }


    public:

        QuoriNode() : Node("quori_node") {

            declare_parameter("robot_ip_address",robot_ip_address);
            declare_parameter("ethernet_interface",ethernet_interface);
            declare_parameter("wheelbase",wheelbase);
            declare_parameter("wheelradius",wheelradius);
            declare_parameter("publish_odom_tf",publish_odom_tf);
            declare_parameter("publish_wheel_joint_states",publish_wheel_joints);
            declare_parameter("swap_left_right",swap_left_right);

            robot_ip_address = get_parameter("robot_ip_address").as_string();
            ethernet_interface = get_parameter("ethernet_interface").as_string();
            wheelbase = get_parameter("wheelbase").as_double();
            wheelradius = get_parameter("wheelradius").as_double();
            publish_odom_tf = get_parameter("publish_odom_tf").as_bool();
            publish_wheel_joints = get_parameter("publish_wheel_joint_states").as_bool();
            swap_left_right = get_parameter("swap_left_right").as_bool();


            RCLCPP_INFO(get_logger(),"Param: robot_ip_address=%s",robot_ip_address.c_str());
            RCLCPP_INFO(get_logger(),"Param: ethernet_interface=%s",ethernet_interface.c_str());
            RCLCPP_INFO(get_logger(),"Param: wheelbase=%5.2f",wheelbase);
            RCLCPP_INFO(get_logger(),"Param: wheelradius=%5.2f",wheelradius);
            RCLCPP_INFO(get_logger(),"Param: publish_odom_tf=%s",publish_odom_tf?"True":"False");
            RCLCPP_INFO(get_logger(),"Param: publish_wheel_joint_states=%s",publish_wheel_joints?"True":"False");
            RCLCPP_INFO(get_logger(),"Param: swap_left_right=%s",swap_left_right?"True":"False");

            QUORI_API_STS sts = quori_open(context,robot_ip_address.c_str(),ethernet_interface.c_str());
       
            RCLCPP_INFO(get_logger(),"Quori Node opened quoriLib: sts=%d",(uint32_t)sts);  
            if (sts != QUORI_API_STS::STS_OK) {
               int qerr = quori_get_errno();
               RCLCPP_INFO(get_logger(),"QuoriLib open error: %d",qerr); 
            }

            open_stream(QUORI_MTR_STS_STREAM, MK_PACKET_CB(mtr_sts_packet_cb));
            open_stream(QUORI_IMU_STREAM, MK_PACKET_CB(imu_packet_cb));
            open_stream(QUORI_IVT_STREAM, MK_PACKET_CB(ivt_packet_cb));

     

            cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10, std::bind(&QuoriNode::cmd_vel_cb,this,_1));
            odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom_raw",10);
            imu_main_pub = create_publisher<sensor_msgs::msg::Imu>("imu_main_raw",10);
            imu_head_pub = create_publisher<sensor_msgs::msg::Imu>("imu_head_raw",10);
            imu_base_mtr_pub = create_publisher<sensor_msgs::msg::Imu>("imu_base_mtr_raw",10);
            mtr_status_pub = create_publisher<quori_msgs::msg::MotorStatus>("motor_status",10);

            if(publish_wheel_joints) {
                wheel_joint_pub = create_publisher<sensor_msgs::msg::JointState>("base_wheel_joints",10);
            }
            

            tf_bc = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            odom_helper = std::make_shared<Odom>(this,odom_pub,tf_bc,wheelbase,wheelradius,publish_odom_tf,swap_left_right);


            sts = quori_start_streaming(context);
            RCLCPP_INFO(get_logger(),"Quori Node streaming started: sts=%d",(uint32_t)sts); 
            if(sts != QUORI_API_STS::STS_OK) {
                int qerr = quori_get_errno();
                RCLCPP_INFO(get_logger(),"QuoriLib start stream error: %d",qerr); 
            }

            RCLCPP_INFO(get_logger(),"Quori Node Init Ok");

        };

        void open_stream(QuoriStreamID stream_id, QuoriStreamCallback cb) {
            QUORI_API_STS sts = quori_open_stream(context,stream_id,cb,(void*)this);
            RCLCPP_INFO(get_logger(),"Quori Node opened stream %u: sts=%d",stream_id,(uint32_t)sts); 
            if(sts != QUORI_API_STS::STS_OK) {
                int qerr = quori_get_errno();
                RCLCPP_INFO(get_logger(),"QuoriLib open stream error: %d",qerr); 
            }

        }

        void mtr_sts_packet_cb(NET_PACKET & packet, QuoriStreamID stream_id, void* arg) {
            (void)arg;
            (void)stream_id;

            if(packet.header.type == NET_PACKET_TYPE::PT_MTR_STS) {

                RCLCPP_DEBUG(get_logger(),"Got motor sts packet");
                odom_helper->publish(packet);
                if(publish_wheel_joints) {
                    odom_helper->publish_wheel_joint_state(wheel_joint_pub);
                }

                quori_msgs::msg::MotorStatus msg;
                MTR_STS_PACKET & mtr_sts = packet.data.mtr_sts;

                msg.temp[0] = mtr_sts.temp[0];
                msg.temp[1] = mtr_sts.temp[1];
                msg.temp[2] = mtr_sts.temp[2];

                msg.state = mtr_sts.state;

                msg.volts[0] = mtr_sts.volts[0];
                msg.volts[1] = mtr_sts.volts[1];
                msg.volts[2] = mtr_sts.volts[2];

                msg.amps[0] = mtr_sts.amps[0];
                msg.amps[1] = mtr_sts.amps[1];

                msg.encoder_count[0] = mtr_sts.encoder_count[0];
                msg.encoder_count[1] = mtr_sts.encoder_count[1];

                msg.vel[0] = mtr_sts.vel[0];
                msg.vel[1] = mtr_sts.vel[1];

                msg.din = mtr_sts.din;

                msg.fm_faults[0] = mtr_sts.fm_faults[0];
                msg.fm_faults[1] = mtr_sts.fm_faults[1];

                msg.fs_flags = mtr_sts.fs_flags;
                msg.ff_faults = mtr_sts.ff_faults;

                msg.coms_error_count = mtr_sts.coms_error_count;

                mtr_status_pub->publish(msg);

            }

        }

        
        void imu_packet_cb(NET_PACKET & packet, QuoriStreamID stream_id, void* arg) {
            (void)arg;

            RCLCPP_DEBUG(get_logger(),"Got imu sts packet on stream %d",stream_id);

            sensor_msgs::msg::Imu imu;
            IMU_PACKET & imu_data = packet.data.imu;

            std::array<double,9> cv= { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };

            imu.header.stamp.sec = imu_data.timestamp.sec;
            imu.header.stamp.nanosec = imu_data.timestamp.nanosec;

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
                    imu.header.frame_id = "imu_main";
                    imu_main_pub->publish(imu);
                    break;

                case IMU_ID::HEAD_BOARD_IMU:
                    imu.header.frame_id = "imu_head";
                    imu_head_pub->publish(imu);
                    break;

                case IMU_ID::BASE_MOTOR_BOARD_IMU:
                    imu.header.frame_id = "imu_base_mtr";
                    imu_base_mtr_pub->publish(imu);
                    break;

                default:
                    break;
            }
           
        }

        void ivt_packet_cb(NET_PACKET & packet, QuoriStreamID stream_id, void* arg) {
            (void)arg;
            (void)packet;

            RCLCPP_DEBUG(get_logger(),"Got ivt sts packet on stream %d",stream_id);
           // nav_msgs::msg::Odometry odom;

            //odom_pub->publish(odom);
        }

};



