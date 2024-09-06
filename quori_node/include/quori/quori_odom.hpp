#ifndef __QUARI_ODOM_H__
#define __QUARI_ODOM_H__

#include "quorilib/quori_lib.h"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class Odom {

    private:
        rclcpp::Node* node;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc;

        float wheelbase;
        float wheelradius;

        bool publish_tf=false;
        bool swap_left_right=false;

        rclcpp::Time last_stamp = node->get_clock()->now();

        double ja1 = 0.0;
        double ja2 = 0.0;

        double w1=0.0;
        double w2=0.0;

        double dt=0.0;
        double last_time=0.0;

        float bot_x=0.0;
        float bot_y=0.0;
        float bot_w=0.0;

    public:
        Odom( rclcpp::Node* node, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
         std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc, float wheelbase , float wheelradius, bool publish_tf, bool swap_left_right) : 

                    node(node), odom_pub(odom_pub), tf_bc(tf_bc), wheelbase(wheelbase), wheelradius(wheelradius), publish_tf(publish_tf),
                    swap_left_right(swap_left_right) {
           
        }

        void publish_wheel_joint_state(rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub) {
   
                sensor_msgs::msg::JointState wheel_joint_states;

                wheel_joint_states.name = {"left_wheel_joint", "right_wheel_joint"};
                wheel_joint_states.position = {ja1,ja2};
                wheel_joint_states.velocity = {w1,w2};

                wheel_joint_states.header.stamp = last_stamp;
                wheel_joint_states.header.frame_id = "base_link";

                joint_pub->publish(wheel_joint_states);
        };


        void publish(NET_PACKET & packet) {

            RCLCPP_DEBUG(node->get_logger(),"Publish Odom");

            nav_msgs::msg::Odometry odom;

            MTR_STS_PACKET & mtr_sts = packet.data.mtr_sts;

            rclcpp::Time timestamp = node->get_clock()->now();

            //timestamp = rclcpp::Time( mtr_sts.timestamp.sec, mtr_sts.timestamp.nanosec); 
    
            odom.header.stamp= timestamp; 
            last_stamp = timestamp;
        
            double time = timestamp.seconds();
            if(dt == 0.0 && last_time==0.0) {
                last_time = time;
                return;
            }

            dt = time-last_time;
            last_time = time;

            //RCLCPP_DEBUG(node->get_logger(),"Got timestamp");

            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_footprint";

            if(swap_left_right) {
                w1=mtr_sts.vel[1];
                w2=mtr_sts.vel[0];
            }
            else {
                w1=mtr_sts.vel[0];
                w2=mtr_sts.vel[1];
            }

            ja1 += w1 * dt;
            ja2 += w2 * dt;

            while(ja1 > M_PI) {
                ja1 -= 2.0 *M_PI;
            }
            while(ja1 < M_PI) {
                ja1 += 2.0 * M_PI;
            }
            while(ja2 > M_PI) {
                ja2 -= 2.0 *M_PI;
            }
            while(ja2 < M_PI) {
                ja2 += 2.0 * M_PI;
            }


            w1 = wheelradius * w1;
            w2 = wheelradius * w2;

            //RCLCPP_INFO(node->get_logger(),"Vel: %f,%f",w1,w2);

            float vel = (w1 + w2)/2.0;
            float w   = (w2 - w1)/wheelbase;

            bot_w += w * dt;

            bot_x += vel * cos(bot_w) * dt;
            bot_y += vel * sin(bot_w) * dt;

            

            odom.pose.pose.position.x = bot_x;
            odom.pose.pose.position.y = bot_y;
            odom.pose.pose.position.z = 0;

            tf2::Quaternion q;
            q.setRPY(0.0,0.0,bot_w);

            //RCLCPP_DEBUG(node->get_logger(),"Got Quaternion");

            odom.pose.pose.orientation.x = q.getX();
            odom.pose.pose.orientation.y = q.getY();
            odom.pose.pose.orientation.z = q.getZ();
            odom.pose.pose.orientation.w = q.getW();

            odom.twist.twist.linear.x = vel;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.linear.z = 0.0;

            odom.twist.twist.angular.x = 0.0;
            odom.twist.twist.angular.y = 0.0;
            odom.twist.twist.angular.z = w;

            //RCLCPP_DEBUG(node->get_logger(),"Done Pose");

            std::array<double,36> cv =
            { 
                0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0001, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.001
            };

            //RCLCPP_DEBUG(node->get_logger(),"Do CV");

            odom.pose.covariance = cv;
            odom.twist.covariance = cv;

            //RCLCPP_DEBUG(node->get_logger(),"CV Ok");

            odom_pub->publish(odom);

            //RCLCPP_DEBUG(node->get_logger(),"Publish odom ok");
            
            if(publish_tf) {
                geometry_msgs::msg::TransformStamped tf;

                tf.header.frame_id = "odom";
                tf.child_frame_id = "base_footprint";

                tf.transform.translation.x = bot_x;
                tf.transform.translation.y = bot_y;
                tf.transform.translation.z = 0.0;

                tf.transform.rotation.x = q.getX();
                tf.transform.rotation.y = q.getY();
                tf.transform.rotation.z = q.getZ();
                tf.transform.rotation.w = q.getW();

                tf.header.stamp = timestamp;

                //RCLCPP_DEBUG(node->get_logger(),"TF ready");

                tf_bc->sendTransform(tf);

                //RCLCPP_DEBUG(node->get_logger(),"TF Ok");
            }

          
        }

};

#endif // __QUARI_ODOM_H__
