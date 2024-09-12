#ifndef _QUORI_NODE_TYPES_HPP_
#define _QUORI_NODE_TYPES_HPP_
#include <string>

typedef struct {
    float x=0.0;  // world X
    float y=0.0;  // world Y
    float th=0.0; // world theta
    float th_body=0.0; // rotation angle of body relative to base
    float w_body=0.0;  // body rotation speed relative to base
} POSE;

typedef struct {
    std::string robot_ip_address="10.2.99.111";
    std::string ethernet_interface="eth0";

    float wheelbase=0.310;
    float wheelradius=0.065;
    float xy_offset=0.100;

    bool publish_odom_tf=true;
    bool publish_wheel_joints=true;
    bool swap_left_right=false;
    int mtr1_dir =1;
    int mtr2_dir =1;
} PARAMS;

#endif // _QUORI_NODE_TYPES_HPP_