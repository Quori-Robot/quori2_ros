#ifndef _QUORI_NODE_TYPES_HPP_
#define _QUORI_NODE_TYPES_HPP_
#include <string>

#define HOLONOMIC_DRIVE
//#define DIFFERENTIAL_DRIVE

typedef struct {
    float x=0.0;  // world X
    float y=0.0;  // world Y
    float th=0.0; // world theta of base

    float w1=0.0;  // wheel 1 rotation rate
    float w2=0.0;  // wheel 2 rotation rate 
    float w=0.0;   // world rotation rate of base

    float th_body=0.0; // rotation angle of body relative to base
    float w_body=0.0;  // body rotation speed relative to base

    float pos_neck=0.0; // linear position of neck relative to body
    float vel_neck=0.0; // velocity of neck relative body

    float th_pan=0.0; // rotation angle of pan joint relative to neck
    float w_pan=0.0;  // pan rotation speed relative to neck

    float th_tilt=0.0; // rotation angle of tilt joint relative to neck
    float w_tilt=0.0;  // tilt rotation speed relative to neck

}  POSE;

typedef struct {
    std::string robot_ip_address="10.2.99.124";
    std::string ethernet_interface="eth0";
} QUORI_LIB_PARAMS;

typedef struct {

    QUORI_LIB_PARAMS lib_params;
    float wheelbase=0.3225;
    float wheelradius=0.065;
    float xy_offset=0.117935;

    bool publish_odom_tf=true;
    bool publish_wheel_joints=true;
    bool swap_left_right=false;
    
    int mtr1_dir =1;
    int mtr2_dir =1;
    int mtr3_dir =1;

    bool use_sim_time=false;

} PARAMS;

#endif // _QUORI_NODE_TYPES_HPP_