#ifndef _QUORI_CMD_HPP_
#define _QUORI_CMD_HPP_

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"


#include "quori_msgs/msg/quori_ivt.hpp"
#include "quori_msgs/msg/quori_ivt.hpp"

#include "quori/quori_node_types.hpp"
#include "quori2_interface/quori2_interface.h"
using std::placeholders::_1;

#define MK_PACKET_CB(func) [] (NET_PACKET & packet, short unsigned int stream_id, void * arg) { ((QuoriIVT*)arg)->func(packet,stream_id,arg); }

class QuoriIVT : public rclcpp::Node 
{ 
    
    private:
        QuoriContext context;

        QUORI_LIB_PARAMS params;

        rclcpp::Publisher<quori_msgs::msg::QuoriIVT>::SharedPtr ivt_pub;

    public:
        QuoriIVT() : Node("quori_ivt") {

            declare_parameter("robot_ip_address",params.robot_ip_address);
            declare_parameter("ethernet_interface",params.ethernet_interface);
            
            params.robot_ip_address = get_parameter("robot_ip_address").as_string();
            params.ethernet_interface = get_parameter("ethernet_interface").as_string();
           
            RCLCPP_INFO(get_logger(),"Param: robot_ip_address=%s",params.robot_ip_address.c_str());
            RCLCPP_INFO(get_logger(),"Param: ethernet_interface=%s",params.ethernet_interface.c_str());
       
            QUORI2_INTERFACE_STS sts = quori_open(context,params.robot_ip_address.c_str(),params.ethernet_interface.c_str());
        
            RCLCPP_INFO(get_logger(),"Quori Node opened quoriLib: sts=%d",(uint32_t)sts);  
            if (sts != QUORI2_INTERFACE_STS::STS_OK) {
                int qerr = quori_get_errno();
                RCLCPP_INFO(get_logger(),"QuoriLib open error: %d",qerr); 
            }

            open_quori_stream(QUORI_IVT_STREAM, MK_PACKET_CB(ivt_packet_cb));
           
            ivt_pub = create_publisher<quori_msgs::msg::QuoriIVT>("quori_ivt",10);

            sts = quori_start_streaming(context);
            RCLCPP_INFO(get_logger(),"Quori Node streaming started: sts=%d",(uint32_t)sts); 
            if(sts != QUORI2_INTERFACE_STS::STS_OK) {
                int qerr = quori_get_errno();
                RCLCPP_INFO(get_logger(),"QuoriLib start stream error: %d",qerr); 
            }

            RCLCPP_INFO(get_logger(),"Quori IVT Init Ok");

        
        };


    private:

        void open_quori_stream(QuoriStreamID stream_id, QuoriStreamCallback cb) {
            QUORI2_INTERFACE_STS sts = quori_open_stream(context,stream_id,cb,(void*)this);
            RCLCPP_INFO(get_logger(),"Quori Node opened stream %u: sts=%d",stream_id,(uint32_t)sts); 
            if(sts != QUORI2_INTERFACE_STS::STS_OK) {
                int qerr = quori_get_errno();
                RCLCPP_INFO(get_logger(),"QuoriLib open stream error: %d",qerr); 
             }
        }


        void ivt_packet_cb(NET_PACKET & packet, QuoriStreamID /* stream_id */, void* /*arg */) {
            RCLCPP_INFO(get_logger(),"QuoriIVT Got %d packet.",(int)packet.header.type);
          
         
            if(packet.header.type == NET_PACKET_TYPE::PT_IVT) {

                IVT_PACKET & pkt = packet.data.ivt;
                quori_msgs::msg::QuoriIVT msg;

                msg.num_values = pkt.num_values;

                for(int ivt=0; ivt < pkt.num_values; ivt++) {
                    msg.ivt_id[ivt] = (uint32_t)pkt.ivt[ivt].ivt_id;
                    msg.value[ivt] = pkt.ivt[ivt].value;
                    //RCLCPP_INFO(get_logger(),"QuoriIVT ID=%d, value=%f.\n",pkt.ivt[ivt].ivt_id,pkt.ivt[ivt].value);
                }
               
                ivt_pub->publish(msg);
            }

            
        }


};


#endif //  _QUORI_CMD_HPP_
