#ifndef _QUORI_CMD_HPP_
#define _QUORI_CMD_HPP_

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <future>

#include "quori_msgs/msg/quori_cmd.hpp"
#include "quori_msgs/msg/quori_version_info.hpp"
#include "quori_msgs/msg/switch_status.hpp"
#include "quori_msgs/msg/quori_health_status.hpp"
#include "std_msgs/msg/string.hpp"

#include "quori/quori_node_types.hpp"
#include "quori2_interface/quori2_interface.h"
using std::placeholders::_1;

#define PING_RESP_TIMEOUT 500ms

#define MK_PACKET_CB(func) [] (NET_PACKET & packet, short unsigned int stream_id, void * arg) { ((QuoriCmd*)arg)->func(packet,stream_id,arg); }

using namespace std::chrono_literals;

class QuoriCmd : public rclcpp::Node 
{ 
    
    private:
        QuoriContext context;

        QUORI_LIB_PARAMS params;

        rclcpp::Subscription<quori_msgs::msg::QuoriCmd>::SharedPtr cmd_sub;
        rclcpp::Publisher<quori_msgs::msg::QuoriVersionInfo>::SharedPtr version_info_pub;
        rclcpp::Publisher<quori_msgs::msg::SwitchStatus>::SharedPtr switch_sts_pub;
        rclcpp::Publisher<quori_msgs::msg::QuoriHealthStatus>::SharedPtr health_sts_pub;

        rclcpp::TimerBase::SharedPtr ping_timer;
        rclcpp::TimerBase::SharedPtr ping_resp_timer;

        bool _shutdown = false;
        uint8_t ping_seq = 0;
        bool got_ping_reply=false;
        bool use_sim_time=false;

    public:
        QuoriCmd() : Node("quori_cmd") {

            declare_parameter("robot_ip_address",params.robot_ip_address);
            declare_parameter("ethernet_interface",params.ethernet_interface);
            
            params.robot_ip_address = get_parameter("robot_ip_address").as_string();
            params.ethernet_interface = get_parameter("ethernet_interface").as_string();
            use_sim_time = get_parameter("use_sim_time").as_bool();
           
            RCLCPP_INFO(get_logger(),"Param: robot_ip_address=%s",params.robot_ip_address.c_str());
            RCLCPP_INFO(get_logger(),"Param: ethernet_interface=%s",params.ethernet_interface.c_str());


            QUORI2_INTERFACE_STS sts = quori_open(context,params.robot_ip_address.c_str(),params.ethernet_interface.c_str());
        
            RCLCPP_INFO(get_logger(),"Quori Node opened quoriLib: sts=%d",(uint32_t)sts);  
            if (sts != QUORI2_INTERFACE_STS::STS_OK) {
                int qerr = quori_get_errno();
                RCLCPP_INFO(get_logger(),"QuoriLib open error: %d",qerr); 
            }

            open_quori_stream(QUORI_MAIN_COMS_STS_STREAM, MK_PACKET_CB(main_coms_sts_packet_cb));

            cmd_sub = create_subscription<quori_msgs::msg::QuoriCmd>("quori_cmd",10, std::bind(&QuoriCmd::cmd_cb,this,_1));
            version_info_pub = create_publisher<quori_msgs::msg::QuoriVersionInfo>("quori_version_info",10);
            switch_sts_pub = create_publisher<quori_msgs::msg::SwitchStatus>("quori_switch_status",10);
            health_sts_pub = create_publisher<quori_msgs::msg::QuoriHealthStatus>("quori_health_status",10);

            sts = quori_start_streaming(context);
            RCLCPP_INFO(get_logger(),"Quori Node streaming started: sts=%d",(uint32_t)sts); 
            if(sts != QUORI2_INTERFACE_STS::STS_OK) {
                int qerr = quori_get_errno();
                RCLCPP_INFO(get_logger(),"QuoriLib start stream error: %d",qerr); 
            }

            if(!use_sim_time) {
                ping_timer = this->create_wall_timer(500ms,std::bind(&QuoriCmd::ping_timer_cb,this));
                ping_resp_timer = this->create_wall_timer(PING_RESP_TIMEOUT,std::bind(&QuoriCmd::ping_resp_timer_cb,this));
                ping_resp_timer->cancel();
            }


            RCLCPP_INFO(get_logger(),"Quori Cmd Init Ok");

        
        };

        void close(void) {
            quori_stop_streaming(context);
            quori_close(context);
        }

        inline bool shutdown(void) { return _shutdown; };

        void open_quori_stream(QuoriStreamID stream_id, QuoriStreamCallback cb) {
            QUORI2_INTERFACE_STS sts = quori_open_stream(context,stream_id,cb,(void*)this);
            RCLCPP_INFO(get_logger(),"Quori Node opened stream %u: sts=%d",stream_id,(uint32_t)sts); 
            if(sts != QUORI2_INTERFACE_STS::STS_OK) {
                int qerr = quori_get_errno();
                RCLCPP_INFO(get_logger(),"QuoriLib open stream error: %d",qerr); 
             }
        }

        void cmd_cb(const quori_msgs::msg::QuoriCmd::SharedPtr msg) {
            RCLCPP_INFO(get_logger(),"Got quori_cmd message");

            quori_send_quori_cmd_packet(context,(QUORI_CMD)(msg->cmd),msg->arg1,msg->arg2);
        }


        void main_coms_sts_packet_cb(NET_PACKET & packet, QuoriStreamID /* stream_id */, void* /*arg */) {
            //RCLCPP_INFO(get_logger(),"QuoriCmd Got %d packet.",(int)packet.header.type);
          
         
            if(packet.header.type == NET_PACKET_TYPE::PT_VERSION_INFO) {

                VERSION_INFO_PACKET & pkt = packet.data.version_info;
                quori_msgs::msg::QuoriVersionInfo msg;

              
                for(int pcb=0; pcb < VERSION_INFO_NUM_PCB; pcb++) {
                    msg.pcb_info[pcb].major = pkt.pcb_info[pcb].major;
                    msg.pcb_info[pcb].minor = pkt.pcb_info[pcb].minor;
                    msg.pcb_info[pcb].revision = pkt.pcb_info[pcb].revision;
                    msg.pcb_info[pcb].dirty = pkt.pcb_info[pcb].dirty;
                    msg.pcb_info[pcb].debug = pkt.pcb_info[pcb].debug;
                    msg.pcb_info[pcb].untagged_commit = pkt.pcb_info[pcb].untagged_commit;
                    msg.pcb_info[pcb].commit_number = pkt.pcb_info[pcb].commit_number;
                    msg.pcb_info[pcb].pcb_rev = pkt.pcb_info[pcb].pcb_rev;
                    msg.pcb_info[pcb].stm_dev_id[0]=pkt.pcb_info[pcb].stm_dev_id[0];
                    msg.pcb_info[pcb].stm_dev_id[1]=pkt.pcb_info[pcb].stm_dev_id[1];
                    msg.pcb_info[pcb].stm_uuid[0]=pkt.pcb_info[pcb].stm_uuid[0];
                    msg.pcb_info[pcb].stm_uuid[1]=pkt.pcb_info[pcb].stm_uuid[1];
                    msg.pcb_info[pcb].stm_uuid[2]=pkt.pcb_info[pcb].stm_uuid[2];
                }
                for(int mts=0; mts < VERSION_INFO_NUM_MOTEUS_CTRL; mts++) {
                    msg.moteus_info[mts].sn[0] = pkt.moteus_info[mts].sn[0];
                    msg.moteus_info[mts].sn[1] = pkt.moteus_info[mts].sn[1];
                    msg.moteus_info[mts].sn[2] = pkt.moteus_info[mts].sn[2];

                    msg.moteus_info[mts].fw_version = pkt.moteus_info[mts].fw_version;
                    msg.moteus_info[mts].register_map_version = pkt.moteus_info[mts].register_map_version;
                    msg.moteus_info[mts].model = pkt.moteus_info[mts].model;
                }
                msg.rbtq_firmware_version = std::string(pkt.rbtq_info.rbtq_firmware_version);
                msg.controller_type = std::string(pkt.rbtq_info.controller_type);

                msg.version_valid_flags = pkt.version_valid_flags;
                version_info_pub->publish(msg);
            }

            if(packet.header.type == NET_PACKET_TYPE::PT_SWITCH_STATUS) {

                SWITCH_STS_PACKET & pkt = packet.data.sw_sts;
                quori_msgs::msg::SwitchStatus msg;

                msg.state = pkt.state;
                msg.changed = pkt.changed;

                switch_sts_pub->publish(msg);

            }

            if(packet.header.type == NET_PACKET_TYPE::PT_SHUTDOWN) {
               RCLCPP_INFO(get_logger(),"Quori Cmd Shutdown Requested");
               _shutdown = true;
               //RCLCPP_INFO(get_logger(),"Quori Cmd Shutdown Request done");
            }

            
            if(packet.header.type == NET_PACKET_TYPE::PT_PING) {
                ping_resp_timer->cancel();
                //RCLCPP_INFO(get_logger(),"Quori Cmd got Ping Response");
                got_ping_reply=true;
            }

            if(packet.header.type == NET_PACKET_TYPE::PT_HEALTH_STATUS) {
                  
                quori_msgs::msg::QuoriHealthStatus msg;
                msg.status_flags = (uint32_t) packet.data.health_status.status;
                if(got_ping_reply) {
                    msg.status_flags |= (uint32_t) HealthFlags::HF_BASE_BOARD_OK;
                }
                else {
                    msg.status_flags &= ~(uint32_t)HealthFlags::HF_BASE_BOARD_OK;
                }
                health_sts_pub->publish(msg);
            }
        }

        void ping_timer_cb(void) {
            ping_resp_timer->reset();
            quori_send_quori_ping_packet(context,ping_seq);
        }

        
        void ping_resp_timer_cb(void) {
            ping_resp_timer->cancel();
            RCLCPP_INFO(get_logger(),"Quori Cmd Missed Ping Response");
            got_ping_reply=false;
            quori_msgs::msg::QuoriHealthStatus msg;
            msg.status_flags = (uint32_t) HealthFlags::HF_ORIN_OK;
            health_sts_pub->publish(msg);

        }


};


#endif //  _QUORI_CMD_HPP_
