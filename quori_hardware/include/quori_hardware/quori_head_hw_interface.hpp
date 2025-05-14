#ifndef _QUORI_HEAD_HW_INTERFACE_HPP_
#define _QUORI_HEAD_HW_INTERFACE_HPP_


#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "quori2_interface/quori2_interface.h"
//#include "quori_msgs/msg/motor_status.hpp"

namespace quori_hardware {

class QuoriHeadSystemHardware: public hardware_interface::SystemInterface {

public:

    RCLCPP_SHARED_PTR_DEFINITIONS(QuoriHeadSystemHardware)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    void mtr_sts_packet_cb(NET_PACKET & packet, QuoriStreamID stream_id);

private:

  QuoriContext context;

  std::string robot_ip_address="10.2.99.111";
  std::string ethernet_interface="eth0";

  // State
  double st_pos[3]={0.0,0.0,0.0};
  double st_vel[3]={0.0,0.0,0.0};

  double st_pos_latest[3]={0.0,0.0,0.0};
  double st_vel_latest[3]={0.0,0.0,0.0};

  // Commands
  double cmd_pos[3]={0.0,0.0,0.0};
  double cmd_vel[3]={0.0,0.0,0.0};
  double cmd_accel[3]={0.0,0.0,0.0};

  // Joint Parameters
  float initial_position[3] = { 0.0,0.0,0.0};
  float max_vel[3]={1.0,1.0,1.0};
  float max_trq[3]={3.0,3.0,3.0};
  float max_accel[3] = {NAN,NAN,NAN};
  float transmission_ratio[3] = { 1.0,1.0,1.0};
};

}; // namspace quari_hardware

#endif // _QUORI_HEAD_HW_INTERFACE_HPP_