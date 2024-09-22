
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "quori_hardware/quori_head_hw_interface.hpp"

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace quori_hardware {

static void mtr_sts_packet_cb(NET_PACKET & packet, QuoriStreamID stream_id, void * user_arg);

hardware_interface::CallbackReturn QuoriHeadSystemHardware::on_init(const hardware_interface::HardwareInfo & info) {

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  //hw_start_sec_ = hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  //hw_stop_sec_ = hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  robot_ip_address=info_.hardware_parameters["robot_ip_address"];
  ethernet_interface=info_.hardware_parameters["ethernet_interface"];

  if(info_.joints.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("QuoriHeadSystemHardware"),
        "Found %zu joints, expected 3.", info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // QuoriHeadSystemHardware has exactly two states and one command interface on each joint
    
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("QuoriHeadSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("QuoriHeadSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("QuoriHeadSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("QuoriHeadSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("QuoriHeadSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

   // double initial_position=hardware_interface::stod(joint.parameters["initial_position"]);
    //double max_velocity=hardware_interface::stod(joint.parameters["max_velocity"]);
    //double max_torque=hardware_interface::stod((const std::string)joint.parameters["max_torque"]);
    //double accel_limit=hardware_interface::stod((const std::string)joint.parameters["accel_limit"]);


  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn QuoriHeadSystemHardware::on_configure(const rclcpp_lifecycle::State &  /*previous_state*/) {

    QUORI_API_STS sts = quori_open(context,robot_ip_address.c_str(),ethernet_interface.c_str());
       
    RCLCPP_INFO( rclcpp::get_logger("QuoriHeadSystemHardware"),"Quori Hardware opened quoriLib: sts=%d",(uint32_t)sts);  
    if (sts != QUORI_API_STS::STS_OK) {
        int qerr = quori_get_errno();
        RCLCPP_INFO( rclcpp::get_logger("QuoriHeadSystemHardware"),"QuoriLib open error: %d",qerr); 
        return hardware_interface::CallbackReturn::ERROR;
    }

    sts = quori_open_stream(context,QUORI_HEAD_MTR_STS_STREAM, quori_hardware::mtr_sts_packet_cb,(void*)this);
    if (sts != QUORI_API_STS::STS_OK) {
        quori_close(context);
        int qerr = quori_get_errno();
        RCLCPP_INFO( rclcpp::get_logger("QuoriHeadSystemHardware"),"QuoriLib error in open base_mtr_stream: %d",qerr); 
        return hardware_interface::CallbackReturn::ERROR;
    }

    sts = quori_start_streaming(context);
    RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"),"Quori Node streaming started: sts=%d",(uint32_t)sts); 
    if(sts != QUORI_API_STS::STS_OK) {
        quori_close(context);
        int qerr = quori_get_errno();
        RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"),"QuoriLib start stream error: %d",qerr); 
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuoriHeadSystemHardware::on_cleanup(const rclcpp_lifecycle::State &  /*previous_state*/) {
  quori_close(context);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuoriHeadSystemHardware::on_shutdown(const rclcpp_lifecycle::State &  /*previous_state*/) {
  float pos[3] = { (float)cmd_pos[0], (float)cmd_pos[1], (float)cmd_pos[2]};
  
  quori_send_head_mtr_cmd_packet(context,pos,max_vel,max_trq,max_accel,MTR_CMD_FLAGS::MTR_CMD_STOP);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> QuoriHeadSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &st_pos[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &st_vel[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> QuoriHeadSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, (double*)&cmd_pos[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn QuoriHeadSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"), "Activating ...please wait...");

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values

    for (int i=0; i < 3; i++)
    {
      st_pos[i] = 0;
      st_vel[i] = 0;
      cmd_pos[i] = 0;
    }
  

  RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuoriHeadSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"), "Deactivating ...please wait...");

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type QuoriHeadSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /* period */)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  for (std::size_t i = 0; i < 3; i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    //hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

    RCLCPP_INFO(
      rclcpp::get_logger("QuoriHeadSystemHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'!",
       st_pos[i], st_vel[i],
       info_.joints[i].name.c_str());
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuoriHeadSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"), "Writing...");

  for (auto i = 0u; i < 3; i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("QuoriHeadSystemHardware"), "Got command %.5f for '%s'!", cmd_pos[i],
      info_.joints[i].name.c_str());

  }

  float pos[3] = { (float)cmd_pos[0], (float)cmd_pos[1], (float)cmd_pos[2]};
  
  quori_send_head_mtr_cmd_packet(context,pos,max_vel,max_trq,max_accel,MTR_CMD_FLAGS::MTR_CMD_STOP);


  RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

void QuoriHeadSystemHardware::mtr_sts_packet_cb(NET_PACKET & packet, QuoriStreamID /*stream_id */) {

  MTR_STS_PACKET & mtr_sts = packet.data.mtr_sts;

  int idx=0;

  switch(mtr_sts.mtr_id) {
    case MOTOR_ID::NECK_MOTOR:
      idx=0;
      break;

    case MOTOR_ID::PAN_MOTOR:
      idx=1;
      break;

    case MOTOR_ID::TILT_MOTOR:
      idx=2;
      break;

    default:
      return;
  }

  st_vel[idx] = mtr_sts.vel_pos[0]; 
  st_pos[idx] = mtr_sts.vel_pos[1];

}

void mtr_sts_packet_cb(NET_PACKET & packet, QuoriStreamID stream_id, void * user_arg) {
    ((QuoriHeadSystemHardware*)user_arg)->mtr_sts_packet_cb(packet,stream_id);
}

}  // namespace quori_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quori_hardware::QuoriHeadSystemHardware, hardware_interface::SystemInterface)


