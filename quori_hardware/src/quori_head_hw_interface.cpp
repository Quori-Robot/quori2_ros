
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

  robot_ip_address=info_.hardware_parameters["robot_ip_address"];
  ethernet_interface=info_.hardware_parameters["ethernet_interface"];

  if(info_.joints.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("QuoriHeadSystemHardware"),
        "Found %zu joints, expected 3.", info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
  }

  for (hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // QuoriHeadSystemHardware has exactly two states and one command interface on each joint
    
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("QuoriHeadSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 3 expected.", joint.name.c_str(),
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

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("QuoriHeadSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[2].name != hardware_interface::HW_IF_ACCELERATION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("QuoriHeadSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_ACCELERATION);
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

    std::string ip=joint.parameters["initial_position"];
    std::string mv=joint.parameters["max_velocity"];
    std::string mt=joint.parameters["max_torque"];
    std::string ma=joint.parameters["max_accel"];
    std::string transmission_ratio=joint.parameters["transmission_ratio"];

    RCLCPP_INFO(
        rclcpp::get_logger("QuoriHeadSystemHardware"), "Joint '%s' params-> init pos=%s, max vel=%s, max_trq=%s, max_accel=%s, ratio=%s",joint.name.c_str(),
        ip.c_str(),mv.c_str(),mt.c_str(),ma.c_str(),transmission_ratio.c_str());
    
    double initial_position= ip=="NAN"?NAN:hardware_interface::stod(ip);
    double max_vel  = mv=="NAN"?NAN:hardware_interface::stod(mv);
    double max_trq  = mt=="NAN"?NAN:hardware_interface::stod(mt);
    double max_accel= ma=="NAN"?NAN:hardware_interface::stod(ma);
    double transmission= transmission_ratio=="NAN"?NAN:hardware_interface::stod(transmission_ratio);

    int idx=0;
    if(joint.name == "neck_joint") {
       idx=0;
    }

    if(joint.name == "pan_joint") {
       idx=1;
    }

    if(joint.name == "tilt_joint") {
       idx=2;
    }

    this->initial_position[idx]=initial_position;
    this->max_vel[idx]=max_vel;
    this->max_trq[idx]=max_trq;
    this->max_accel[idx]=max_accel;
    this->transmission_ratio[idx]=transmission;

  }


  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn QuoriHeadSystemHardware::on_configure(const rclcpp_lifecycle::State &  /*previous_state*/) {

    QUORI2_INTERFACE_STS sts = quori_open(context,robot_ip_address.c_str(),ethernet_interface.c_str());
       
    RCLCPP_INFO( rclcpp::get_logger("QuoriHeadSystemHardware"),"Quori Hardware opened quoriLib: sts=%d",(uint32_t)sts);  
    if (sts != QUORI2_INTERFACE_STS::STS_OK) {
        int qerr = quori_get_errno();
        RCLCPP_INFO( rclcpp::get_logger("QuoriHeadSystemHardware"),"QuoriLib open error: %d",qerr); 
        return hardware_interface::CallbackReturn::ERROR;
    }

    sts = quori_open_stream(context,QUORI_HEAD_MTR_STS_STREAM, quori_hardware::mtr_sts_packet_cb,(void*)this);
    if (sts != QUORI2_INTERFACE_STS::STS_OK) {
        quori_close(context);
        int qerr = quori_get_errno();
        RCLCPP_INFO( rclcpp::get_logger("QuoriHeadSystemHardware"),"QuoriLib error in open base_mtr_stream: %d",qerr); 
        return hardware_interface::CallbackReturn::ERROR;
    }

    sts = quori_start_streaming(context);
    RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"),"Quori Node streaming started: sts=%d",(uint32_t)sts); 
    if(sts != QUORI2_INTERFACE_STS::STS_OK) {
        quori_close(context);
        int qerr = quori_get_errno();
        RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"),"QuoriLib start stream error: %d",qerr); 
        return hardware_interface::CallbackReturn::ERROR;
    }
    float pos[3] = { (float)cmd_pos[0], (float)cmd_pos[1], (float)cmd_pos[2]};
    quori_send_head_mtr_cmd_packet(context,pos,max_vel,max_trq,max_accel,MTR_CMD_FLAGS::MTR_CMD_STOP);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuoriHeadSystemHardware::on_cleanup(const rclcpp_lifecycle::State &  /*previous_state*/) {
  float pos[3] = { (float)cmd_pos[0], (float)cmd_pos[1], (float)cmd_pos[2]};
  quori_send_head_mtr_cmd_packet(context,pos,max_vel,max_trq,max_accel,MTR_CMD_FLAGS::MTR_CMD_STOP);

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
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, (double*)&cmd_vel[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, (double*)&cmd_accel[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn QuoriHeadSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"), "Activating ...please wait...");

  // set default values

  for (int i=0; i < 3; i++)
  {
    st_pos[i] = NAN;
    st_vel[i] = NAN;

    cmd_pos[i] = NAN;
    cmd_vel[i] = NAN;
    cmd_accel[i] = NAN;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuoriHeadSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"), "Deactivating ...please wait...");


  RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type QuoriHeadSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /* period */)
{

  for(int i=0; i< 3; i++) {
    st_vel[i]=st_vel_latest[i];
    st_pos[i]=st_pos_latest[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuoriHeadSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger("QuoriHeadSystemHardware"), "Writing...");

  if( 
      std::isnan(cmd_pos[0]) ||  std::isnan(cmd_pos[1]) ||  std::isnan(cmd_pos[2]) ||
      //std::isnan(cmd_vel[0]) ||  std::isnan(cmd_vel[1]) ||  std::isnan(cmd_vel[2]) || 
      std::isnan(st_vel[0])  ||  std::isnan(st_vel[1])  ||  std::isnan(st_vel[2])  ||  
      std::isnan(st_pos[0])  ||  std::isnan(st_pos[1])  ||  std::isnan(st_pos[2]) ) {
      return hardware_interface::return_type::OK;
  }


  RCLCPP_DEBUG(rclcpp::get_logger("QuoriHeadSystemHardware"), "Writing Pos=%f, Writing Vel=%f",(float)cmd_pos[0], (float)cmd_vel[0]);
  float pos[3] ={ (float)cmd_pos[0]* transmission_ratio[0], 
                  (float)cmd_pos[1]* transmission_ratio[1],
                  (float)cmd_pos[2]* transmission_ratio[2]};

  float vel[3] = { (float)cmd_vel[0] * transmission_ratio[0], 
                   (float)cmd_vel[1] * transmission_ratio[1],
                   (float)cmd_vel[2] * transmission_ratio[2]
                  };
  float accel[3] = { (float)max_accel[0], (float)max_accel[1], (float)max_accel[2]};
  
  quori_send_head_mtr_cmd_packet(context,pos,vel,max_trq,accel,MTR_CMD_FLAGS::MTR_CMD_SET_POS);

  return hardware_interface::return_type::OK;
}


void QuoriHeadSystemHardware::mtr_sts_packet_cb(NET_PACKET & packet, QuoriStreamID /*stream_id */) {

  MTR_STS_PACKET & mtr_sts = packet.data.mtr_sts;

  switch(mtr_sts.mtr_id) {

      case MOTOR_ID::NECK_MOTOR:
          st_vel_latest[0] = mtr_sts.vel_pos[0] / transmission_ratio[0]; 
          st_pos_latest[0] = mtr_sts.vel_pos[1] / transmission_ratio[0];
          //RCLCPP_INFO(rclcpp::get_logger("QuoriHeadSystemHardware"), "Got Vel=%f", (float)st_vel_latest[0]);
          break;

      case MOTOR_ID::PAN_MOTOR:
          st_vel_latest[1] = mtr_sts.vel_pos[0] / transmission_ratio[1]; 
          st_pos_latest[1] = mtr_sts.vel_pos[1] / transmission_ratio[1];

          break;

      case MOTOR_ID::TILT_MOTOR:
          st_vel_latest[2] = mtr_sts.vel_pos[0] / transmission_ratio[2]; 
          st_pos_latest[2] = mtr_sts.vel_pos[1] / transmission_ratio[2];

          break;

      default:
          break;
  }

}

void mtr_sts_packet_cb(NET_PACKET & packet, QuoriStreamID stream_id, void * user_arg) {
    ((QuoriHeadSystemHardware*)user_arg)->mtr_sts_packet_cb(packet,stream_id);
}

}  // namespace quori_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quori_hardware::QuoriHeadSystemHardware, hardware_interface::SystemInterface)


