// Copyright (c) 2021 PickNik, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Denis Stogl

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
const auto TIME = rclcpp::Time(0);
const auto PERIOD = rclcpp::Duration::from_seconds(0.1);  // 0.1 seconds for easier math
const auto COMPARE_DELTA = 0.0001;
}  // namespace

class TestQuoriHardwareSystem : public ::testing::Test
{
public:

protected:
  void SetUp() override
  {

    quori_system_ =
      R"(
        <ros2_control name="quori_hardware/QuoriHeadSystemHardware" type="system">
            <hardware>
                <plugin>quori_hardware/QuoriHeadSystemHardware</plugin>
                <param name="robot_ip_address">10.2.99.111</param>
                <param name="ethernet_interface">eth0</param>
            </hardware>
            <joint name="neck_joint">
                <command_interface name="position"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
                <param name="initial_position">0</param>
                <param name="max_vel">0.5</param>       
                <param name="max_torque">0.5</param>
                <param name="max_accel">NAN</param>
            </joint>
            <joint name="pan_joint">
                <command_interface name="position"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
                <param name="initial_position">0</param>
                <param name="max_vel">0.5</param>
                <param name="max_torque">0.5</param>
                <param name="max_accel">NAN</param>
            </joint>
            <joint name="tilt_joint">
                <command_interface name="position"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
                <param name="initial_position">0</param>
                <param name="max_vel">0.5</param>
                <param name="max_torque">0.5</param>
                <param name="max_accel">NAN</param>
            </joint>
        </ros2_control>
      )";


  }

  std::string quori_system_;

};

// Forward declaration
namespace hardware_interface
{
class ResourceStorage;
}

class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
  friend TestQuoriHardwareSystem;

  // Provide a valid default constructor.
  TestableResourceManager(
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface)
    : hardware_interface::ResourceManager("default_urdf", clock_interface, logger_interface, false, 100)
  {}

  // Provide a constructor that accepts a URDF string, plus the interfaces.
  TestableResourceManager(
    const std::string & urdf,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_interface,
    bool activate_all = false, const unsigned int update_rate = 100)
    : hardware_interface::ResourceManager(urdf, clock_interface, logger_interface, activate_all, update_rate)
  {}
  // TestableResourceManager() : hardware_interface::ResourceManager() {}

  // TestableResourceManager(const std::string & urdf, bool validate_interfaces = true, bool activate_all = false)
  //       : hardware_interface::ResourceManager(urdf, validate_interfaces, activate_all)
  // {
  // };

  virtual ~TestableResourceManager(){};
};

void set_components_state(
  TestableResourceManager & rm, const std::vector<std::string> & components, const uint8_t state_id,
  const std::string & state_name)
{
  for (const auto & component : components)
  {
    rclcpp_lifecycle::State state(state_id, state_name);
    rm.set_component_state(component, state);
  }
}

auto configure_components = [](
                              TestableResourceManager & rm,
                              const std::vector<std::string> & components = {"QuoriHeadSystemHardware"})
{
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
};

auto activate_components = [](
                             TestableResourceManager & rm,
                             const std::vector<std::string> & components = {"QuoriHeadSystemHardware"})
{
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);
};

auto deactivate_components = [](
                               TestableResourceManager & rm,
                               const std::vector<std::string> & components = {"QuoriHeadSystemHardware"})
{
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
};

TEST_F(TestQuoriHardwareSystem, load_quori_system)
{
  auto urdf = ros2_control_test_assets::urdf_head + quori_system_ +
              ros2_control_test_assets::urdf_tail;
  // ASSERT_NO_THROW(TestableResourceManager rm(urdf));
}
