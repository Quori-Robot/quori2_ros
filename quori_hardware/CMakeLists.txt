cmake_minimum_required(VERSION 3.8)
project(quori_hardware)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(quori_msgs REQUIRED)
find_package(QuoriLib REQUIRED)

add_library(quori_hardware SHARED src/quori_head_hw_interface.cpp)

target_compile_features(quori_hardware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(quori_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/quori_hardware>
)

target_link_libraries(quori_hardware quorilib)

ament_target_dependencies(quori_hardware  rclcpp rclcpp_lifecycle hardware_interface pluginlib quori_msgs QuoriLib)

pluginlib_export_plugin_description_file(hardware_interface quori_hardware.xml)


install(
    DIRECTORY
    DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
    )

# Install config files
install(DIRECTORY 
    config
    DESTINATION share/${PROJECT_NAME}
    )


install(TARGETS quori_hardware
  EXPORT export_quori_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/head_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

  ament_add_gtest(quori_hardware_test_node test/test_load_quori_hardware.cpp)
  target_include_directories(quori_hardware_test_node PRIVATE include)
  #target_link_libraries(quori_hardware_test_node ${PROJECT_NAME}_lib)
  ament_target_dependencies(quori_hardware_test_node 
    pluginlib
    ament_cmake_gtest
    hardware_interface
    rclcpp_lifecycle
    ros2_control_test_assets
  )

endif()

ament_export_libraries($(PROJECT_NAME))
ament_export_targets(export_quori_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib QuoriLib)

ament_package()
