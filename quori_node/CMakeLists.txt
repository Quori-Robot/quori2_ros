cmake_minimum_required(VERSION 3.8)
project(quori_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quori_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

find_package(Quori2_Interface REQUIRED)
find_package(Threads REQUIRED)

add_executable(quori_node src/quori_node.cpp include/quori/quori_node.hpp include/quori/quori_odom.hpp)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_include_directories(quori_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} quori2_interface Eigen3::Eigen)

target_compile_features(quori_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(quori_node rclcpp  tf2 tf2_ros std_msgs nav_msgs sensor_msgs geometry_msgs quori_msgs Quori2_Interface)



add_executable(quori_cmd src/quori_cmd.cpp include/quori/quori_cmd.hpp)
target_include_directories(quori_cmd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(quori_cmd quori2_interface)

target_compile_features(quori_cmd PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(quori_cmd rclcpp quori_msgs Quori2_Interface)

add_executable(quori_ivt src/quori_ivt.cpp include/quori/quori_ivt.hpp)
target_include_directories(quori_ivt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(quori_ivt quori2_interface)

target_compile_features(quori_ivt PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(quori_ivt rclcpp quori_msgs Quori2_Interface sensor_msgs)


#install package
install(
    TARGETS quori_node quori_cmd quori_ivt
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
    )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
