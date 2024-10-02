#include "quori/quori_cmd.hpp"

int main(int argc, char **argv) {

    rclcpp::init(argc,argv);
    std::shared_ptr<QuoriCmd> quori_cmd = std::make_shared<QuoriCmd>();

    //rclcpp::executors::MultiThreadedExecutor exec;
    //exec.add_node(quori_node);
    //exec.spin();
    rclcpp::spin(quori_cmd);
    rclcpp::shutdown();
    return 0;

}
