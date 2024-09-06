
#include "quori/quori_node.hpp"

int main(int argc, char **argv) {

    rclcpp::init(argc,argv);
    std::shared_ptr<QuoriNode> quori_node = std::make_shared<QuoriNode>();

    //rclcpp::executors::MultiThreadedExecutor exec;
    //exec.add_node(quori_node);
    //exec.spin();
    rclcpp::spin(quori_node);
    rclcpp::shutdown();
    return 0;

}
