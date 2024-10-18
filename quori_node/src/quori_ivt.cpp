#include "quori/quori_ivt.hpp"

int main(int argc, char **argv) {

    rclcpp::init(argc,argv);
    std::shared_ptr<QuoriIVT> quori_ivt = std::make_shared<QuoriIVT>();

    //rclcpp::executors::MultiThreadedExecutor exec;
    //exec.add_node(quori_node);
    //exec.spin();
    rclcpp::spin(quori_ivt);
    rclcpp::shutdown();
    return 0;

}
