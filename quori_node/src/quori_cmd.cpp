#include "quori/quori_cmd.hpp"
#include <chrono>

using namespace std::chrono_literals;
int main(int argc, char **argv) {

    rclcpp::init(argc,argv);
    std::shared_ptr<QuoriCmd> quori_cmd = std::make_shared<QuoriCmd>();

    //rclcpp::executors::MultiThreadedExecutor exec;
    //exec.add_node(quori_node);
    //exec.spin();
    while(rclcpp::ok()) {
        rclcpp::spin_some(quori_cmd);
        //printf(".\n");
        if(quori_cmd->shutdown()) {
            break;
        }
    }
    printf("quori_cmd finished\n");
    //quori_cmd->close();
    //quori_cmd->destroy_node()
    rclcpp::shutdown();
    return 0;

}
