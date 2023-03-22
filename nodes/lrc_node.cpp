#include "lrc/lrc.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalResiliencyCoordinator::LocalRC>());
    rclcpp::shutdown();
    return 0;
}

