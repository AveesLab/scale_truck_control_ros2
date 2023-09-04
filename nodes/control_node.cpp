#include "scale_truck_control/ScaleTruckController.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<scale_truck_control::ScaleTruckController>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();

    return 0;
}
