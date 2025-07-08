#include <vio_imu.h>  // Include header instead of .cpp

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<vio_imu::VioImu>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
