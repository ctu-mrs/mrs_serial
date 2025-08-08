#ifndef VIO_IMU_H
#define VIO_IMU_H

#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.h>
#include <string>
#include <functional>

#include <mrs_lib/param_loader.h>
#include <mrs_msgs/srv/set_int.hpp>
#include <mrs_serial/serial_port.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/publisher_handler.h>

#include "mrs_lib/service_server_handler.h"

#define BUFFER_SIZE 256
#define MAXIMAL_TIME_INTERVAL 1
#define SERIAL_BUFFER_SIZE 32   // This is replacement ofthe serial_buffer_size_ variable in the header. I do not know why there was second buffer length in the first place...

namespace vio_imu {

class VioImu : public rclcpp::Node {

public:
    VioImu(const rclcpp::NodeOptions & options);

private:
    enum serial_receiver_state {
        WAITING_FOR_MESSSAGE,
        EXPECTING_SIZE,
        EXPECTING_PAYLOAD,
        EXPECTING_CHECKSUM
    };

    // Timer handlers
    std::shared_ptr<mrs_lib::ROSTimer> serial_timer_;
    std::shared_ptr<mrs_lib::ROSTimer> maintainer_timer_;

    // Callback methods
    void interpretSerialData(uint8_t data);
    void callbackSerialTimer(void);
    void callbackMaintainerTimer(void);

    // Sensor connection
    uint8_t connectToSensor(void);

    void changeFrequency(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res);
    void changeCamFrequency(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res);
    void changeGyroUIFilter(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res);
    void changeAccUIFilter(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res);
    void changeGyroFilter(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res);
    void changeAccFilter(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res);

    // Message processing
    void processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec,
                        bool checksum_correct);

    // Node handle
    rclcpp::Node::SharedPtr nh_;

    // Publishers
    mrs_lib::PublisherHandler<sensor_msgs::msg::Imu> imu_publisher_;
    mrs_lib::PublisherHandler<sensor_msgs::msg::Imu> imu_publisher_sync_;

    mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt> service_frequency;
    mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt> service_camera_frequency;
    mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt> service_gyro_ui;
    mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt> service_accel_ui;
    mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt> service_gyro_filter;
    mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt> service_accel_filter;

    // Serial port
    serial_port::SerialPort serial_port_;

    // Callback function
    std::function<void(uint8_t)> serial_data_callback_function_;

    // Configuration parameters
    bool publish_bad_checksum;
    bool _use_timeout_;
    bool _verbose_;
    uint16_t received_msg_ok = 0;
    uint16_t received_msg_bad_checksum = 0;

    int serial_rate_ = 5000;

    std::string _portname_;
    int baudrate_;
    std::string _uav_name_;

    unsigned int message_counter;
    unsigned int skip_factor;
    int desired_publish_rate;

    // Timing
    rclcpp::Time interval_;
    rclcpp::Time last_received_;

    // State flags
    bool is_connected_ = false;
    bool is_initialized_ = false;
};

} // namespace vio_imu

#endif // VIO_IMU_H
