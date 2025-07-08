#ifndef VIO_IMU_H
#define VIO_IMU_H

#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.h>
#include <mutex>
#include <string>
#include <functional>

#include "mrs_lib/param_loader.h"
#include <mrs_msgs/srv/set_int.hpp>
#include "serial_port.h"
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/publisher_handler.h>

#define BUFFER_SIZE 256
#define MAXIMAL_TIME_INTERVAL 1

namespace vio_imu {

class VioImu : public rclcpp::Node {

public:
    VioImu();
    virtual void onInit();

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

    // Message processing
    void processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec,
                        bool checksum_correct);

    // Node handle
    rclcpp::Node::SharedPtr nh_;

    // Publishers
    mrs_lib::PublisherHandler<sensor_msgs::msg::Imu> imu_publisher_;
    mrs_lib::PublisherHandler<sensor_msgs::msg::Imu> imu_publisher_sync_;

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
    int serial_buffer_size_ = 32;

    std::string _portname_;
    int baudrate_;
    std::string _uav_name_;

    // Timing
    rclcpp::Time interval_;
    rclcpp::Time last_received_;

    // State flags
    bool is_connected_ = false;
    bool is_initialized_ = false;
};

} // namespace vio_imu

#endif // VIO_IMU_H