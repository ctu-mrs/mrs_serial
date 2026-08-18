#ifndef BACA_PROTOCOL_H
#define BACA_PROTOCOL_H

#include <rclcpp/rclcpp.hpp>
#include <mrs_modules_msgs/msg/baca_protocol.hpp>
#include <mrs_serial/serial_port.h>
#include <string>

#define BUFFER_SIZE 256
#define MAXIMAL_TIME_INTERVAL 1

namespace baca_protocol {

class BacaProtocol : public rclcpp::Node {

public:
    BacaProtocol(const rclcpp::NodeOptions & options);

private:
    enum serial_receiver_state {
        WAITING_FOR_MESSSAGE,
        EXPECTING_SIZE,
        EXPECTING_PAYLOAD,
        EXPECTING_CHECKSUM
    };

    // Timers
    rclcpp::TimerBase::SharedPtr serial_timer_;
    rclcpp::TimerBase::SharedPtr maintainer_timer_;

    // Callbacks
    void interpretSerialData(uint8_t data);
    void callbackSerialTimer();
    void callbackMaintainerTimer();
    void callbackSendMessage(const mrs_modules_msgs::msg::BacaProtocol::SharedPtr msg);

    // Message processing
    void processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct);

    // Sensor connection
    uint8_t connectToSensor(void);

    // Node handle
    rclcpp::Node::SharedPtr nh_;

    // Pub/sub
    rclcpp::Publisher<mrs_modules_msgs::msg::BacaProtocol>::SharedPtr baca_protocol_pub_;
    rclcpp::Subscription<mrs_modules_msgs::msg::BacaProtocol>::SharedPtr baca_protocol_sub_;

    // Serial port
    serial_port::SerialPort serial_port_;

    // Configuration parameters
    bool _use_timeout_;
    uint16_t received_msg_ok = 0;
    uint16_t received_msg_bad_checksum = 0;

    int serial_rate_ = 5000;

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

} // namespace baca_protocol

#endif // BACA_PROTOCOL_H
