#include <baca_protocol.h>

namespace baca_protocol {

// Constructor implementation
BacaProtocol::BacaProtocol(const rclcpp::NodeOptions & options) : rclcpp::Node("baca_protocol", options) {
    nh_ = std::shared_ptr<rclcpp::Node>(this);
    serial_port_.set_node(nh_);

    interval_ = nh_->get_clock()->now();
    last_received_ = nh_->get_clock()->now();

    // | ---------------------- Param loader ---------------------- |

    _uav_name_ = declare_parameter<std::string>("uav_name", std::string("uav"));
    _portname_ = declare_parameter<std::string>("portname", std::string("/dev/ttyUSB0"));
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    _use_timeout_ = declare_parameter<bool>("use_timeout", true);
    serial_rate_ = declare_parameter<int>("serial_rate", 5000);

    // | ---------------------------------------------------------- |

    baca_protocol_pub_ = create_publisher<mrs_modules_msgs::msg::BacaProtocol>("~/baca_protocol_out", 1);

    baca_protocol_sub_ = create_subscription<mrs_modules_msgs::msg::BacaProtocol>(
        "~/baca_protocol_in", 10, std::bind(&BacaProtocol::callbackSendMessage, this, std::placeholders::_1));

    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "[%s] is up and running with the following parameters:", nh_->get_name());
    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "[%s] portname: %s", nh_->get_name(), _portname_.c_str());
    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "[%s] baudrate: %i", nh_->get_name(), baudrate_);

    connectToSensor();

    serial_timer_ = nh_->create_wall_timer(
        std::chrono::duration<double>(1.0 / serial_rate_), std::bind(&BacaProtocol::callbackSerialTimer, this));
    maintainer_timer_ = nh_->create_wall_timer(
        std::chrono::duration<double>(1.0), std::bind(&BacaProtocol::callbackMaintainerTimer, this));

    is_initialized_ = true;
}

// | ------------------------ callbacks ------------------------ |

void BacaProtocol::callbackSerialTimer() {

    uint8_t read_buffer[BUFFER_SIZE];
    int bytes_read;

    bytes_read = serial_port_.readSerial(read_buffer, BUFFER_SIZE);

    for (int i = 0; i < bytes_read; i++) {
        interpretSerialData(read_buffer[i]);
    }
}

void BacaProtocol::callbackMaintainerTimer() {

    if (is_connected_) {
        if (!serial_port_.checkConnected()) {
            is_connected_ = false;
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "[" << nh_->get_name() << "] Serial device is disconnected! ");
        }
    }

    if (((nh_->get_clock()->now() - last_received_).seconds() > MAXIMAL_TIME_INTERVAL) && _use_timeout_ && is_connected_) {
        is_connected_ = false;
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "[" << nh_->get_name()
                             << "] Serial port timed out - no messages were received in " << MAXIMAL_TIME_INTERVAL << " seconds");
    }

    if (is_connected_) {
        received_msg_ok = 0;
        received_msg_bad_checksum = 0;
        interval_ = nh_->get_clock()->now();
    } else {
        connectToSensor();
    }
}

void BacaProtocol::callbackSendMessage(const mrs_modules_msgs::msg::BacaProtocol::SharedPtr msg) {

    if (!is_initialized_) {
        return;
    }

    uint8_t payload_size = static_cast<uint8_t>(msg->payload.size());
    uint8_t checksum = 0;
    uint16_t it = 0;

    std::vector<uint8_t> out_buffer(payload_size + 3);

    out_buffer[it++] = 'b';
    checksum += 'b';
    out_buffer[it++] = payload_size;
    checksum += payload_size;

    for (int i = 0; i < payload_size; i++) {
        out_buffer[it++] = msg->payload[i];
        checksum += msg->payload[i];
    }

    out_buffer[it] = checksum;

    serial_port_.sendCharArray(out_buffer.data(), payload_size + 3);
}

// | ------------------------ routines ------------------------ |

void BacaProtocol::interpretSerialData(uint8_t single_character) {

    static serial_receiver_state rec_state = WAITING_FOR_MESSSAGE;
    static uint8_t payload_size = 0;
    static uint8_t input_buffer[BUFFER_SIZE];
    static uint8_t buffer_counter = 0;
    static uint8_t checksum = 0;

    switch (rec_state) {
        case WAITING_FOR_MESSSAGE:

            if (single_character == 'b' || single_character == 'a') {
                checksum = single_character;
                buffer_counter = 0;
                rec_state = EXPECTING_SIZE;
            }
            break;

        case EXPECTING_SIZE:

            if (single_character == 0) {
                RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "[%s]: Message with 0 payload_size received, discarding.",
                                       nh_->get_name());
                rec_state = WAITING_FOR_MESSSAGE;
            } else {
                payload_size = single_character;
                checksum += single_character;
                rec_state = EXPECTING_PAYLOAD;
            }
            break;

        case EXPECTING_PAYLOAD:

            input_buffer[buffer_counter] = single_character;
            checksum += single_character;
            buffer_counter++;
            if (buffer_counter >= payload_size) {
                rec_state = EXPECTING_CHECKSUM;
            }
            break;

        case EXPECTING_CHECKSUM:

            if (checksum == single_character) {
                processMessage(payload_size, input_buffer, checksum, single_character, true);
                last_received_ = nh_->get_clock()->now();
                rec_state = WAITING_FOR_MESSSAGE;
            } else {
                received_msg_bad_checksum++;
                rec_state = WAITING_FOR_MESSSAGE;
            }
            break;
    }
}

void BacaProtocol::processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct) {

    if (checksum_correct) {
        received_msg_ok++;
    }

    mrs_modules_msgs::msg::BacaProtocol msg;
    msg.stamp = nh_->get_clock()->now();
    for (uint8_t i = 0; i < payload_size; i++) {
        msg.payload.push_back(input_buffer[i]);
    }
    msg.checksum_received = checksum_rec;
    msg.checksum_calculated = checksum;
    msg.checksum_correct = checksum_correct;

    baca_protocol_pub_->publish(msg);
}

uint8_t BacaProtocol::connectToSensor(void) {

    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "[%s]: Openning the serial port.", nh_->get_name());

    if (!serial_port_.connect(_portname_, baudrate_)) {
        RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "[%s]: Could not connect to sensor.", nh_->get_name());
        is_connected_ = false;
        return 0;
    }

    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "[%s]: Connected to sensor.", nh_->get_name());
    is_connected_ = true;
    last_received_ = nh_->get_clock()->now();

    return 1;
}

} // namespace baca_protocol

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(baca_protocol::BacaProtocol)
