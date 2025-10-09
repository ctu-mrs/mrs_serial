#include <vio_imu.h>

const double G = 9.80665;
const double DEG2RAD = 57.2958;

namespace vio_imu {

// Constructor implementation
VioImu::VioImu(const rclcpp::NodeOptions & options) : rclcpp::Node("vio_imu", options) {
    nh_ = std::shared_ptr<rclcpp::Node>(this);
    serial_port_.set_node(nh_);
    
    // Initialize timing members here
    interval_ = nh_->get_clock()->now();
    last_received_ = nh_->get_clock()->now();

    // | ---------------------- Param loader ---------------------- |

    mrs_lib::ParamLoader param_loader(nh_, "VioImu");

    param_loader.loadParam("uav_name", _uav_name_);
    param_loader.loadParam("portname", _portname_, std::string("/dev/ttyUSB0"));
    param_loader.loadParam("baudrate", baudrate_);
    param_loader.loadParam("use_timeout", _use_timeout_, true);
    param_loader.loadParam("serial_rate", serial_rate_, 460800);
    param_loader.loadParam("verbose", _verbose_, true);
    param_loader.loadParam("desired_publish_rate", desired_publish_rate, 200);

    skip_factor = 1000/(desired_publish_rate);

    if (!param_loader.loadedSuccessfully()) {
        RCLCPP_ERROR(nh_->get_logger(), "[Status]: Could not load all parameters!");
        rclcpp::shutdown();
    } else {
        RCLCPP_INFO(nh_->get_logger(), "[Status]: All params loaded!");
    }

    // | ---------------------------------------------------------- |

    imu_publisher_ = mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>(nh_, "~/imu_raw_out");
    imu_publisher_sync_ = mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>(nh_, "~/imu_raw_synchronized_out");

    // Output loaded parameters to console for double checking
    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s] is up and running with the following parameters:",
                        nh_->get_name());
    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s] portname: %s", nh_->get_name(), _portname_.c_str());
    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s] baudrate: %i", nh_->get_name(), baudrate_);

    connectToSensor();

    service_frequency = mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt>(
        nh_, 
        std::string("change_frequency"), 
        std::bind(&VioImu::changeFrequency, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::SystemDefaultsQoS(),  // QoS parameter
        nullptr  // callback group
    );

    service_camera_frequency = mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt>(
        nh_, 
        std::string("change_camera_frequency"), 
        std::bind(&VioImu::changeCamFrequency, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::SystemDefaultsQoS(),  // QoS parameter
        nullptr  // callback group
    );

    service_gyro_ui = mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt>(
        nh_, 
        std::string("change_gyro_ui_filter"), 
        std::bind(&VioImu::changeGyroUIFilter, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::SystemDefaultsQoS(),  // QoS parameter
        nullptr  // callback group
    );

    service_accel_ui = mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt>(
        nh_, 
        std::string("change_acc_ui_filter"), 
        std::bind(&VioImu::changeAccUIFilter, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::SystemDefaultsQoS(),  // QoS parameter
        nullptr  // callback group
    );

    service_gyro_filter = mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt>(
        nh_, 
        std::string("change_gyro_filter"), 
        std::bind(&VioImu::changeGyroFilter, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::SystemDefaultsQoS(),  // QoS parameter
        nullptr  // callback group
    );

    service_accel_filter = mrs_lib::ServiceServerHandler<mrs_msgs::srv::SetInt>(
        nh_, 
        std::string("change_acc_filter"), 
        std::bind(&VioImu::changeAccFilter, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::SystemDefaultsQoS(),  // QoS parameter
        nullptr  // callback group
    );

    serial_timer_ = std::make_shared<mrs_lib::ROSTimer>(nh_, rclcpp::Rate(serial_rate_, nh_->get_clock()), std::bind(&VioImu::callbackSerialTimer, this));
    maintainer_timer_ = std::make_shared<mrs_lib::ROSTimer>(nh_, rclcpp::Rate(1, nh_->get_clock()), std::bind(&VioImu::callbackMaintainerTimer, this));

    is_initialized_ = true;
}

// | ------------------------ callbacks ------------------------ |

/* callbackSerialTimer() //{ */

    void VioImu::callbackSerialTimer() {

        uint8_t read_buffer[SERIAL_BUFFER_SIZE];
        int bytes_read;

        bytes_read = serial_port_.readSerial(read_buffer, SERIAL_BUFFER_SIZE);

        for (int i = 0; i < bytes_read; i++) {
            interpretSerialData(read_buffer[i]);
        }
        /* processMessage */
    }

//}

/* callbackMaintainerTimer() //{ */

    void VioImu::callbackMaintainerTimer() {

        if (is_connected_) {

            if (!serial_port_.checkConnected()) {
                is_connected_ = false;
                RCLCPP_ERROR_STREAM(nh_->get_logger(),"[" << nh_->get_name() << "] Serial device is disconnected! ");
            }
        }

        if (((nh_->get_clock()->now() - last_received_).seconds() > MAXIMAL_TIME_INTERVAL) && _use_timeout_ && is_connected_) {

            is_connected_ = false;

            RCLCPP_ERROR_STREAM(nh_->get_logger(),"[" << nh_->get_name()
                                 << "] Serial port timed out - no messages were received in " << MAXIMAL_TIME_INTERVAL
                                 << " seconds");
        }

        if (is_connected_) {

            received_msg_ok = 0;
            received_msg_bad_checksum = 0;

            interval_ = nh_->get_clock()->now();

        } else {

            connectToSensor();
        }
    }

//}

// | ------------------------ routines ------------------------ |

/* interpretSerialData() //{ */

    void VioImu::interpretSerialData(uint8_t single_character) {

        static serial_receiver_state rec_state = WAITING_FOR_MESSSAGE;
        static uint8_t payload_size = 0;
        static uint8_t input_buffer[BUFFER_SIZE];
        static uint8_t buffer_counter = 0;
        static uint8_t checksum = 0;

        //if (_verbose_)
        //    RCLCPP_INFO_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "[VioImu]: receiving IMU ok");

        switch (rec_state) {
            case WAITING_FOR_MESSSAGE:

                if (single_character == 'b') {
                    checksum = single_character;
                    buffer_counter = 0;
                    rec_state = EXPECTING_SIZE;
                }
                break;

            case EXPECTING_SIZE:

                if (single_character == 0) {
                    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s]: Message with 0 payload_size received, discarding.",
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
                    if (publish_bad_checksum) {
                        processMessage(payload_size, input_buffer, checksum, single_character, false);
                    }
                    received_msg_bad_checksum++;
                    rec_state = WAITING_FOR_MESSSAGE;
                }
                break;
        }
    }

//}

/* processMessage() //{ */

    void VioImu::processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec,
                                bool checksum_correct) {
        message_counter++;
        if(message_counter % skip_factor) return;

        (void)checksum; (void)checksum_rec; // suppressing 'unused parameter' warning

        if (payload_size == 13 && (input_buffer[0] == 0x30 || input_buffer[0] == 0x31) && checksum_correct) {

            int32_t acc_x, acc_y, acc_z = 0;
            int32_t gyro_x, gyro_y, gyro_z = 0;

            acc_x = int16_t(input_buffer[1] << 8) | (input_buffer[2] & 0xff);
            acc_y = int16_t(input_buffer[3] << 8) | (input_buffer[4] & 0xff);
            acc_z = int16_t(input_buffer[5] << 8) | (input_buffer[6] & 0xff);

            gyro_x = int16_t(input_buffer[7] << 8) | (input_buffer[8] & 0xff);
            gyro_y = int16_t(input_buffer[9] << 8) | (input_buffer[10] & 0xff);
            gyro_z = int16_t(input_buffer[11] << 8) | (input_buffer[12] & 0xff);

            sensor_msgs::msg::Imu imu;

            imu.linear_acceleration.x = (double(acc_x) / 4096) * G;
            imu.linear_acceleration.y = (double(acc_y) / 4096) * G;
            imu.linear_acceleration.z = (double(acc_z) / 4096) * G;

            imu.angular_velocity.x = (double(gyro_x) / 65.536) / DEG2RAD;
            imu.angular_velocity.y = (double(gyro_y) / 65.536) / DEG2RAD;
            imu.angular_velocity.z = (double(gyro_z) / 65.536) / DEG2RAD;

            imu.header.stamp = nh_->get_clock()->now();
            imu.header.frame_id = _uav_name_ + "/vio_imu";
            if (input_buffer[0] == 0x30) {

                imu_publisher_.publish(imu);
            } else {
                imu_publisher_.publish(imu);
                imu_publisher_sync_.publish(imu);
            }
        }
    }  // namespace vio_imu

//}

/* connectToSensors() //{ */

    uint8_t VioImu::connectToSensor(void) {

        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s]: Openning the serial port.", nh_->get_name());

        if (!serial_port_.connect(_portname_, baudrate_)) {
            RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s]: Could not connect to sensor.", nh_->get_name());
            is_connected_ = false;
            return 0;
        }

        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s]: Connected to sensor.", nh_->get_name());
        is_connected_ = true;
        last_received_ = nh_->get_clock()->now();

        return 1;
    }

//}

// | ------------------------ services ------------------------ |

void VioImu::changeGyroUIFilter(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res) {
    if (!is_initialized_) {
        res->success = false;
        res->message = "not initialized yet";
        RCLCPP_WARN(get_logger(), res->message.c_str());
        return;
    }
    if (req->value < 1 || req->value > 3) {
        res->success = false;
        res->message = "invalid value - not between 1 and 3 ";
        RCLCPP_ERROR(get_logger(), res->message.c_str());
        return;
    }
    char msg[8];
    sprintf(msg, "b\x01%05ld", req->value);
    if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
        RCLCPP_INFO(get_logger(), "[%s] : change gyro UI filter to %ld order", nh_->get_name(), req->value);
        res->success = true;
        res->message = "Done";
        return;
    } else {
        res->success = false;
        res->message = "Fail";
        return;
    }
}

void VioImu::changeAccUIFilter(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res) {
    if (!is_initialized_) {
        res->success = false;
        res->message = "not initialized yet";
        RCLCPP_WARN(get_logger(), res->message.c_str());
        return;
    }
    if (req->value < 1 || req->value > 3) {
        res->success = false;
        res->message = "invalid value - not between 1 and 3 ";
        RCLCPP_ERROR(get_logger(), res->message.c_str());
        return;
    }
    char msg[8];
    sprintf(msg, "b\x02%05ld", req->value);
    if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
        RCLCPP_INFO(get_logger(), "[%s] : change accelerometer UI filter to %ld order", nh_->get_name(),
                    req->value);
        res->success = true;
        res->message = "Done";
        return;
    } else {
        res->success = false;
        res->message = "Fail";
        return;
    }
}

void VioImu::changeFrequency(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res) {
    if (!is_initialized_) {
        res->success = false;
        res->message = "not initialized yet";
        RCLCPP_WARN(get_logger(), res->message.c_str());
        return;
    }
    unsigned int max_val = 99999;
    if (req->value > max_val) {
        res->success = false;
        std::ostringstream oss;
        oss << "invalid value - it is greater than " << max_val;
        res->message = oss.str();
        RCLCPP_ERROR(get_logger(), res->message.c_str());
        return;
    }
    char msg[8];
    sprintf(msg, "b\x03%05zu", req->value);
    if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
        RCLCPP_INFO(get_logger(), "[%s] : changed samples frequency to %ld", nh_->get_name(), req->value);
        res->success = true;
        res->message = "Done";
        return;
    } else {
        res->success = false;
        res->message = "Fail";
        return;
    }
}

void VioImu::changeCamFrequency(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res) {
    if (!is_initialized_) {
        res->success = false;
        res->message = "not initialized yet";
        RCLCPP_WARN(get_logger(), res->message.c_str());
        return;
    }
    unsigned int max_val = 99999;
    if (req->value > max_val) {
        res->success = false;
        std::ostringstream oss;
        oss << "invalid value - it is greater than " << max_val;
        res->message = oss.str();
        RCLCPP_ERROR(get_logger(), res->message.c_str());
        return;
    }
    char msg[8];
    sprintf(msg, "b\x04%05zu", req->value);
    if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
        RCLCPP_INFO(get_logger(), "[%s] : changed camera frequency to %ld", nh_->get_name(), req->value);
        res->success = true;
        res->message = "Done";
        return;
    } else {
        res->success = false;
        res->message = "Fail";
        return;
    }
}

void VioImu::changeGyroFilter(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res) {
    if (!is_initialized_) {
        res->success = false;
        res->message = "not initialized yet";
        RCLCPP_WARN(get_logger(), res->message.c_str());
        return;
    }
    char msg[8];
    sprintf(msg, "b\x05%05ld", req->value);
    if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
        auto turn = [](int a) -> std::string { if (a == 1) return "on"; else if (a == 0) return "off"; return "";};
        RCLCPP_INFO(get_logger(), "[%s] : turn %s gyro filters", nh_->get_name(),
                    turn(req->value).c_str());
        res->success = true;
        res->message = "Done";
        return;
    } else {
        res->success = false;
        res->message = "Fail";
        return;
    }
}

void VioImu::changeAccFilter(std::shared_ptr<mrs_msgs::srv::SetInt::Request> req, std::shared_ptr<mrs_msgs::srv::SetInt::Response> res) {
    if (!is_initialized_) {
        res->success = false;
        res->message = "not initialized yet";
        RCLCPP_WARN(get_logger(), res->message.c_str());
        return;
    }
    char msg[8];
    sprintf(msg, "b\x06%05ld", req->value);
    if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
        auto turn = [](int a) -> std::string { if (a == 1) return "on"; else if (a == 0) return "off"; return "";};
        RCLCPP_INFO(get_logger(), "[%s] : turn %s accelerometer filters", nh_->get_name(),
                    turn(req->value).c_str());
        res->success = true;
        res->message = "Done";
        return;
    } else {
        res->success = false;
        res->message = "Fail";
        return;
    }
}


}  // namespace vio_imu

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vio_imu::VioImu)
