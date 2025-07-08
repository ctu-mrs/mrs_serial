// //#include <ros/package.h>
// #include <stdlib.h>
// #include <rclcpp/rclcpp.hpp>

// #include <sensor_msgs/msg/imu.hpp>
// #include <std_srvs/srv/trigger.h>
// #include <mutex>

// #include "mrs_lib/param_loader.h"

// #include <mrs_msgs/srv/set_int.hpp>

// #include <string>

// #include "serial_port.h"

// #include <mrs_lib/timer_handler.h>
// #include <mrs_lib/publisher_handler.h>

// //#include <nodelet/nodelet.h>
// //#include <pluginlib/class_list_macros.h>

// #define BUFFER_SIZE 256

// #define MAXIMAL_TIME_INTERVAL 1

// const double G       = 9.80665;
// const double DEG2RAD = 57.2958;

// namespace vio_imu {

// /* class VioImu //{ */

    // class VioImu : public rclcpp::Node {

    // public:
    //     VioImu() : rclcpp::Node("vio_imu") {
    //         nh_ = shared_from_this();
    //         serial_port_.set_node(nh_);
    //     }
    //     virtual void onInit();

    // private:
    //     enum serial_receiver_state {
    //         WAITING_FOR_MESSSAGE,
    //         EXPECTING_SIZE,
    //         EXPECTING_PAYLOAD,
    //         EXPECTING_CHECKSUM
    //     };


    //     std::shared_ptr<mrs_lib::ROSTimer> serial_timer_;
    //     std::shared_ptr<mrs_lib::ROSTimer> maintainer_timer_;

    //     // //rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr netgun_arm;
    //     // ros::ServiceServer netgun_safe;
    //     // ros::ServiceServer netgun_fire;

    //     // ros::ServiceServer service_frequency;
    //     // ros::ServiceServer service_camera_frequency;
    //     // ros::ServiceServer service_gyro_ui;
    //     // ros::ServiceServer service_accel_ui;
    //     // ros::ServiceServer service_gyro_filter;
    //     // ros::ServiceServer service_accel_filter;


    //     void interpretSerialData(uint8_t data);

    //     void callbackSerialTimer(void);

    //     void callbackMaintainerTimer(void);

    //     uint8_t connectToSensor(void);

    //     void processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec,
    //                         bool checksum_correct);

    //     // bool changeFrequency(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res);

    //     // bool changeCamFrequency(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res);

    //     // bool changeGyroUIFilter(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res);

    //     // bool changeAccUIFilter(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res);

    //     // bool changeGyroFilter(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res);

    //     // bool changeAccFilter(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res);


    //     rclcpp::Node::SharedPtr nh_;

    //     mrs_lib::PublisherHandler<sensor_msgs::msg::Imu> imu_publisher_;
    //     mrs_lib::PublisherHandler<sensor_msgs::msg::Imu> imu_publisher_sync_;

    //     serial_port::SerialPort serial_port_;

    //     std::function<void(uint8_t)> serial_data_callback_function_;

    //     bool publish_bad_checksum;
    //     bool _use_timeout_;
    //     bool _verbose_;
    //     uint16_t received_msg_ok = 0;
    //     uint16_t received_msg_bad_checksum = 0;

    //     int serial_rate_ = 5000;
    //     int serial_buffer_size_ = 32;

    //     std::string _portname_;
    //     int baudrate_;
    //     std::string _uav_name_;

    //     rclcpp::Time interval_ = nh_->get_clock()->now();
    //     rclcpp::Time last_received_ = nh_->get_clock()->now();

    //     bool is_connected_ = false;
    //     bool is_initialized_ = false;

    // };

//}

#include "vio_imu.h"

// Remove the #ifndef VIO_IMU_CPP_ and class definition
// Keep only the constants and implementation
const double G = 9.80665;
const double DEG2RAD = 57.2958;

namespace vio_imu {

// Constructor implementation
VioImu::VioImu() : rclcpp::Node("vio_imu") {
    nh_ = std::shared_ptr<rclcpp::Node>(this);
    serial_port_.set_node(nh_);
    
    // Initialize timing members here
    interval_ = nh_->get_clock()->now();
    last_received_ = nh_->get_clock()->now();

    // Get paramters
    //nh_ = std::make_shared<rclcpp::Node>("vio_imu");
    //rclcpp::Time::waitForValid();

    // | ---------------------- Param loader ---------------------- |

    mrs_lib::ParamLoader param_loader(nh_, "VioImu");

    param_loader.loadParam("uav_name", _uav_name_);
    param_loader.loadParam("portname", _portname_, std::string("/dev/ttyUSB0"));
    param_loader.loadParam("baudrate", baudrate_);
    param_loader.loadParam("use_timeout", _use_timeout_, true);
    param_loader.loadParam("serial_rate", serial_rate_, 460800);
    param_loader.loadParam("verbose", _verbose_, true);

    if (!param_loader.loadedSuccessfully()) {
        RCLCPP_ERROR(nh_->get_logger(), "[Status]: Could not load all parameters!");
        rclcpp::shutdown();
    } else {
        RCLCPP_INFO(nh_->get_logger(), "[Status]: All params loaded!");
    }

    // | ---------------------------------------------------------- |

    // imu_publisher_ = nh_.advertise<sensor_msgs::msg::Imu>("imu_raw", 1);
    imu_publisher_ = mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>(nh_, "/imu_raw");
    // imu_publisher_sync_ = nh_.advertise<sensor_msgs::msg::Imu>("imu_raw_synchronized", 1);
    imu_publisher_sync_ = mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>(nh_, "/imu_raw_synchronized");

    // Output loaded parameters to console for double checking
    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s] is up and running with the following parameters:",
                        nh_->get_name());
    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s] portname: %s", nh_->get_name(), _portname_.c_str());
    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s] baudrate: %i", nh_->get_name(), baudrate_);

    connectToSensor();

    // service_frequency = nh_.advertiseService("change_frequency", &VioImu::changeFrequency, this);
    // service_camera_frequency = nh_.advertiseService("change_camera_frequency", &VioImu::changeCamFrequency, this);
    // service_gyro_ui = nh_.advertiseService("change_gyro_ui_filter", &VioImu::changeGyroUIFilter, this);
    // service_accel_ui = nh_.advertiseService("change_acc_ui_filter", &VioImu::changeAccUIFilter, this);
    // service_gyro_filter = nh_.advertiseService("change_gyro_filter", &VioImu::changeGyroFilter, this);
    // service_accel_filter = nh_.advertiseService("change_acc_filter", &VioImu::changeAccFilter, this);

    //serial_timer_ = nh_.createTimer(ros::Rate(serial_rate_), &VioImu::callbackSerialTimer, this);
    serial_timer_ = std::make_shared<mrs_lib::ROSTimer>(nh_, rclcpp::Rate(serial_rate_, nh_->get_clock()), std::bind(&VioImu::callbackSerialTimer, this));
    //maintainer_timer_ = nh_.createTimer(ros::Rate(1), &VioImu::callbackMaintainerTimer, this);
    maintainer_timer_ = std::make_shared<mrs_lib::ROSTimer>(nh_, rclcpp::Rate(1, nh_->get_clock()), std::bind(&VioImu::callbackMaintainerTimer, this));

    is_initialized_ = true;
}

/* onInit() //{ */

    //void VioImu::onInit() {

        // // Get paramters
        // //nh_ = std::make_shared<rclcpp::Node>("vio_imu");
        // //rclcpp::Time::waitForValid();

        // // | ---------------------- Param loader ---------------------- |

        // mrs_lib::ParamLoader param_loader(nh_, "VioImu");

        // param_loader.loadParam("uav_name", _uav_name_);
        // param_loader.loadParam("portname", _portname_, std::string("/dev/ttyUSB0"));
        // param_loader.loadParam("baudrate", baudrate_);
        // param_loader.loadParam("use_timeout", _use_timeout_, true);
        // param_loader.loadParam("serial_rate", serial_rate_, 460800);
        // param_loader.loadParam("verbose", _verbose_, true);

        // if (!param_loader.loadedSuccessfully()) {
        //     RCLCPP_ERROR(nh_->get_logger(), "[Status]: Could not load all parameters!");
        //     rclcpp::shutdown();
        // } else {
        //     RCLCPP_INFO(nh_->get_logger(), "[Status]: All params loaded!");
        // }

        // // | ---------------------------------------------------------- |

        // // imu_publisher_ = nh_.advertise<sensor_msgs::msg::Imu>("imu_raw", 1);
        // imu_publisher_ = mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>(nh_, "/imu_raw");
        // // imu_publisher_sync_ = nh_.advertise<sensor_msgs::msg::Imu>("imu_raw_synchronized", 1);
        // imu_publisher_sync_ = mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>(nh_, "/imu_raw_synchronized");

        // // Output loaded parameters to console for double checking
        // RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s] is up and running with the following parameters:",
        //                   nh_->get_name());
        // RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s] portname: %s", nh_->get_name(), _portname_.c_str());
        // RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[%s] baudrate: %i", nh_->get_name(), baudrate_);

        // connectToSensor();

        // // service_frequency = nh_.advertiseService("change_frequency", &VioImu::changeFrequency, this);
        // // service_camera_frequency = nh_.advertiseService("change_camera_frequency", &VioImu::changeCamFrequency, this);
        // // service_gyro_ui = nh_.advertiseService("change_gyro_ui_filter", &VioImu::changeGyroUIFilter, this);
        // // service_accel_ui = nh_.advertiseService("change_acc_ui_filter", &VioImu::changeAccUIFilter, this);
        // // service_gyro_filter = nh_.advertiseService("change_gyro_filter", &VioImu::changeGyroFilter, this);
        // // service_accel_filter = nh_.advertiseService("change_acc_filter", &VioImu::changeAccFilter, this);

        // //serial_timer_ = nh_.createTimer(ros::Rate(serial_rate_), &VioImu::callbackSerialTimer, this);
        // serial_timer_ = std::make_shared<mrs_lib::ROSTimer>(nh_, rclcpp::Rate(serial_rate_, nh_->get_clock()), std::bind(&VioImu::callbackSerialTimer, this));
        // //maintainer_timer_ = nh_.createTimer(ros::Rate(1), &VioImu::callbackMaintainerTimer, this);
        // maintainer_timer_ = std::make_shared<mrs_lib::ROSTimer>(nh_, rclcpp::Rate(1, nh_->get_clock()), std::bind(&VioImu::callbackMaintainerTimer, this));

        // is_initialized_ = true;
    //}
//}


//}

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

        if (_verbose_)
            RCLCPP_INFO_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1, "[VioImu]: receiving IMU ok");

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

// /* changeGyroFilter() //{ */

//     bool VioImu::changeGyroUIFilter(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
//         if (!is_initialized_) {
//             return false;
//         }
//         if (req.value < 1 || req.value > 3) {
//             return false;
//         }
//         char msg[8];
//         sprintf(msg, "b\x01%05ld", req.value);
//         if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
//             ROS_INFO("[%s] : change gyro UI filter to %ld order", nh_->get_name(), req.value);
//             res.success = true;
//             res.message = "Done";
//             return true;
//         } else {
//             res.success = false;
//             res.message = "Fail";
//             return false;
//         }
//     }

// //}

// /* changeAccFilter() //{ */

//     bool VioImu::changeAccUIFilter(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
//         if (!is_initialized_) {
//             return false;
//         }
//         if (req.value < 1 || req.value > 3) {
//             return false;
//         }
//         char msg[8];
//         sprintf(msg, "b\x02%05ld", req.value);
//         if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
//             ROS_INFO("[%s] : change accelerometer UI filter to %ld order", nh_->get_name(),
//                      req.value);
//             res.success = true;
//             res.message = "Done";
//             return true;
//         } else {
//             res.success = false;
//             res.message = "Fail";
//             return false;
//         }
//     }

//}

// /* changeFrequency() //{ */

//     bool VioImu::changeFrequency(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
//         if (!is_initialized_) {
//             return false;
//         }
//         if (req.value > 99999) {
//             return false;
//         }
//         char msg[8];
//         sprintf(msg, "b\x03%05zu", req.value);
//         if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
//             ROS_INFO("[%s] : changed samples frequency to %ld", nh_->get_name(), req.value);
//             res.success = true;
//             res.message = "Done";
//             return true;
//         } else {
//             res.success = false;
//             res.message = "Fail";
//             return false;
//         }
//     }

// //}

// /* changeCamFrequency() //{ */

//     bool VioImu::changeCamFrequency(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
//         if (!is_initialized_) {
//             return false;
//         }
//         if (req.value > 99999) {
//             return false;
//         }
//         char msg[8];
//         sprintf(msg, "b\x04%05zu", req.value);
//         if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
//             ROS_INFO("[%s] : changed camera frequency to %ld", nh_->get_name(), req.value);
//             res.success = true;
//             res.message = "Done";
//             return true;
//         } else {
//             res.success = false;
//             res.message = "Fail";
//             return false;
//         }
//     }

// //}

// /* changeGyroFilter() //{ */

//     bool VioImu::changeGyroFilter(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
//         if (!is_initialized_) {
//             return false;
//         }
//         char msg[8];
//         sprintf(msg, "b\x05%05ld", req.value);
//         if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
//             auto turn = [](int a) -> std::string { if (a == 1) return "on"; else if (a == 0) return "off"; return "";};
//             ROS_INFO("[%s] : turn %s gyro filters", nh_->get_name(),
//                      turn(req.value).c_str());
//             res.success = true;
//             res.message = "Done";
//             return true;
//         } else {
//             res.success = false;
//             res.message = "Fail";
//             return false;
//         }
//     }

// //}

// /* changeAccFilter() //{ */

//     bool VioImu::changeAccFilter(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
//         if (!is_initialized_) {
//             return false;
//         }
//         char msg[8];
//         sprintf(msg, "b\x06%05ld", req.value);
//         if (serial_port_.sendCharArray(reinterpret_cast<uint8_t *>(msg), sizeof(msg))) {
//             auto turn = [](int a) -> std::string { if (a == 1) return "on"; else if (a == 0) return "off"; return "";};
//             ROS_INFO("[%s] : turn %s accelerometer filters", nh_->get_name(),
//                      turn(req.value).c_str());
//             res.success = true;
//             res.message = "Done";
//             return true;
//         } else {
//             res.success = false;
//             res.message = "Fail";
//             return false;
//         }
//     }

// //}


}  // namespace vio_imu

//PLUGINLIB_EXPORT_CLASS(vio_imu::VioImu, nodelet::Nodelet);