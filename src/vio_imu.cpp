#include <ros/package.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <mutex>

#include <string>

#include <serial_port.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#define BUFFER_SIZE 256

#define MAXIMAL_TIME_INTERVAL 1

const double G       = 9.80665;
const double DEG2RAD = 57.2958;

namespace vio_imu
{

/* class VioImu //{ */

class VioImu : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  enum serial_receiver_state
  {
    WAITING_FOR_MESSSAGE,
    EXPECTING_SIZE,
    EXPECTING_PAYLOAD,
    EXPECTING_CHECKSUM
  };


  ros::Timer serial_timer_;
  ros::Timer maintainer_timer_;

  ros::ServiceServer netgun_arm;
  ros::ServiceServer netgun_safe;
  ros::ServiceServer netgun_fire;


  void interpretSerialData(uint8_t data);
  void callbackSerialTimer(const ros::TimerEvent &event);
  void callbackMaintainerTimer(const ros::TimerEvent &event);

  uint8_t connectToSensor(void);
  void    processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct);


  ros::NodeHandle nh_;

  ros::Publisher imu_publisher_;
  ros::Publisher imu_publisher_sync_;

  serial_port::SerialPort serial_port_;

  boost::function<void(uint8_t)> serial_data_callback_function_;

  bool     publish_bad_checksum;
  bool     use_timeout;
  uint16_t received_msg_ok           = 0;
  uint16_t received_msg_bad_checksum = 0;

  int serial_rate_        = 5000;
  int serial_buffer_size_ = 1024;

  std::string _portname_;
  std::string _uav_name_;

  ros::Time interval_      = ros::Time::now();
  ros::Time last_received_ = ros::Time::now();

  bool is_connected_   = false;
  bool is_initialized_ = false;
};

//}

/* onInit() //{ */

void VioImu::onInit() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  nh_.param("uav_name", _uav_name_, std::string("uav"));
  nh_.param("portname", _portname_, std::string("/dev/vio_imu"));
  nh_.param("use_timeout", use_timeout, true);
  nh_.param("serial_rate", serial_rate_, 5000);

  imu_publisher_      = nh_.advertise<sensor_msgs::Imu>("imu_raw", 1);
  imu_publisher_sync_ = nh_.advertise<sensor_msgs::Imu>("imu_raw_synchronized", 1);

  // Output loaded parameters to console for double checking
  ROS_INFO_THROTTLE(1.0, "[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO_THROTTLE(1.0, "[%s] portname: %s", ros::this_node::getName().c_str(), _portname_.c_str());

  connectToSensor();

  serial_timer_     = nh_.createTimer(ros::Rate(serial_rate_), &VioImu::callbackSerialTimer, this);
  maintainer_timer_ = nh_.createTimer(ros::Rate(1), &VioImu::callbackMaintainerTimer, this);

  is_initialized_ = true;
}
//}


//}

// | ------------------------ callbacks ------------------------ |

/* callbackSerialTimer() //{ */

void VioImu::callbackSerialTimer(const ros::TimerEvent &event) {

  uint8_t read_buffer[serial_buffer_size_];
  int     bytes_read;

  bytes_read = serial_port_.readSerial(read_buffer, serial_buffer_size_);

  for (int i = 0; i < bytes_read; i++) {
    interpretSerialData(read_buffer[i]);
  }
  /* processMessage */
}

//}

/* callbackMaintainerTimer() //{ */

void VioImu::callbackMaintainerTimer(const ros::TimerEvent &event) {

  if (is_connected_) {

    if (!serial_port_.checkConnected()) {
      is_connected_ = false;
      ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "] Serial device is disconnected! ");
    }
  }

  if (((ros::Time::now() - last_received_).toSec() > MAXIMAL_TIME_INTERVAL) && use_timeout && is_connected_) {

    is_connected_ = false;

    ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "] Serial port timed out - no messages were received in " << MAXIMAL_TIME_INTERVAL
                         << " seconds");
  }

  if (is_connected_) {

    received_msg_ok           = 0;
    received_msg_bad_checksum = 0;

    interval_ = ros::Time::now();

  } else {

    connectToSensor();
  }
}

//}

// | ------------------------ routines ------------------------ |

/* interpretSerialData() //{ */

void VioImu::interpretSerialData(uint8_t single_character) {

  static serial_receiver_state rec_state    = WAITING_FOR_MESSSAGE;
  static uint8_t               payload_size = 0;
  static uint8_t               input_buffer[BUFFER_SIZE];
  static uint8_t               buffer_counter = 0;
  static uint8_t               checksum       = 0;

  ROS_INFO_THROTTLE(1.0, "[VioImu]: receiving IMU ok ");

  switch (rec_state) {
    case WAITING_FOR_MESSSAGE:

      if (single_character == 'b') {
        checksum       = single_character;
        buffer_counter = 0;
        rec_state      = EXPECTING_SIZE;
      }
      break;

    case EXPECTING_SIZE:

      if (single_character == 0) {
        ROS_ERROR_THROTTLE(1.0, "[%s]: Message with 0 payload_size received, discarding.", ros::this_node::getName().c_str());
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
        last_received_ = ros::Time::now();
        rec_state      = WAITING_FOR_MESSSAGE;
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

void VioImu::processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct) {

  if (payload_size == 13 && (input_buffer[0] == 0x30 || input_buffer[0] == 0x31) && checksum_correct) {

    int32_t acc_x, acc_y, acc_z    = 0;
    int32_t gyro_x, gyro_y, gyro_z = 0;

    acc_x = int16_t(input_buffer[1] << 8) | (input_buffer[2] & 0xff);
    acc_y = int16_t(input_buffer[3] << 8) | (input_buffer[4] & 0xff);
    acc_z = int16_t(input_buffer[5] << 8) | (input_buffer[6] & 0xff);

    gyro_x = int16_t(input_buffer[7] << 8) | (input_buffer[8] & 0xff);
    gyro_y = int16_t(input_buffer[9] << 8) | (input_buffer[10] & 0xff);
    gyro_z = int16_t(input_buffer[11] << 8) | (input_buffer[12] & 0xff);

    sensor_msgs::Imu imu;

    imu.linear_acceleration.x = (double(acc_x) / 8192) * G;
    imu.linear_acceleration.y = (double(acc_y) / 8192) * G;
    imu.linear_acceleration.z = (double(acc_z) / 8192) * G;

    imu.angular_velocity.x = (double(gyro_x) / 16.384) / DEG2RAD;
    imu.angular_velocity.y = (double(gyro_y) / 16.384) / DEG2RAD;
    imu.angular_velocity.z = (double(gyro_z) / 16.384) / DEG2RAD;

    imu.header.stamp    = ros::Time::now();
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

  ROS_INFO_THROTTLE(1.0, "[%s]: Openning the serial port.", ros::this_node::getName().c_str());

  if (!serial_port_.connect(_portname_)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not connect to sensor.", ros::this_node::getName().c_str());
    is_connected_ = false;
    return 0;
  }

  ROS_INFO_THROTTLE(1.0, "[%s]: Connected to sensor.", ros::this_node::getName().c_str());
  is_connected_  = true;
  last_received_ = ros::Time::now();

  return 1;
}

//}

}  // namespace vio_imu

PLUGINLIB_EXPORT_CLASS(vio_imu::VioImu, nodelet::Nodelet);
