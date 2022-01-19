#include "mrs_msgs/SetIntRequest.h"
#include "mrs_msgs/SetIntResponse.h"
#include "mrs_msgs/TarotGimbalState.h"
#include <ros/package.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Char.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/SetBool.h>
#include <mrs_msgs/SetInt.h>
#include <std_msgs/Empty.h>
#include <mutex>

#include <string>
#include <mrs_msgs/BacaProtocol.h>
#include <mrs_msgs/SerialRaw.h>

#include <serial_port.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#define BUFFER_SIZE 256

#define MAX_RANGE 764  // cm
#define MIN_RANGE 21   // cm

#define MAXIMAL_TIME_INTERVAL 1

#define GIMBAL_MODE 0x01
#define GIMBAL_IS_ON 0x02

namespace tarot_gimbal
{

/* class TarotGimbal //{ */

class TarotGimbal : public nodelet::Nodelet {

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

  void interpretSerialData(uint8_t data);
  void callbackSerialTimer(const ros::TimerEvent &event);
  void callbackMaintainerTimer(const ros::TimerEvent &event);

  bool callbackAll(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackTarotGimbal(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackOuster(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  void callbackSendRawMessage(const mrs_msgs::SerialRawConstPtr &msg);
  void callbackSendCommand(const mrs_msgs::TarotGimbalState &msg);

  uint8_t connectToSensor(void);
  void    processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct);

  ros::NodeHandle nh_;

  ros::Publisher gimbal_status_publisher;
  ros::Publisher baca_protocol_publisher_;

  ros::Subscriber raw_message_subscriber;
  ros::Subscriber gimbal_command_subscriber;

  serial_port::SerialPort serial_port_;

  boost::function<void(uint8_t)> serial_data_callback_function_;

  bool publish_bad_checksum;
  bool use_timeout;

  uint16_t received_msg_ok           = 0;
  uint16_t received_msg_ok_gimbal    = 0;
  uint16_t received_msg_bad_checksum = 0;

  int serial_rate_        = 5000;
  int serial_buffer_size_ = 1024;

  std::string portname_;
  int         baudrate_;
  std::string uav_name_;

  std::mutex mutex_msg;

  ros::Time interval_      = ros::Time::now();
  ros::Time last_received_ = ros::Time::now();

  bool is_connected_   = false;
  bool is_initialized_ = false;
};

//}

/* onInit() //{ */

void TarotGimbal::onInit() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  nh_.param("uav_name", uav_name_, std::string("uav"));
  nh_.param("portname", portname_, std::string("/dev/ttyUSB0"));
  nh_.param("baudrate", baudrate_, 115200);
  nh_.param("publish_bad_checksum", publish_bad_checksum, false);
  nh_.param("use_timeout", use_timeout, true);
  nh_.param("serial_rate", serial_rate_, 5000);
  nh_.param("serial_buffer_size", serial_buffer_size_, 1024);

  // Publishers
  baca_protocol_publisher_ = nh_.advertise<mrs_msgs::BacaProtocol>("baca_protocol_out", 1);
  gimbal_status_publisher  = nh_.advertise<mrs_msgs::TarotGimbalState>("gimbal_state", 1);

  raw_message_subscriber = nh_.subscribe("raw_in", 10, &TarotGimbal::callbackSendRawMessage, this, ros::TransportHints().tcpNoDelay());

  gimbal_command_subscriber = nh_.subscribe("gimbal_command_in", 10, &TarotGimbal::callbackSendCommand, this, ros::TransportHints().tcpNoDelay());

  // Output loaded parameters to console for double checking
  ROS_INFO_THROTTLE(1.0, "[%s] test test:", ros::this_node::getName().c_str());
  ROS_INFO_THROTTLE(1.0, "[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO_THROTTLE(1.0, "[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());
  ROS_INFO_THROTTLE(1.0, "[%s] baudrate: %i", ros::this_node::getName().c_str(), baudrate_);
  ROS_INFO_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName().c_str() << "] publishing messages with wrong checksum: " << publish_bad_checksum);

  connectToSensor();

  serial_timer_     = nh_.createTimer(ros::Rate(serial_rate_), &TarotGimbal::callbackSerialTimer, this);
  maintainer_timer_ = nh_.createTimer(ros::Rate(1), &TarotGimbal::callbackMaintainerTimer, this);

  is_initialized_ = true;
}
//}

//}

// | ------------------------ callbacks ------------------------ |

/* callbackSerialTimer() //{ */

void TarotGimbal::callbackSerialTimer(const ros::TimerEvent &event) {

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

void TarotGimbal::callbackMaintainerTimer(const ros::TimerEvent &event) {

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

    ROS_INFO_STREAM("Got msgs - TarotGimbal: " << received_msg_ok_gimbal << " Generic msg: " << received_msg_ok << "  Wrong checksum: "
                                               << received_msg_bad_checksum << "; in the last " << (ros::Time::now() - interval_).toSec() << " s");
    received_msg_ok_gimbal    = 0;
    received_msg_ok           = 0;
    received_msg_bad_checksum = 0;

    interval_ = ros::Time::now();

  } else {

    connectToSensor();
  }
}

//}

/* callbackSendRawMessage() //{ */

void TarotGimbal::callbackSendRawMessage(const mrs_msgs::SerialRawConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  uint8_t payload_size = msg->payload.size();
  uint8_t out_buffer[payload_size];

  ROS_INFO_STREAM("SENDING");
  serial_port_.sendCharArray(out_buffer, payload_size);
}

//}

/* callbackSendCommand() //{ */

void TarotGimbal::callbackSendCommand(const mrs_msgs::TarotGimbalState &msg) {

  if (!is_initialized_) {
    return;
  }


  ROS_INFO_STREAM_THROTTLE(1.0, "Sending a command to the gimbal");

  std::scoped_lock lock(mutex_msg);

  uint8_t  payload_size = 6;
  uint8_t  checksum     = 0;
  uint16_t it           = 0;
  uint16_t ch1          = msg.gimbal_tilt;
  uint16_t ch2          = msg.gimbal_pan;
  bool     mode         = msg.fpv_mode;
  bool     is_on        = msg.is_on;
  uint8_t  bytes[payload_size];

  uint8_t out_buffer[payload_size + 3];

  bytes[0] = 0x1B;
  bytes[1] = (ch1 >> 8) & 0xFF;  // split 16 bit integer to two 8 bit integers, encode ch1
  bytes[2] = ch1 & 0xFF;

  bytes[3] = (ch2 >> 8) & 0xFF;
  // split 16 bit integer to two 8 bit integers, encode ch2
  bytes[4] = ch2 & 0xFF;

  bytes[5] = 0x00;  // put the two status bools in the last byte
  if (mode) {
    bytes[5] |= GIMBAL_MODE;
  }
  if (is_on) {
    bytes[5] |= GIMBAL_IS_ON;
  }


  out_buffer[it++] = 'b';
  checksum += 'b';
  out_buffer[it++] = payload_size;
  checksum += payload_size;

  for (int i = 0; i < payload_size; i++) {
    out_buffer[it++] = bytes[i];
    checksum += bytes[i];
  }

  out_buffer[it] = checksum;

  serial_port_.sendCharArray(out_buffer, payload_size + 3);
}

//}


// | ------------------------ routines ------------------------ |

/* interpretSerialData() //{ */

void TarotGimbal::interpretSerialData(uint8_t single_character) {

  static serial_receiver_state rec_state    = WAITING_FOR_MESSSAGE;
  static uint8_t               payload_size = 0;
  static uint8_t               input_buffer[BUFFER_SIZE];
  static uint8_t               buffer_counter = 0;
  static uint8_t               checksum       = 0;

  switch (rec_state) {
    case WAITING_FOR_MESSSAGE:

      if (single_character == 'b' ||
          single_character == 'a') {  // the 'a' is there for backwards-compatibility, going forwards all messages should start with 'b'
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

void TarotGimbal::processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct) {

  if (payload_size == 6 && (input_buffer[0] == 0x1A) && checksum_correct) {
    /* Special message reserved for tarot gimbal */

    received_msg_ok_gimbal++;

    uint8_t message_id = input_buffer[0];

    int16_t ch1 = input_buffer[1] << 8;
    ch1 |= input_buffer[2];

    int16_t ch2 = input_buffer[3] << 8;
    ch2 |= input_buffer[4];

    bool gimbal_mode  = false;
    bool gimbal_is_on = false;

    if (input_buffer[5] & GIMBAL_MODE) {
      gimbal_mode = true;
    }

    if (input_buffer[5] & GIMBAL_IS_ON) {
      gimbal_is_on = true;
    }

    mrs_msgs::TarotGimbalState gimbal_msg;

    gimbal_msg.is_on           = gimbal_is_on;
    gimbal_msg.fpv_mode        = gimbal_mode;
    gimbal_msg.gimbal_tilt     = ch1;
    gimbal_msg.gimbal_pan      = ch2;


    gimbal_msg.header.frame_id = uav_name_ + "/tarot_gimbal";

    try {
      gimbal_status_publisher.publish(gimbal_msg);
    }
    catch (...) {
      ROS_ERROR("[MrsSerial]: exception caught during publishing topic %s", gimbal_status_publisher.getTopic().c_str());
    }

  } else {
    /* General serial message */

    if (checksum_correct) {
      received_msg_ok++;
    }

    mrs_msgs::BacaProtocol msg;
    msg.stamp = ros::Time::now();
    for (uint8_t i = 0; i < payload_size; i++) {
      msg.payload.push_back(input_buffer[i]);
    }
    msg.checksum_received   = checksum_rec;
    msg.checksum_calculated = checksum;
    msg.checksum_correct    = checksum_correct;
    try {
      baca_protocol_publisher_.publish(msg);
    }
    catch (...) {
      ROS_ERROR("[MrsSerial]: exception caught during publishing topic %s", baca_protocol_publisher_.getTopic().c_str());
    }
  }
}

//}

/* connectToSensors() //{ */

uint8_t TarotGimbal::connectToSensor(void) {

  ROS_INFO_THROTTLE(1.0, "[%s]: Openning the serial port.", ros::this_node::getName().c_str());

  if (!serial_port_.connect(portname_, baudrate_)) {
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

}  // namespace tarot_gimbal

PLUGINLIB_EXPORT_CLASS(tarot_gimbal::TarotGimbal, nodelet::Nodelet);
