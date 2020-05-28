#include <ros/package.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Char.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Empty.h>
#include <mutex>

#include <string>
#include <mrs_msgs/BacaProtocol.h>
#include <mrs_msgs/SerialRaw.h>

#include "serial_port.h"

#define BUFFER_SIZE 256

#define MAXIMAL_TIME_INTERVAL 1

// for garmin
#define MAX_RANGE 4000  // cm
#define MIN_RANGE 10    // cm


/* class BacaProtocol //{ */

class BacaProtocol {
public:
  BacaProtocol();

  void callbackSerialData(uint8_t data);

  enum serial_receiver_state
  {
    WAITING_FOR_MESSSAGE,
    EXPECTING_SIZE,
    EXPECTING_PAYLOAD,
    EXPECTING_CHECKSUM
  };


  ros::ServiceServer netgun_arm;
  ros::ServiceServer netgun_safe;
  ros::ServiceServer netgun_fire;

  bool callbackNetgunSafe(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackNetgunArm(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackNetgunFire(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void callbackSendMessage(const mrs_msgs::BacaProtocolConstPtr &msg);
  void callbackSendRawMessage(const mrs_msgs::SerialRawConstPtr &msg);
  void callbackMagnet(const std_msgs::EmptyConstPtr &msg);


  uint8_t connectToSensor(void);
  void    releaseSerialLine(void);
  void    processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct);

  ros::Time lastReceived;
  ros::Time lastPrinted;

  ros::NodeHandle nh_;

  ros::Publisher range_publisher_A_;
  ros::Publisher range_publisher_B_;
  ros::Publisher baca_protocol_publisher_;

  ros::Subscriber raw_message_subscriber;
  ros::Subscriber baca_protocol_subscriber;
  ros::Subscriber magnet_subscriber;

  serial_device::SerialPort *    serial_port_;
  boost::function<void(uint8_t)> serial_data_callback_function_;

  bool     publish_bad_checksum;
  bool     use_timeout;
  bool     swap_garmins;
  uint16_t received_msg_ok           = 0;
  uint16_t received_msg_ok_garmin    = 0;
  uint16_t received_msg_bad_checksum = 0;

  std::string portname_;
  std::string uav_name_;
  std::string garmin_A_frame_;
  std::string garmin_B_frame_;

  std::mutex mutex_msg;
};

//}

/* BacaProtocol() //{ */

BacaProtocol::BacaProtocol() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  nh_.param("uav_name", uav_name_, std::string("uav"));
  nh_.param("portname", portname_, std::string("/dev/ttyUSB0"));
  nh_.param("publish_bad_checksum", publish_bad_checksum, false);
  nh_.param("use_timeout", use_timeout, true);
  nh_.param("swap_garmins", swap_garmins, false);

  // Publishers
  std::string postfix_A = swap_garmins ? "_up" : "";
  std::string postfix_B = swap_garmins ? "" : "_up";
  range_publisher_A_    = nh_.advertise<sensor_msgs::Range>("range" + postfix_A, 1);
  range_publisher_B_    = nh_.advertise<sensor_msgs::Range>("range" + postfix_B, 1);
  garmin_A_frame_       = uav_name_ + "/garmin" + postfix_A;
  garmin_B_frame_       = uav_name_ + "/garmin" + postfix_B;

  baca_protocol_publisher_ = nh_.advertise<mrs_msgs::BacaProtocol>("baca_protocol_out", 1);

  baca_protocol_subscriber = nh_.subscribe("baca_protocol_in", 1, &BacaProtocol::callbackSendMessage, this, ros::TransportHints().tcpNoDelay());

  raw_message_subscriber = nh_.subscribe("raw_in", 1, &BacaProtocol::callbackSendRawMessage, this, ros::TransportHints().tcpNoDelay());

  // Output loaded parameters to console for double checking
  ROS_INFO_THROTTLE(1.0, "[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO_THROTTLE(1.0, "[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());
  ROS_INFO_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName().c_str() << "] publishing messages with wrong checksum: " << publish_bad_checksum);
  lastReceived = ros::Time::now();
  lastPrinted  = ros::Time::now();

  connectToSensor();
}

//}

// | ------------------------ callbacks ------------------------ |

/* callbackSerialData() //{ */

void BacaProtocol::callbackSerialData(uint8_t single_character) {

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
        lastReceived = ros::Time::now();
        rec_state    = WAITING_FOR_MESSSAGE;
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

/* callbackSendMessage() //{ */

void BacaProtocol::callbackSendMessage(const mrs_msgs::BacaProtocolConstPtr &msg) {

  ROS_INFO_STREAM_THROTTLE(1.0, "SENDING: " << msg->payload[0]);

  std::scoped_lock lock(mutex_msg);
  uint8_t          payload_size = msg->payload.size();
  uint8_t          checksum     = 0;
  uint16_t         it           = 0;

  uint8_t out_buffer[payload_size + 3];

  out_buffer[it++] = 'b';
  checksum += 'b';
  out_buffer[it++] = payload_size;
  checksum += payload_size;

  for (int i = 0; i < payload_size; i++) {
    out_buffer[it++] = msg->payload[i];
    checksum += msg->payload[i];
  }

  out_buffer[it] = checksum;

  serial_port_->sendCharArray(out_buffer, payload_size + 3);
}

//}

/* callbackSendRawMessage() //{ */

void BacaProtocol::callbackSendRawMessage(const mrs_msgs::SerialRawConstPtr &msg) {

  uint8_t payload_size = msg->payload.size();
  uint8_t out_buffer[payload_size];

  for (unsigned int i = 0; i < payload_size; i++) {

    ROS_INFO_STREAM("SENDING " << std::hex << msg->payload[i]);
    serial_port_->sendCharArray(out_buffer, payload_size);

  }
}

//}

// | ------------------------ routines ------------------------ |

/* processMessage() //{ */

void BacaProtocol::processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct) {

  if (payload_size == 3 && (input_buffer[0] == 0x00 || input_buffer[0] == 0x01) && checksum_correct) {
    /* Special message reserved for garmin rangefinder */
    received_msg_ok_garmin++;
    uint8_t message_id = input_buffer[0];
    int16_t range      = input_buffer[1] << 8;
    range |= input_buffer[2];

    sensor_msgs::Range range_msg;
    range_msg.field_of_view  = 0.0523599;  // +-3 degree
    range_msg.max_range      = MAX_RANGE * 0.01;
    range_msg.min_range      = MIN_RANGE * 0.01;
    range_msg.radiation_type = sensor_msgs::Range::INFRARED;
    range_msg.header.stamp   = ros::Time::now();

    range_msg.range = range * 0.01;  // convert to m

    if (range > MAX_RANGE) {
      range_msg.range = std::numeric_limits<double>::infinity();
    } else if (range < MIN_RANGE) {
      range_msg.range = -std::numeric_limits<double>::infinity();
    }

    if (message_id == 0x00) {
      range_msg.header.frame_id = garmin_A_frame_;
      range_publisher_A_.publish(range_msg);
    } else if (message_id == 0x01) {
      range_msg.header.frame_id = garmin_B_frame_;
      range_publisher_B_.publish(range_msg);
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
    baca_protocol_publisher_.publish(msg);
  }
}

//}

/* releaseSerialLine() //{ */

void BacaProtocol::releaseSerialLine(void) {

  delete serial_port_;
}

//}

/* connectToSensors() //{ */

uint8_t BacaProtocol::connectToSensor(void) {

  // Create serial port
  serial_port_ = new serial_device::SerialPort();

  // Set callback function for the serial ports
  serial_data_callback_function_ = boost::bind(&BacaProtocol::callbackSerialData, this, _1);
  serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);

  // Connect serial port
  ROS_INFO_THROTTLE(1.0, "[%s]: Openning the serial port.", ros::this_node::getName().c_str());
  if (!serial_port_->connect(portname_)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not connect to sensor.", ros::this_node::getName().c_str());
    return 0;
  }

  ROS_INFO_THROTTLE(1.0, "[%s]: Connected to sensor.", ros::this_node::getName().c_str());

  lastReceived = ros::Time::now();

  return 1;
}

//}

/* main() //{ */

int main(int argc, char **argv) {

  ros::init(argc, argv, "BacaProtocol");

  BacaProtocol serial_line;

  ros::Rate loop_rate(100);

  while (ros::ok()) {

    // check whether the teraranger stopped sending data
    ros::Duration interval  = ros::Time::now() - serial_line.lastReceived;
    ros::Duration interval2 = ros::Time::now() - serial_line.lastPrinted;

    if (interval2.toSec() > 1.0) {
      ROS_INFO_STREAM_THROTTLE(1.0, "Got msgs - Garmin: " << serial_line.received_msg_ok_garmin << " Generic msg: " << serial_line.received_msg_ok
                                                          << "  Wrong checksum: " << serial_line.received_msg_bad_checksum << "; in the last "
                                                          << interval2.toSec() << " s");
      serial_line.received_msg_ok_garmin    = 0;
      serial_line.received_msg_ok           = 0;
      serial_line.received_msg_bad_checksum = 0;
      serial_line.lastPrinted               = ros::Time::now();
    }

    if (interval.toSec() > MAXIMAL_TIME_INTERVAL && serial_line.use_timeout) {

      serial_line.releaseSerialLine();

      ROS_WARN_THROTTLE(1.0, "[%s]: Serial device not responding, resetting connection...", ros::this_node::getName().c_str());

      // if establishing the new connection was successfull
      if (serial_line.connectToSensor() == 1) {

        ROS_INFO_THROTTLE(1.0, "[%s]: New connection to Serial device was established.", ros::this_node::getName().c_str());
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

//}
