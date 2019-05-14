#include <ros/package.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Char.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <mutex>

#include <string>
#include <mrs_msgs/BacaProtocol.h>

#include "serial_port.h"

#define BUFFER_SIZE 256

// for garmin
#define MAXIMAL_TIME_INTERVAL 1
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

  uint8_t connectToSensor(void);
  void    releaseSerialLine(void);
  void    processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, bool checksum_correct);

  ros::Time lastReceived;

  ros::NodeHandle nh_;

  ros::Publisher  range_publisher_;
  ros::Publisher  range_publisher_up_;
  ros::Publisher  baca_protocol_publisher_;

  ros::Subscriber baca_protocol_subscriber;

  serial_device::SerialPort *    serial_port_;
  boost::function<void(uint8_t)> serial_data_callback_function_;

  std::string portname_;

  std::mutex mutex_msg;
};

//}

/* BacaProtocol() //{ */

BacaProtocol::BacaProtocol() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  nh_.param("portname", portname_, std::string("/dev/ttyUSB0"));

  // Publishers
  range_publisher_         = nh_.advertise<sensor_msgs::Range>("range", 1);
  range_publisher_up_      = nh_.advertise<sensor_msgs::Range>("range_up", 1);
  baca_protocol_publisher_ = nh_.advertise<mrs_msgs::BacaProtocol>("baca_protocol_out", 1);

  baca_protocol_subscriber = nh_.subscribe("baca_protocol_in", 1, &BacaProtocol::callbackSendMessage, this, ros::TransportHints().tcpNoDelay());

  // service out
  netgun_arm  = nh_.advertiseService("netgun_arm", &BacaProtocol::callbackNetgunArm, this);
  netgun_safe = nh_.advertiseService("netgun_safe", &BacaProtocol::callbackNetgunSafe, this);
  netgun_fire = nh_.advertiseService("netgun_fire", &BacaProtocol::callbackNetgunFire, this);

  // Output loaded parameters to console for double checking
  ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());
  lastReceived = ros::Time::now();

  connectToSensor();
}

//}

// | ------------------------ callbacks ------------------------ |

/*  callbackNetgunSafe()//{ */

bool BacaProtocol::callbackNetgunSafe([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  mrs_msgs::BacaProtocol msg;
  msg.stamp = ros::Time::now();
  msg.payload.push_back('7');

  mrs_msgs::BacaProtocolConstPtr const_msg(new mrs_msgs::BacaProtocol(msg));
  callbackSendMessage(const_msg);
  
  ROS_INFO("Safing net gun");
  res.message = "Safing net gun";
  res.success = true;

  return true;
}

//}

/* callbackNetgunArm() //{ */

bool BacaProtocol::callbackNetgunArm([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  mrs_msgs::BacaProtocol msg;
  msg.stamp = ros::Time::now();
  msg.payload.push_back('8');

  mrs_msgs::BacaProtocolConstPtr const_msg(new mrs_msgs::BacaProtocol(msg));
  callbackSendMessage(const_msg);
  
  ROS_INFO("Arming net gun");
  res.message = "Arming net gun";
  res.success = true;

  return true;
}

//}

/* callbackNetgunFire() //{ */

bool BacaProtocol::callbackNetgunFire([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  
  mrs_msgs::BacaProtocol msg;
  msg.stamp = ros::Time::now();
  msg.payload.push_back('9');

  mrs_msgs::BacaProtocolConstPtr const_msg(new mrs_msgs::BacaProtocol(msg));
  callbackSendMessage(const_msg);
  
  ROS_INFO("Firing net gun");
  res.message = "Firing net gun";
  res.success = true;
  return true;
}

//}

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
        ROS_ERROR("[%s]: Message with 0 payload_size received, discarding.", ros::this_node::getName().c_str());
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
        processMessage(payload_size, input_buffer, checksum, true);
        lastReceived = ros::Time::now();
        rec_state    = WAITING_FOR_MESSSAGE;
      } else {
        ROS_ERROR_STREAM("[ " << ros::this_node::getName().c_str() << "]: Message with bad checksum received, received: " << static_cast<unsigned>(single_character)
                              << ", calculated: " << static_cast<unsigned>(checksum));
        rec_state = WAITING_FOR_MESSSAGE;
      }
      break;
  }
}

//}

/* callbackSendMessage() //{ */

void BacaProtocol::callbackSendMessage(const mrs_msgs::BacaProtocolConstPtr &msg) {

  std::scoped_lock lock(mutex_msg);
  uint8_t          payload_size = msg->payload.size();
  uint8_t          tmp_send     = 'b';
  uint8_t          checksum     = 0;

  serial_port_->sendChar(tmp_send);
  checksum += tmp_send;

  serial_port_->sendChar(payload_size);
  checksum += payload_size;

  for (int i = 0; i < payload_size; i++) {
    tmp_send = msg->payload[i];
    checksum += tmp_send;
    serial_port_->sendChar(tmp_send);
  }
  serial_port_->sendChar(checksum);
}

//}

// | ------------------------ routines ------------------------ |

/* processMessage() //{ */

void BacaProtocol::processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, bool checksum_correct) {

  if (payload_size == 3 && (input_buffer[0] == 0x00 || input_buffer[0] == 0x01) && checksum_correct) {
    /* Special message reserved for garmin rangefinder */
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
      range_msg.header.frame_id = "garmin_frame";
      range_publisher_.publish(range_msg);
    } else if (message_id == 0x01) {
      range_msg.header.frame_id = "garmin_frame_up";
      range_publisher_up_.publish(range_msg);
    }
  } else {
    /* General serial message */
    mrs_msgs::BacaProtocol msg;
    msg.stamp = ros::Time::now();
    for (uint8_t i = 0; i < payload_size; i++) {
      msg.payload.push_back(input_buffer[i]);
    }
    msg.checksum         = checksum;
    msg.checksum_correct = checksum_correct;
    ROS_INFO("[%s]: published", ros::this_node::getName().c_str());
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
  ROS_INFO("[%s]: Openning the serial port.", ros::this_node::getName().c_str());
  if (!serial_port_->connect(portname_)) {
    ROS_ERROR("[%s]: Could not connect to sensor.", ros::this_node::getName().c_str());
    return 0;
  }

  ROS_INFO("[%s]: Connected to sensor.", ros::this_node::getName().c_str());

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
    ros::Duration interval = ros::Time::now() - serial_line.lastReceived;
    if (interval.toSec() > MAXIMAL_TIME_INTERVAL) {

      serial_line.releaseSerialLine();

      ROS_WARN("[%s]: BacaProtocol not responding, resetting connection...", ros::this_node::getName().c_str());

      // if establishing the new connection was successfull
      if (serial_line.connectToSensor() == 1) {

        ROS_INFO("[%s]: New connection to BacaProtocol was established.", ros::this_node::getName().c_str());
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

//}
