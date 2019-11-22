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

#include "serial_port.h"

#define BUFFER_SIZE 256

#define MAXIMAL_TIME_INTERVAL 100

/* class MrsSerial //{ */

class MrsSerial {
public:
  MrsSerial();

  void callbackSerialData(uint8_t data);

  enum serial_receiver_state
  {
    WAITING_FOR_DOLLAR,
    WAITING_FOR_GNGGA,
    RECEIVING_MESSAGE,
  };

  uint8_t connectToSensor(void);
  void    releaseSerialLine(void);
  void    processMessage(uint8_t *input_buffer, uint16_t buffer_conter);

  ros::Time lastReceived;
  ros::Time lastPrinted;

  ros::NodeHandle nh_;

  ros::Publisher rtk_publisher;

  serial_device::SerialPort *    serial_port_;
  boost::function<void(uint8_t)> serial_data_callback_function_;

  bool     use_timeout;
  uint16_t received_msg_ok           = 0;
  uint16_t received_msg_ok_garmin    = 0;
  uint16_t received_msg_bad_checksum = 0;

  std::string portname_;

  std::mutex mutex_msg;
};

//}

/* MrsSerial() //{ */

MrsSerial::MrsSerial() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  nh_.param("portname", portname_, std::string("/dev/ttyUSB0"));
  nh_.param("use_timeout", use_timeout, true);

  // Publishers
  /* rtk_publisher         = nh_.advertise<sensor_msgs::Range>("range", 1); */

  // Output loaded parameters to console for double checking
  ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());
  lastReceived = ros::Time::now();
  lastPrinted  = ros::Time::now();

  connectToSensor();
}

//}

// | ------------------------ callbacks ------------------------ |

/* callbackSerialData() //{ */

void MrsSerial::callbackSerialData(uint8_t single_character) {

  static serial_receiver_state rec_state = WAITING_FOR_DOLLAR;
  static uint8_t               input_buffer[BUFFER_SIZE];
  static uint16_t              buffer_counter = 0;

  switch (rec_state) {
    case WAITING_FOR_DOLLAR:

      if (single_character == '$') {
        buffer_counter = 0;
        rec_state      = WAITING_FOR_GNGGA;
      }
      break;

    case WAITING_FOR_GNGGA:

      input_buffer[buffer_counter] = single_character;
      buffer_counter++;

      if (buffer_counter == 5) {
        if (input_buffer[0] == 'G' && input_buffer[1] == 'N' && input_buffer[2] == 'G' && input_buffer[3] == 'G' && input_buffer[4] == 'A') {
          ROS_INFO("Got one!");
          rec_state = RECEIVING_MESSAGE;
        } else {
          ROS_INFO("Got shit!");
          rec_state = WAITING_FOR_DOLLAR;
        }
      }
      break;

    case RECEIVING_MESSAGE:

      if (single_character == '\n') {
        processMessage(input_buffer, buffer_counter);
        rec_state = WAITING_FOR_DOLLAR;
      } else {
        input_buffer[buffer_counter] = single_character;
        buffer_counter++;
      }
      break;
  }
}

//}

// | ------------------------ routines ------------------------ |

/* processMessage() //{ */

void MrsSerial::processMessage(uint8_t *input_buffer, uint16_t buffer_counter) {
  std::string s = "";
  for (int i = 0; i < buffer_counter; i++) {
    s = s + (char)input_buffer[i];
  }
  ROS_INFO_STREAM("it is: " << s);
}

//}

/* releaseSerialLine() //{ */

void MrsSerial::releaseSerialLine(void) {

  delete serial_port_;
}

//}

/* connectToSensors() //{ */

uint8_t MrsSerial::connectToSensor(void) {

  // Create serial port
  serial_port_ = new serial_device::SerialPort();

  // Set callback function for the serial ports
  serial_data_callback_function_ = boost::bind(&MrsSerial::callbackSerialData, this, _1);
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

  ros::init(argc, argv, "MrsSerial");

  MrsSerial serial_line;

  ros::Rate loop_rate(100);

  while (ros::ok()) {

    // check whether the teraranger stopped sending data
    ros::Duration interval  = ros::Time::now() - serial_line.lastReceived;
    ros::Duration interval2 = ros::Time::now() - serial_line.lastPrinted;

    if (interval2.toSec() > 1.0) {
      /* ROS_INFO_STREAM("Got msgs - Garmin: " << serial_line.received_msg_ok_garmin << " Generic msg: " << serial_line.received_msg_ok */
      /*                                       << "  Wrong checksum: " << serial_line.received_msg_bad_checksum << "; in the last " << interval2.toSec() << "
       * s"); */
      serial_line.received_msg_ok_garmin    = 0;
      serial_line.received_msg_ok           = 0;
      serial_line.received_msg_bad_checksum = 0;
      serial_line.lastPrinted               = ros::Time::now();
    }

    if (interval.toSec() > MAXIMAL_TIME_INTERVAL && serial_line.use_timeout) {

      serial_line.releaseSerialLine();

      ROS_WARN("[%s]: Serial device not responding, resetting connection...", ros::this_node::getName().c_str());

      // if establishing the new connection was successfull
      if (serial_line.connectToSensor() == 1) {

        ROS_INFO("[%s]: New connection to Serial device was established.", ros::this_node::getName().c_str());
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

//}
