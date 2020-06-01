#include <ros/package.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <mutex>

#include <mrs_msgs/Gpgga.h>
#include <mrs_msgs/Bestpos.h>
#include <mrs_msgs/GpsStatus.h>

#include <std_msgs/String.h>

#include <boost/algorithm/string.hpp>

#include "serial_port.h"

#define MAXIMAL_TIME_INTERVAL 5

typedef enum
{
  SINGLE,
  PSRDIFF,
  L1_INT,
  L1_FLOAT,
  NONE
} rtk_state;

/* class NmeaParser //{ */

class NmeaParser {
public:
  NmeaParser();

  void callbackSerialData(uint8_t data);

  enum serial_receiver_state
  {
    WAITING_FOR_DOLLAR,
    RECEIVING,
  };

  serial_receiver_state state_ = WAITING_FOR_DOLLAR;

  std::string msg_;

  uint8_t connectToSensor(void);
  void    releaseSerialLine(void);
  void    processMessage();
  void    stringTimer(const ros::TimerEvent& event);

  void processGPGGA(std::vector<std::string>& results);

  ros::Time lastReceived;
  ros::Time lastPrinted;

  ros::NodeHandle nh_;

  ros::Publisher gpgga_pub_;
  ros::Publisher bestpos_pub_;
  ros::Publisher baca_protocol_publisher_;
  ros::Publisher status_string_publisher_;

  ros::Subscriber raw_message_subscriber;
  ros::Subscriber baca_protocol_subscriber;
  ros::Subscriber magnet_subscriber;

  ros::Timer string_timer_;

  serial_device::SerialPort*     serial_port_;
  boost::function<void(uint8_t)> serial_data_callback_function_;

  rtk_state rtk_state_ = NONE;

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

/* NmeaParser() //{ */

NmeaParser::NmeaParser() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  nh_.param("uav_name", uav_name_, std::string("uav"));
  nh_.param("portname", portname_, std::string("/dev/ttyUSB0"));

  gpgga_pub_               = nh_.advertise<mrs_msgs::Gpgga>("gpgga_out", 1);
  bestpos_pub_             = nh_.advertise<mrs_msgs::Bestpos>("bestpos_out", 1);
  status_string_publisher_ = nh_.advertise<std_msgs::String>("string_out", 1);

  string_timer_ = nh_.createTimer(ros::Rate(1), &NmeaParser::stringTimer, this);

  // Publishers
  /* std::string postfix_A = swap_garmins ? "_up" : ""; */
  /* std::string postfix_B = swap_garmins ? "" : "_up"; */
  /* range_publisher_A_    = nh_.advertise<sensor_msgs::Range>("range" + postfix_A, 1); */
  /* range_publisher_B_    = nh_.advertise<sensor_msgs::Range>("range" + postfix_B, 1); */
  /* garmin_A_frame_       = uav_name_ + "/garmin" + postfix_A; */
  /* garmin_B_frame_       = uav_name_ + "/garmin" + postfix_B; */


  /* baca_protocol_subscriber = nh_.subscribe("baca_protocol_in", 1, &NmeaParser::callbackSendMessage, this, ros::TransportHints().tcpNoDelay()); */

  /* raw_message_subscriber = nh_.subscribe("raw_in", 1, &NmeaParser::callbackSendRawMessage, this, ros::TransportHints().tcpNoDelay()); */

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

void NmeaParser::callbackSerialData(uint8_t single_character) {

  ROS_INFO_STREAM_THROTTLE(1.0, "State:  " << state_);

  if (state_ == RECEIVING) {

    if (single_character == '\n') {

      ROS_INFO_STREAM(msg_);
      processMessage();
      msg_   = "";
      state_ = WAITING_FOR_DOLLAR;

    } else {

      msg_ += single_character;
    }
  }

  if (state_ == WAITING_FOR_DOLLAR) {

    if (single_character == '$') {

      state_ = RECEIVING;
    }
  }


  lastReceived = ros::Time::now();
}

//}

// | ------------------------ routines ------------------------ |

/* stringTimer() //{ */

void NmeaParser::stringTimer([[maybe_unused]] const ros::TimerEvent& event) {
  std_msgs::String msg;

  switch (rtk_state_) {
    case SINGLE:
      msg.data = "-y RTK: SINGLE";
      break;
    case PSRDIFF:
      msg.data = "-y RTK: PSRDIFF";
      break;
    case L1_INT:
      msg.data = "-g RTK: L1_INT";
      break;
    case L1_FLOAT:
      msg.data = "-y RTK: L1_FLOAT";
      break;
    default:
      msg.data = "-r RTK: NONE";
      break;
  }

  try {
    status_string_publisher_.publish(msg);
  }
  catch (...) {
    ROS_ERROR("[Nmea Parser]: exception caught during publishing topic ");
  }
}

//}

/* processMessage() //{ */

void NmeaParser::processMessage() {

  std::vector<std::string> results;

  boost::split(results, msg_, [](char c) { return c == ','; });  // split the input string into words and put them in results vector

  if (results[0] == "GPGGA" || results[0] == "GNGGA") {
    processGPGGA(results);
  }

  /* for (unsigned long i = 0; i < results.size(); i++) { */
  /*   ROS_INFO_STREAM(results[i]); */
  /* } */
}

//}

/* processGPGGA() //{ */

void NmeaParser::processGPGGA(std::vector<std::string>& results) {

  mrs_msgs::Gpgga     gpgga_msg;
  mrs_msgs::Bestpos   bestpos_msg;
  mrs_msgs::GpsStatus gps_status;

  try {

    gpgga_msg.utc_seconds      = stod(results[1]);
    gpgga_msg.latitude         = stod(results[2]);
    gpgga_msg.latitude_dir     = results[3];
    gpgga_msg.longitude        = stod(results[4]);
    gpgga_msg.longitude_dir    = results[5];
    gps_status.quality         = stoi(results[6]);
    gpgga_msg.gps_quality      = gps_status;
    gpgga_msg.num_sats         = stoi(results[7]);
    gpgga_msg.hdop             = stod(results[8]);
    gpgga_msg.altitude         = stod(results[9]);
    gpgga_msg.altitude_units   = results[10];
    gpgga_msg.undulation       = stod(results[11]);
    gpgga_msg.undulation_units = results[12];

    if (results[13] == "") {
      gpgga_msg.diff_age = 0;
    } else {
      gpgga_msg.diff_age = stoi(results[13]);
    }

    std::vector<std::string> results2;
    boost::split(results2, results[14], [](char c) { return c == '*'; });  // split the input string into words and put them in results vector

    gpgga_msg.station_id = results2[0];
  }

  catch (const std::invalid_argument& e) {

    ROS_ERROR("Invalid argument exception in processGPGGA");
  }

  bestpos_msg.latitude               = gpgga_msg.latitude;
  bestpos_msg.longitude              = gpgga_msg.longitude;
  bestpos_msg.height                 = gpgga_msg.altitude;
  bestpos_msg.undulation             = gpgga_msg.undulation;
  bestpos_msg.diff_age               = gpgga_msg.diff_age;
  bestpos_msg.num_satellites_tracked = gpgga_msg.num_sats;

  switch (gpgga_msg.gps_quality.quality) {
    case 1:
      bestpos_msg.position_type = "SINGLE";
      rtk_state_                = SINGLE;
      break;
    case 2:
      bestpos_msg.position_type = "PSRDIFF";
      rtk_state_                = PSRDIFF;
      break;
    case 4:
      bestpos_msg.position_type = "L1_INT";
      rtk_state_                = L1_INT;
      break;
    case 5:
      bestpos_msg.position_type = "L1_FLOAT";
      rtk_state_                = L1_FLOAT;
      break;
    default:
      bestpos_msg.position_type = "NONE";
      rtk_state_                = NONE;
      break;
  }
  try {
    gpgga_pub_.publish(gpgga_msg);
    bestpos_pub_.publish(bestpos_msg);
  }
  catch (...) {
    ROS_ERROR("[Nmea parser]: exception caught during publishing");
  }
}

//}

/* releaseSerialLine() //{ */

void NmeaParser::releaseSerialLine(void) {

  delete serial_port_;
}

//}

/* connectToSensors() //{ */

uint8_t NmeaParser::connectToSensor(void) {

  // Create serial port
  serial_port_ = new serial_device::SerialPort();

  // Set callback function for the serial ports
  serial_data_callback_function_ = boost::bind(&NmeaParser::callbackSerialData, this, _1);
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

int main(int argc, char** argv) {

  ros::init(argc, argv, "NmeaParser");

  NmeaParser serial_line;

  ros::Rate loop_rate(100);

  while (ros::ok()) {

    // check whether the teraranger stopped sending data
    ros::Duration interval  = ros::Time::now() - serial_line.lastReceived;
    ros::Duration interval2 = ros::Time::now() - serial_line.lastPrinted;

    if (interval2.toSec() > 1.0) {
      serial_line.lastPrinted = ros::Time::now();
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
