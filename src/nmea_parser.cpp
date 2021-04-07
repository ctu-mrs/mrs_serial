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

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#define MAXIMAL_TIME_INTERVAL 5

namespace nmea_parser
{

typedef enum
{
  SINGLE,
  PSRDIFF,
  L1_INT,
  L1_FLOAT,
  NONE
} rtk_state;

/* class NmeaParser //{ */

class NmeaParser : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  void interpretSerialData(uint8_t data);
  void callbackSerialTimer(const ros::TimerEvent& event);
  void callbackMaintainerTimer(const ros::TimerEvent& event);

  enum serial_receiver_state
  {
    WAITING_FOR_DOLLAR,
    RECEIVING,
  };

  serial_receiver_state state_ = WAITING_FOR_DOLLAR;

  std::string msg_;

  uint8_t connectToSensor(void);
  void    processMessage();
  void    stringTimer(const ros::TimerEvent& event);

  void processGPGGA(std::vector<std::string>& results);

  ros::NodeHandle nh_;

  ros::Publisher gpgga_pub_;
  ros::Publisher bestpos_pub_;
  ros::Publisher baca_protocol_publisher_;
  ros::Publisher status_string_publisher_;

  ros::Timer string_timer_;
  ros::Timer serial_timer_;
  ros::Timer maintainer_timer_;

  serial_port::SerialPort serial_port_;

  rtk_state rtk_state_ = NONE;

  bool     publish_bad_checksum;
  bool     use_timeout;
  bool     swap_garmins;
  uint16_t received_msg_ok           = 0;
  uint16_t received_msg_ok_garmin    = 0;
  uint16_t received_msg_bad_checksum = 0;

  int serial_rate_        = 500;
  int serial_buffer_size_ = 1024;

  std::string portname_;
  int baudrate_;
  std::string uav_name_;
  std::string garmin_A_frame_;
  std::string garmin_B_frame_;

  std::mutex mutex_msg;

  int msg_counter_ = 0;

  ros::Time last_received_;
  ros::Time interval_;

  bool is_connected_   = false;
  bool is_initialized_ = false;
};

//}

/* onInit() //{ */

void NmeaParser::onInit() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  nh_.param("uav_name", uav_name_, std::string("uav"));
  nh_.param("portname", portname_, std::string("/dev/ttyUSB0"));
  nh_.param("baudrate", baudrate_);
  nh_.param("serial_rate", serial_rate_, 500);
  nh_.param("serial_buffer_size", serial_buffer_size_, 1024);

  gpgga_pub_               = nh_.advertise<mrs_msgs::Gpgga>("gpgga_out", 1);
  bestpos_pub_             = nh_.advertise<mrs_msgs::Bestpos>("bestpos_out", 1);
  status_string_publisher_ = nh_.advertise<std_msgs::String>("string_out", 1);

  string_timer_     = nh_.createTimer(ros::Rate(1), &NmeaParser::stringTimer, this);
  serial_timer_     = nh_.createTimer(ros::Rate(serial_rate_), &NmeaParser::stringTimer, this);
  maintainer_timer_ = nh_.createTimer(ros::Rate(1), &NmeaParser::stringTimer, this);

  // Output loaded parameters to console for double checking
  ROS_INFO_THROTTLE(1.0, "[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO_THROTTLE(1.0, "[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());
  ROS_INFO_THROTTLE(1.0, "[%s] baudrate: %i", ros::this_node::getName().c_str(), baudrate_);
  ROS_INFO_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName().c_str() << "] publishing messages with wrong checksum: " << publish_bad_checksum);

  connectToSensor();

  serial_timer_     = nh_.createTimer(ros::Rate(serial_rate_), &NmeaParser::callbackSerialTimer, this);
  maintainer_timer_ = nh_.createTimer(ros::Rate(1), &NmeaParser::callbackMaintainerTimer, this);

  is_initialized_ = true;
}

//}

// | ------------------------ callbacks ------------------------ |

/* callbackSerialRead() //{ */

void NmeaParser::callbackSerialTimer(const ros::TimerEvent& event) {

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

void NmeaParser::callbackMaintainerTimer(const ros::TimerEvent& event) {

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

    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "] Got " << msg_counter_ << " GPGGA/GNGGA messages in last "
                         << (ros::Time::now() - interval_).toSec() << " s");
    msg_counter_ = 0;
    interval_    = ros::Time::now();

  } else {

    connectToSensor();
  }
}

//}

/* interpretSerialData() //{ */

void NmeaParser::interpretSerialData(uint8_t single_character) {

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


  last_received_ = ros::Time::now();
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
}

//}

/* processGPGGA() //{ */

void NmeaParser::processGPGGA(std::vector<std::string>& results) {

  mrs_msgs::Gpgga     gpgga_msg;
  mrs_msgs::Bestpos   bestpos_msg;
  mrs_msgs::GpsStatus gps_status;

  try {

    gpgga_msg.utc_seconds      = stod(results[1]);
    std::string lat            = results[2];
    gpgga_msg.latitude         = stod(lat.substr(0, 2)) + stod(lat.substr(2, 10)) / 60;
    gpgga_msg.latitude_dir     = results[3];
    std::string lon            = results[4];
    gpgga_msg.longitude        = stod(lon.substr(0, 3)) + stod(lon.substr(3, 10)) / 60;
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
    msg_counter_++;
  }
  catch (...) {
    ROS_ERROR("[Nmea parser]: exception caught during publishing");
  }
}

//}

/* connectToSensors() //{ */

uint8_t NmeaParser::connectToSensor(void) {

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

}  // namespace nmea_parser

PLUGINLIB_EXPORT_CLASS(nmea_parser::NmeaParser, nodelet::Nodelet);
