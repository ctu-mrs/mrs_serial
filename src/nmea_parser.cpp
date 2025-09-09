#include <ros/package.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <mutex>

#include <mrs_modules_msgs/Gpgga.h>
#include <mrs_modules_msgs/Gpgsa.h>
#include <mrs_modules_msgs/Gpgst.h>
#include <mrs_modules_msgs/Gpvtg.h>

#include <mrs_msgs/StringStamped.h>

#include <mrs_modules_msgs/Bestpos.h>
#include <mrs_modules_msgs/GpsStatus.h>

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
  void processGPGSA(std::vector<std::string>& results);
  void processGPGST(std::vector<std::string>& results);
  void processGPVTG(std::vector<std::string>& results);


  double stodSafe(std::string& string_in);
  int    stoiSafe(std::string& string_in);

  ros::NodeHandle nh_;

  ros::Publisher gpgga_pub_;
  ros::Publisher gpgsa_pub_;
  ros::Publisher gpgst_pub_;
  ros::Publisher gpvtg_pub_;
  ros::Publisher bestpos_pub_;
  ros::Publisher string_pub_;
  ros::Publisher string_raw_pub_;
  ros::Publisher baca_protocol_publisher_;

  ros::Timer string_timer_;
  ros::Timer serial_timer_;
  ros::Timer maintainer_timer_;

  mrs_modules_msgs::Bestpos bestpos_msg_;

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
  int         baudrate_;
  std::string uav_name_;
  std::string garmin_A_frame_;
  std::string garmin_B_frame_;

  std::mutex mutex_msg;

  int msg_counter_gpgga_ = 0;
  int msg_counter_gpgsa_ = 0;
  int msg_counter_gpgst_ = 0;
  int msg_counter_gpvtg_ = 0;

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
  nh_.param("baudrate", baudrate_, 115200);
  nh_.param("serial_rate", serial_rate_, 500);
  nh_.param("serial_buffer_size", serial_buffer_size_, 1024);

  gpgga_pub_               = nh_.advertise<mrs_modules_msgs::Gpgga>("gpgga_out", 1);
  gpgsa_pub_               = nh_.advertise<mrs_modules_msgs::Gpgsa>("gpgsa_out", 1);
  gpgst_pub_               = nh_.advertise<mrs_modules_msgs::Gpgst>("gpgst_out", 1);
  gpvtg_pub_               = nh_.advertise<mrs_modules_msgs::Gpvtg>("gpvtg_out", 1);
  bestpos_pub_             = nh_.advertise<mrs_modules_msgs::Bestpos>("bestpos_out", 1);
  string_pub_              = nh_.advertise<std_msgs::String>("status_out", 1);
  string_raw_pub_          = nh_.advertise<mrs_msgs::StringStamped>("raw_out", 1);

  bestpos_msg_.diff_age = 9999;

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

    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "] Got " << msg_counter_gpgga_ << " GPGGA, " << msg_counter_gpgsa_ << " GPGSA, "
                        << msg_counter_gpgst_ << " GPGST, " << msg_counter_gpvtg_ << " GPVTG messages in last " << (ros::Time::now() - interval_).toSec()
                        << " s");
    msg_counter_gpgga_ = 0;
    msg_counter_gpgsa_ = 0;
    msg_counter_gpgst_ = 0;
    msg_counter_gpvtg_ = 0;
    interval_          = ros::Time::now();

  } else {

    connectToSensor();
  }
}

//}

/* interpretSerialData() //{ */

void NmeaParser::interpretSerialData(uint8_t single_character) {

  if (state_ == RECEIVING) {

    if (single_character == '\n') {

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

/* processMessage() //{ */

void NmeaParser::processMessage() {

  mrs_msgs::StringStamped string_raw_out;
  string_raw_out.header.stamp = ros::Time::now();
  string_raw_out.data         = msg_;

  try {
    string_raw_pub_.publish(string_raw_out);
    /* ROS_INFO_STREAM("[NmeaParser]: " << msg_); */
  }
  catch (...) {
    ROS_ERROR("[Nmea parser]: exception caught during publishing");
  }


  std::vector<std::string> results;
  boost::split(results, msg_, [](char c) { return c == ','; });  // split the input string into words and put them in results vector

  if (results[0] == "GPGGA" || results[0] == "GNGGA") {
    /* ROS_INFO_STREAM("[NmeaParser]: GPGGA "); */
    processGPGGA(results);
  }

  if (results[0] == "GPGSA" || results[0] == "GNGSA") {
    /* ROS_INFO_STREAM("[NmeaParser]: GPGSA "); */
    processGPGSA(results);
  }

  if (results[0] == "GPGST" || results[0] == "GNGST") {
    /* ROS_INFO_STREAM("[NmeaParser]: GPGST "); */
    processGPGST(results);
  }

  if (results[0] == "GPVTG" || results[0] == "GNVTG") {
    /* ROS_INFO_STREAM("[NmeaParser]: GPVTG "); */
    processGPVTG(results);
  }
}

//}

/* processGPGGA() //{ */

void NmeaParser::processGPGGA(std::vector<std::string>& results) {

  mrs_modules_msgs::Gpgga     gpgga_msg;
  mrs_modules_msgs::GpsStatus gps_status;

  /* for (size_t i = 0; i < results.size(); i++) { */
  /*   ROS_INFO_STREAM("[NmeaParser]:  " << i << "  " << results[i]); */
  /* } */


  gpgga_msg.header.stamp    = ros::Time::now();
  bestpos_msg_.header.stamp = ros::Time::now();

  gpgga_msg.utc_seconds = stodSafe(results[1]);

  std::string lat    = results[2];
  std::string sub_s1 = "";
  std::string sub_s2 = "";
  std::string sub_s3 = "";
  std::string sub_s4 = "";
  if (lat != "") {
    sub_s1 = lat.substr(0, 2);
    sub_s2 = lat.substr(2, 10);
  }

  gpgga_msg.latitude     = stodSafe(sub_s1) + stodSafe(sub_s2) / 60;
  gpgga_msg.latitude_dir = results[3];

  if (results[3] == "S") {
    gpgga_msg.latitude *= -1;
  }

  std::string lon = results[4];
  if (lon != "") {
    sub_s3 = lon.substr(0, 3);
    sub_s4 = lon.substr(3, 10);
  }

  gpgga_msg.longitude        = stodSafe(sub_s3) + stodSafe(sub_s4) / 60;
  gpgga_msg.longitude_dir    = results[5];

  if (results[5] == "W") {
    gpgga_msg.longitude *= -1;
  }

  gps_status.quality         = stoiSafe(results[6]);
  gpgga_msg.gps_quality      = gps_status;
  gpgga_msg.num_sats         = stoiSafe(results[7]);
  gpgga_msg.hdop             = stodSafe(results[8]);
  gpgga_msg.altitude         = stodSafe(results[9]);
  gpgga_msg.altitude_units   = results[10];
  gpgga_msg.undulation       = stodSafe(results[11]);
  gpgga_msg.undulation_units = results[12];

  if (results[13] == "") {
    gpgga_msg.diff_age = 9999;
  } else {
    gpgga_msg.diff_age = stoiSafe(results[13]);
  }

  std::vector<std::string> results_checksum_remove;
  boost::split(results_checksum_remove, results[14], [](char c) { return c == '*'; });
  if(results_checksum_remove[0] == ""){
    // no basestation ID in GPGGA msg => no corrections are incoming
    gpgga_msg.diff_age = 9999;
  }




  std::vector<std::string> results2;
  boost::split(results2, results[14], [](char c) { return c == '*'; });  // split the input string into words and put them in results vector

  gpgga_msg.station_id = results2[0];

  bestpos_msg_.latitude               = gpgga_msg.latitude;
  bestpos_msg_.longitude              = gpgga_msg.longitude;
  bestpos_msg_.height                 = gpgga_msg.altitude;
  bestpos_msg_.undulation             = gpgga_msg.undulation;
  bestpos_msg_.diff_age               = gpgga_msg.diff_age;
  bestpos_msg_.num_satellites_tracked = gpgga_msg.num_sats;

  std_msgs::String string_msg;

  switch (gpgga_msg.gps_quality.quality) {
    case 1:
      bestpos_msg_.position_type = "SINGLE";
      string_msg.data            = "-y RTK: SINGLE";
      rtk_state_                 = SINGLE;
      break;
    case 2:
      bestpos_msg_.position_type = "PSRDIFF";
      string_msg.data            = "-y RTK: PSRDIFF";
      rtk_state_                 = PSRDIFF;
      break;
    case 4:
      bestpos_msg_.position_type = "L1_INT";
      string_msg.data            = "-g RTK: L1_INT";
      rtk_state_                 = L1_INT;
      break;
    case 5:
      bestpos_msg_.position_type = "L1_FLOAT";
      string_msg.data            = "-y RTK: L1_FLOAT";
      rtk_state_                 = L1_FLOAT;
      break;
    default:
      bestpos_msg_.position_type = "NONE";
      string_msg.data            = "-r RTK: NONE";
      rtk_state_                 = NONE;
      break;
  }

  ROS_INFO_STREAM_THROTTLE(1.0, "[NmeaParser]: " << string_msg.data.substr(3));

  double diff_age = bestpos_msg_.diff_age;
  if (diff_age > 99.9) {
    diff_age = 99.9;
  }
  std::stringstream stream;
  stream << std::fixed << std::setprecision(2) << diff_age;

  string_msg.data += " age: " + stream.str();

  if (diff_age > 10) {
    string_msg.data[0] = '-';
    string_msg.data[1] = 'R';
    string_msg.data[2] = ' ';
  }

  try {
    gpgga_pub_.publish(gpgga_msg);
    bestpos_pub_.publish(bestpos_msg_);
    string_pub_.publish(string_msg);

    msg_counter_gpgga_++;
  }
  catch (...) {
    ROS_ERROR("[Nmea parser]: exception caught during publishing");
  }
}  // namespace nmea_parser

//}

/* processGPGSA() //{ */

void NmeaParser::processGPGSA(std::vector<std::string>& results) {

  mrs_modules_msgs::Gpgsa gpgsa_msg;
  gpgsa_msg.header.stamp = ros::Time::now();

  gpgsa_msg.auto_manual_mode = results[2];
  gpgsa_msg.fix_mode         = stoiSafe(results[3]);
  for (int i = 0; i < 12; i++) {
    if (results[3 + i] == "") {
      gpgsa_msg.prn.push_back(0);
    } else {
      gpgsa_msg.prn.push_back(stoiSafe(results[3 + i]));
    }
  }
  gpgsa_msg.pdop = stodSafe(results[15]);
  gpgsa_msg.hdop = stodSafe(results[16]);

  std::vector<std::string> results_checksum_remove;
  boost::split(results_checksum_remove, results[17], [](char c) { return c == '*'; });
  gpgsa_msg.vdop = stodSafe(results_checksum_remove[0]);

  try {
    gpgsa_pub_.publish(gpgsa_msg);

    msg_counter_gpgsa_++;
  }
  catch (...) {
    ROS_ERROR("[Nmea parser]: exception caught during publishing");
  }
}  // namespace nmea_parser

//}

/* processGPGST() //{ */

void NmeaParser::processGPGST(std::vector<std::string>& results) {

  mrs_modules_msgs::Gpgst gpgst_msg;
  gpgst_msg.header.stamp = ros::Time::now();

  /* for (size_t i = 0; i < results.size(); i++) { */
  /*   ROS_INFO_STREAM("[NmeaParser]:  " << i << "  " << results[i]); */
  /* } */
  gpgst_msg.utc      = stodSafe(results[1]);
  gpgst_msg.rms      = stodSafe(results[2]);
  gpgst_msg.smjr_std = stodSafe(results[3]);
  gpgst_msg.smnr_std = stodSafe(results[4]);
  gpgst_msg.orient   = stodSafe(results[5]);
  gpgst_msg.lat_std  = stodSafe(results[6]);
  gpgst_msg.lon_std  = stodSafe(results[7]);

  std::vector<std::string> results_checksum_remove;
  boost::split(results_checksum_remove, results[8], [](char c) { return c == '*'; });
  gpgst_msg.alt_std = stodSafe(results_checksum_remove[0]);


  try {
    gpgst_pub_.publish(gpgst_msg);

    msg_counter_gpgst_++;
  }
  catch (...) {
    ROS_ERROR("[Nmea parser]: exception caught during publishing");
  }
}

//}

/* processGPVTG() //{ */

void NmeaParser::processGPVTG(std::vector<std::string>& results) {

  mrs_modules_msgs::Gpvtg gpvtg_msg;
  gpvtg_msg.header.stamp = ros::Time::now();

  /* for (size_t i = 0; i < results.size(); i++) { */
  /*   ROS_INFO_STREAM("[NmeaParser]:  " << i << "  " << results[i]); */
  /* } */

  gpvtg_msg.track_true           = stodSafe(results[1]);
  gpvtg_msg.track_true_indicator = results[2];

  gpvtg_msg.track_mag           = stodSafe(results[3]);
  gpvtg_msg.track_mag_indicator = results[4];

  gpvtg_msg.speed_knots           = stodSafe(results[5]);
  gpvtg_msg.speed_knots_indicator = results[6];

  gpvtg_msg.speed_kmh           = stodSafe(results[7]);
  gpvtg_msg.speed_kmh_indicator = results[8];
  gpvtg_msg.mode_indicator = results[9];


  try {
    gpvtg_pub_.publish(gpvtg_msg);

    msg_counter_gpvtg_++;
  }
  catch (...) {
    ROS_ERROR("[Nmea parser]: exception caught during publishing");
  }
}

//}

/* stodSafe() //{ */

double NmeaParser::stodSafe(std::string& string_in) {
  double ret_val = 0.0;
  if (string_in != "") {
    try {
      ret_val = stod(string_in);
    }
    catch (const std::invalid_argument& e) {
      ROS_ERROR("Invalid argument exception in stodSafe");
    }
  }
  return ret_val;
}

//}

/* stoiSafe() //{ */

int NmeaParser::stoiSafe(std::string& string_in) {
  int ret_val = 0;
  if (string_in != "") {
    try {
      ret_val = stoi(string_in);
    }
    catch (const std::invalid_argument& e) {
      ROS_ERROR("Invalid argument exception in stodSafe");
    }
  }
  return ret_val;
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
