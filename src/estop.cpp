#include <ros/package.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Char.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Empty.h>
#include <mutex>

#include <string>

#include <mrs_modules_msgs/BacaProtocol.h>
#include <mrs_modules_msgs/SerialRaw.h>

#include <mrs_msgs/SetInt.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/service_client_handler.h>

#include <serial_port.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#define BUFFER_SIZE 256

#define MAXIMAL_TIME_INTERVAL 1

namespace estop
{

/* class Estop //{ */

class Estop : public nodelet::Nodelet {

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

  std::atomic<bool> estop_triggered_          = false;
  std::atomic<bool> was_outside_null_tracker_ = false;
  std::atomic<bool> null_tracker_             = false;

  ros::Timer serial_timer_;
  ros::Timer poll_timer_;
  ros::Timer estop_timer_;
  ros::Timer maintainer_timer_;

  mrs_lib::ServiceClientHandler<std_srvs::Trigger> service_eland_;
  mrs_lib::ServiceClientHandler<std_srvs::SetBool> service_set_leds_;
  mrs_lib::ServiceClientHandler<std_srvs::SetBool> service_set_ouster_;
  mrs_lib::ServiceClientHandler<std_srvs::SetBool> service_set_all_;

  void interpretSerialData(uint8_t data);
  void callbackSerialTimer(const ros::TimerEvent &event);
  void callbackPollTimer(const ros::TimerEvent &event);
  void callbackEstopTimer(const ros::TimerEvent &event);
  void callbackMaintainerTimer(const ros::TimerEvent &event);

  void callbackSendRawMessage(const mrs_modules_msgs::SerialRawConstPtr &msg);
  void controlManagerCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr &msg);

  uint8_t connectToSensor(void);
  void    processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct);

  ros::NodeHandle nh_;

  ros::Publisher status_publisher;

  ros::Subscriber raw_message_subscriber;
  ros::Subscriber control_manager_subscriber_;

  serial_port::SerialPort serial_port_;

  boost::function<void(uint8_t)> serial_data_callback_function_;

  bool publish_bad_checksum;
  bool use_timeout;

  uint16_t received_msg_ok           = 0;
  uint16_t received_msg_ok_garmin    = 0;
  uint16_t received_msg_bad_checksum = 0;

  int serial_rate_        = 5000;
  int serial_buffer_size_ = 1024;

  std::string portname_;
  int         baudrate_;
  std::string uav_name_;

  std::vector<uint8_t> poll_msg_;
  std::vector<uint8_t> normal_response_msg_;
  std::vector<uint8_t> estop_response_msg_;

  std::vector<uint8_t> response_msg_;

  std::mutex mutex_msg;

  ros::Time interval_      = ros::Time::now();
  ros::Time last_received_ = ros::Time::now();

  bool is_connected_   = false;
  bool is_initialized_ = false;
};

//}

/* onInit() //{ */

void Estop::onInit() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "serial_estop");


  param_loader.loadParam("uav_name", uav_name_, std::string("uav"));
  param_loader.loadParam("portname", portname_, std::string("/dev/ttyUSB0"));
  param_loader.loadParam("baudrate", baudrate_, 115200);
  param_loader.loadParam("publish_bad_checksum", publish_bad_checksum, false);
  param_loader.loadParam("use_timeout", use_timeout, true);
  param_loader.loadParam("serial_rate", serial_rate_, 5000);
  param_loader.loadParam("serial_buffer_size", serial_buffer_size_, 1024);

  std::vector<int> poll_msg_load;
  std::vector<int> normal_response_msg_load;
  std::vector<int> estop_response_msg_load;

  param_loader.loadParam("poll_msg", poll_msg_load);
  param_loader.loadParam("normal_response_msg", normal_response_msg_load);
  param_loader.loadParam("estop_response_msg", estop_response_msg_load);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Serial estop]: Could not load all parameters!");
    ros::shutdown();
  } else {
    ROS_INFO("[Serial estop]: All params loaded!");
  }

  for (size_t i = 0; i < poll_msg_load.size(); i++) {
    poll_msg_.push_back(uint8_t(poll_msg_load[i]));
  }

  for (size_t i = 0; i < normal_response_msg_load.size(); i++) {
    normal_response_msg_.push_back(uint8_t(normal_response_msg_load[i]));
  }

  for (size_t i = 0; i < estop_response_msg_load.size(); i++) {
    estop_response_msg_.push_back(uint8_t(estop_response_msg_load[i]));
  }

  /* ROS_INFO_STREAM("[Estop]: loaded poll message: "); */
  /* for (size_t i = 0; i < poll_msg_.size(); i++) { */
  /*   ROS_INFO_STREAM(poll_msg_[i]); */
  /* } */

  /* ROS_INFO_STREAM("[Estop]: loaded resp normal message: "); */
  /* for (size_t i = 0; i < normal_response_msg_.size(); i++) { */
  /*   ROS_INFO_STREAM(normal_response_msg_[i]); */
  /* } */

  /* ROS_INFO_STREAM("[Estop]: loaded resp estop message: "); */
  /* for (size_t i = 0; i < estop_response_msg_.size(); i++) { */
  /*   ROS_INFO_STREAM(estop_response_msg_[i]); */
  /* } */


  raw_message_subscriber      = nh_.subscribe("raw_in", 10, &Estop::callbackSendRawMessage, this, ros::TransportHints().tcpNoDelay());
  control_manager_subscriber_ = nh_.subscribe("control_manager_in", 10, &Estop::controlManagerCallback, this, ros::TransportHints().tcpNoDelay());

  // Output loaded parameters to console for double checking
  ROS_INFO_THROTTLE(1.0, "[%s] test test:", ros::this_node::getName().c_str());
  ROS_INFO_THROTTLE(1.0, "[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO_THROTTLE(1.0, "[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());
  ROS_INFO_THROTTLE(1.0, "[%s] baudrate: %i", ros::this_node::getName().c_str(), baudrate_);
  ROS_INFO_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName().c_str() << "] publishing messages with wrong checksum: " << publish_bad_checksum);

  connectToSensor();

  serial_timer_     = nh_.createTimer(ros::Rate(10), &Estop::callbackSerialTimer, this);
  poll_timer_       = nh_.createTimer(ros::Rate(1), &Estop::callbackPollTimer, this);
  estop_timer_      = nh_.createTimer(ros::Rate(1), &Estop::callbackEstopTimer, this);
  maintainer_timer_ = nh_.createTimer(ros::Rate(1), &Estop::callbackMaintainerTimer, this);

  service_eland_      = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "eland_out");
  service_set_leds_   = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "set_leds_out");
  service_set_ouster_ = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "set_ouster_out");
  service_set_all_    = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "set_all_out");

  is_initialized_ = true;
}
//}

//}

// | ------------------------ callbacks ------------------------ |

/* callbackSerialTimer() //{ */

void Estop::callbackSerialTimer(const ros::TimerEvent &event) {

  uint8_t read_buffer[serial_buffer_size_];
  int     bytes_read;

  bytes_read = serial_port_.readSerial(read_buffer, serial_buffer_size_);

  for (int i = 0; i < bytes_read; i++) {
    interpretSerialData(read_buffer[i]);
  }
  /* processMessage */
}

//}

/* callbackPollTimer() //{ */

void Estop::callbackPollTimer(const ros::TimerEvent &event) {

  response_msg_.clear();

  size_t  payload_size = poll_msg_.size();
  uint8_t msg_out_buffer[payload_size];

  for (int i = 0; i < payload_size; i++) {
    msg_out_buffer[i] = poll_msg_[i];
  }

  serial_port_.sendCharArray(msg_out_buffer, payload_size);
}

//}

/* callbackEstopTimer() //{ */

void Estop::callbackEstopTimer(const ros::TimerEvent &event) {

  if (estop_triggered_) {


    ROS_INFO("[Estop]:GOT ESTOP, calling eland, turning leds off");

    std_srvs::Trigger trig;
    service_eland_.call(trig);

    ROS_INFO("[Estop]: waiting for null tracker");

    if (null_tracker_) {

      ROS_INFO("[Estop]: got null tracker, turning off ouster and lights and terminating");
      std_srvs::SetBool set_bool;
      set_bool.request.data = false;
      service_set_all_.call(set_bool);

      serial_timer_.stop();
      poll_timer_.stop();
      estop_timer_.stop();
    }
  }
}

//}

/* callbackMaintainerTimer() //{ */

void Estop::callbackMaintainerTimer(const ros::TimerEvent &event) {

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

    received_msg_ok_garmin    = 0;
    received_msg_ok           = 0;
    received_msg_bad_checksum = 0;

    interval_ = ros::Time::now();

  } else {

    connectToSensor();
  }
}

//}

/* callbackSendRawMessage() //{ */

void Estop::callbackSendRawMessage(const mrs_modules_msgs::SerialRawConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  uint8_t payload_size = msg->payload.size();
  uint8_t out_buffer[payload_size];

  ROS_INFO_STREAM("SENDING");
  serial_port_.sendCharArray(out_buffer, payload_size);
}

//}

/* controlManagerCallback() //{ */

void Estop::controlManagerCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  if (msg->active_tracker != "NullTracker") {
    was_outside_null_tracker_ = true;
  }

  if (msg->active_tracker == "NullTracker" && was_outside_null_tracker_) {
    null_tracker_ = true;
  }
}

//}

// | ------------------------ routines ------------------------ |

/* interpretSerialData() //{ */

void Estop::interpretSerialData(uint8_t single_character) {

  response_msg_.push_back(single_character);

  if (response_msg_.size() == normal_response_msg_.size()) {

    if (response_msg_ == normal_response_msg_) {
      ROS_INFO("[Estop]: got normal response from xbee");
      response_msg_.clear();
    }

    else if (response_msg_ == estop_response_msg_) {
      ROS_INFO("[Estop]: got ESTOP response from xbee");
      response_msg_.clear();
      estop_triggered_ = true;
    }

    else {
      ROS_INFO("[Estop]: got non-defined response from xbee");
      response_msg_.erase(response_msg_.begin());  // Not nice, but our vector is very small
    }
  }
}

//}

/* processMessage() //{ */

void Estop::processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct) {

  /* if (payload_size == 3 && (input_buffer[0] == 0x00 || input_buffer[0] == 0x01) && checksum_correct) { */
  /*   /1* Special message reserved for garmin rangefinder *1/ */
  /*   received_msg_ok_garmin++; */
  /*   uint8_t message_id = input_buffer[0]; */
  /*   int16_t range      = input_buffer[1] << 8; */
  /*   range |= input_buffer[2]; */

  /*   sensor_msgs::Range range_msg; */
  /*   range_msg.field_of_view  = 0.0523599;  // +-3 degree */
  /*   range_msg.max_range      = MAX_RANGE * 0.01; */
  /*   range_msg.min_range      = MIN_RANGE * 0.01; */
  /*   range_msg.radiation_type = sensor_msgs::Range::INFRARED; */
  /*   range_msg.header.stamp   = ros::Time::now(); */

  /*   range_msg.range = range * 0.01;  // convert to m */

  /*   if (range > MAX_RANGE) { */
  /*     range_msg.range = std::numeric_limits<double>::infinity(); */
  /*   } else if (range < MIN_RANGE) { */
  /*     range_msg.range = -std::numeric_limits<double>::infinity(); */
  /*   } */

  /*   if (message_id == 0x00) { */
  /*     range_msg.header.frame_id = garmin_A_frame_; */

  /*     try { */
  /*       range_publisher_A_.publish(range_msg); */
  /*     } */
  /*     catch (...) { */
  /*       ROS_ERROR("[MrsSerial]: exception caught during publishing topic %s", range_publisher_A_.getTopic().c_str()); */
  /*     } */

  /*   } else if (message_id == 0x01) { */
  /*     range_msg.header.frame_id = garmin_B_frame_; */

  /*     try { */
  /*       range_publisher_B_.publish(range_msg); */
  /*     } */
  /*     catch (...) { */
  /*       ROS_ERROR("[MrsSerial]: exception caught during publishing topic %s", range_publisher_B_.getTopic().c_str()); */
  /*     } */
  /*   } */
  /* } else { */
  /* General serial message */
  if (checksum_correct) {
    received_msg_ok++;
  }
  mrs_modules_msgs::BacaProtocol msg;
  msg.stamp = ros::Time::now();
  for (uint8_t i = 0; i < payload_size; i++) {
    msg.payload.push_back(input_buffer[i]);
  }
  msg.checksum_received   = checksum_rec;
  msg.checksum_calculated = checksum;
  msg.checksum_correct    = checksum_correct;
  try {
    /* baca_protocol_publisher_.publish(msg); */
  }
  catch (...) {
    /* ROS_ERROR("[MrsSerial]: exception caught during publishing topic %s", baca_protocol_publisher_.getTopic().c_str()); */
  }
  /* } */
}

//}

/* connectToSensors() //{ */

uint8_t Estop::connectToSensor(void) {

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

}  // namespace estop

PLUGINLIB_EXPORT_CLASS(estop::Estop, nodelet::Nodelet);

