#include <string>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <std_srvs/Trigger.h>

#include "garmin/garmin.h"

#define MAXIMAL_TIME_INTERVAL 1
#define MAX_RANGE 4000  // cm
#define MIN_RANGE 10    // cm

/* Garmin() //{ */

Garmin::Garmin() {

  // Get paramters
  ros::NodeHandle private_node_handle_("~");

  ros::Time::waitForValid();

  private_node_handle_.param("portname", portname_, std::string("/dev/ttyUSB0"));
  private_node_handle_.param("enable_servo", enable_servo_, false);
  private_node_handle_.param("enable_uvleds", enable_uvleds_, false);
  private_node_handle_.param("enable_switch", enable_switch_, false);
  private_node_handle_.param("enable_beacon", enable_beacon_, false);

  // Publishers
  range_publisher_    = nh_.advertise<sensor_msgs::Range>("range", 1);
  range_publisher_up_ = nh_.advertise<sensor_msgs::Range>("range_up", 1);

  if (enable_servo_) {
    netgun_arm  = nh_.advertiseService("netgun_arm", &Garmin::callbackNetgunArm, this);
    netgun_safe = nh_.advertiseService("netgun_safe", &Garmin::callbackNetgunSafe, this);
    netgun_fire = nh_.advertiseService("netgun_fire", &Garmin::callbackNetgunFire, this);
  }
  if (enable_uvleds_) {
    uvled_start_left  = nh_.advertiseService("uvled_start_left", &Garmin::callbackUvLedStartLeft, this);
    uvled_start_right = nh_.advertiseService("uvled_start_right", &Garmin::callbackUvLedStartRight, this);
    uvled_stop        = nh_.advertiseService("uvled_stop", &Garmin::callbackUvLedStop, this);
  }
  if (enable_beacon_) {
    beacon_on  = nh_.advertiseService("beacon_start", &Garmin::callbackBeaconOn, this);
    beacon_off = nh_.advertiseService("beacon_stop", &Garmin::callbackBeaconOff, this);
  }
  if (enable_switch_) {
    board_switch = nh_.advertiseService("board_switch", &Garmin::callbackBoardSwitch, this);
  }
  // Output loaded parameters to console for double checking
  ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());
  ROS_INFO("[%s] enable_servo: %d", ros::this_node::getName().c_str(), enable_servo_);
  ROS_INFO("[%s] enable_switch: %d", ros::this_node::getName().c_str(), enable_switch_);
  ROS_INFO("[%s] enable_uvleds: %d", ros::this_node::getName().c_str(), enable_uvleds_);
  lastReceived = ros::Time::now();

  connectToSensor();
}

//}

/* ~Garmin() //{ */

Garmin::~Garmin() {
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | ------------------------ services ------------------------ |

/*  callbackNetgunSafe()//{ */

bool Garmin::callbackNetgunSafe([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  char id      = '7';
  char tmpSend = 'a';
  char crc     = tmpSend;

  serial_port_->sendChar(tmpSend);
  tmpSend = 1;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  tmpSend = id;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  serial_port_->sendChar(crc);

  ROS_INFO("Safing net gun");
  res.message = "Safing net gun";
  res.success = true;

  return true;
}

//}

/* callbackNetgunArm() //{ */

bool Garmin::callbackNetgunArm([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  char id      = '8';
  char tmpSend = 'a';
  char crc     = tmpSend;

  serial_port_->sendChar(tmpSend);
  tmpSend = 1;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  tmpSend = id;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  serial_port_->sendChar(crc);

  ROS_INFO("Arming net gun");
  res.message = "Arming net gun";
  res.success = true;

  return true;
}

//}

/* callbackNetgunFire() //{ */

bool Garmin::callbackNetgunFire([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  char id      = '9';
  char tmpSend = 'a';
  char crc     = tmpSend;

  serial_port_->sendChar(tmpSend);
  tmpSend = 1;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  tmpSend = id;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  serial_port_->sendChar(crc);

  ROS_INFO("Firing net gun");
  res.message = "Firing net gun";
  res.success = true;
  return true;
}

//}

/* callbackBeaconOn() //{ */

bool Garmin::callbackBeaconOn([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  char id      = '4';
  char tmpSend = 'a';
  char crc     = tmpSend;

  serial_port_->sendChar(tmpSend);
  tmpSend = 1;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  tmpSend = id;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  serial_port_->sendChar(crc);

  ROS_INFO("Starting sirene and beacon");
  res.message = "Starting sirene and beacon";
  res.success = true;
  return true;
}

//}

/* callbackBeaconOff() //{ */

bool Garmin::callbackBeaconOff([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  char id      = '5';
  char tmpSend = 'a';
  char crc     = tmpSend;

  serial_port_->sendChar(tmpSend);
  tmpSend = 1;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  tmpSend = id;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  serial_port_->sendChar(crc);

  ROS_INFO("Stopping sirene and beacon");
  res.message = "Stopping sirene and beacon";
  res.success = true;
  return true;
}

//}

/* callbackUvLedStartLeft() //{ */

bool Garmin::callbackUvLedStartLeft(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  char id      = '1';
  char tmpSend = 'a';
  char crc     = tmpSend;

  serial_port_->sendChar(tmpSend);

  tmpSend = 2;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);

  tmpSend = id;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);

  tmpSend = (uint8_t)req.data;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);

  serial_port_->sendChar(crc);

  ROS_INFO("Starting left UV leds. f: %d", req.data);
  res.message = "Starting left UV leds";
  res.success = true;
  return true;
}

//}

/* callbackUvLedStartRight() //{ */

bool Garmin::callbackUvLedStartRight(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  char id      = '2';
  char tmpSend = 'a';
  char crc     = tmpSend;

  serial_port_->sendChar(tmpSend);

  tmpSend = 2;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);

  tmpSend = id;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);

  tmpSend = (uint8_t)req.data;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);

  serial_port_->sendChar(crc);

  ROS_INFO("Starting right UV leds. f: %d", req.data);
  res.message = "Starting right UV leds";
  res.success = true;

  return true;
}

//}

/* callbackUvLedStop() //{ */

bool Garmin::callbackUvLedStop([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  char id      = '3';
  char tmpSend = 'a';
  char crc     = tmpSend;

  serial_port_->sendChar(tmpSend);
  tmpSend = 1;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  tmpSend = id;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);
  serial_port_->sendChar(crc);

  ROS_INFO("Stopping UV LEDs");
  res.message = "Stopping UV LEDs";
  res.success = true;

  return true;
}

//}

/* callbackBoardSwitch() //{ */

bool Garmin::callbackBoardSwitch(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  char id      = '4';
  char tmpSend = 'a';
  char crc     = tmpSend;

  serial_port_->sendChar(tmpSend);

  tmpSend = 2;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);

  tmpSend = id;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);

  tmpSend = (uint8_t)req.data;
  crc += tmpSend;
  serial_port_->sendChar(tmpSend);

  serial_port_->sendChar(crc);

  ROS_INFO("Switching output to: %d", req.data);
  res.message = "Output switched";
  res.success = true;

  return true;
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* connectToSensors() //{ */

uint8_t Garmin::connectToSensor(void) {

  // Create serial port
  serial_port_ = new serial_device::SerialPort();

  // Set callback function for the serial ports
  serial_data_callback_function_ = boost::bind(&Garmin::serialDataCallback, this, _1);
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

/* releaseSerialLine() //{ */

void Garmin::releaseSerialLine(void) {

  delete serial_port_;
}

//}

/* serialDataCallback() //{ */

void Garmin::serialDataCallback(uint8_t single_character) {

  static uint8_t input_buffer[BUFFER_SIZE];
  static int     buffer_ctr        = 0;
  static uint8_t crc_in            = 0;
  static int     payload_size      = 0;
  static int     receiver_state    = 0;
  static int     receiving_message = 0;

  if (receiving_message == 1) {

    // expecting to receive the payload size
    if (receiver_state == 0) {

      // check the message length
      if (single_character >= 0 && single_character < 64) {

        payload_size   = single_character;
        receiver_state = 1;
        crc_in += single_character;

        // the receiving message is over the buffer size
      } else {

        receiving_message = 0;
        receiver_state    = 0;
        ROS_WARN("[%s] reveived expected size out of bounds <0,63>, reset buffer without evaluating data", ros::this_node::getName().c_str());
      }

      // expecting to receive the payload
    } else if (receiver_state == 1) {

      // put the char in the buffer
      input_buffer[buffer_ctr++] = single_character;
      // add crc
      crc_in += single_character;

      // if the message should end, change state
      if (buffer_ctr >= payload_size)
        receiver_state = 2;

      // expecting to receive the crc
    } else if (receiver_state == 2) {

      if (crc_in == single_character) {
        receiving_message = 0;
        if (payload_size == 2) {
          uint8_t message_id = input_buffer[0];
          uint8_t msg        = input_buffer[1];
          if (message_id == 0x11 && msg == 0x11) {
            //TODO failsafe
          }
        }
        else if (payload_size == 3) {
          // just int16
          // input_buffer[0] message_id
          uint8_t message_id = input_buffer[0];
          int16_t range      = input_buffer[1] << 8;
          range |= input_buffer[2];

          sensor_msgs::Range range_msg;
          range_msg.field_of_view  = 0.0523599;  // +-3 degree
          range_msg.max_range      = MAX_RANGE * 0.01;
          range_msg.min_range      = MIN_RANGE * 0.01;
          range_msg.radiation_type = sensor_msgs::Range::INFRARED;
          range_msg.header.stamp   = ros::Time::now();

          // if range is valid
          if (range < MAX_RANGE && range >= MIN_RANGE) {
            range_msg.range = range * 0.01;  // convert to m
            // if not
          } else {
            range_msg.range = 0;
          }

          if (message_id == 0x00) {
            range_msg.header.frame_id = "garmin_frame";
            range_publisher_.publish(range_msg);
          } else if (message_id == 0x01) {
            range_msg.header.frame_id = "garmin_frame_up";
            range_publisher_up_.publish(range_msg);
          }
          lastReceived = ros::Time::now();

          ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(), range * 0.01);
        }
      } else {
        ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
        receiving_message = 0;
        receiver_state    = 0;
      }
    }

  } else {
    // this character precedes every message
    if (single_character == 'b') {

      receiving_message = 1;
      receiver_state    = 0;
      buffer_ctr        = 0;
      crc_in            = single_character;
    }
  }
}

//}

/* setMode() //{ */

void Garmin::setMode(char c) {

  serial_port_->sendChar(c);
}

//}

/* main() //{ */

int main(int argc, char **argv) {

  ros::init(argc, argv, "Garmin");

  Garmin garmin_sensor;

  ros::Rate loop_rate(1);

  while (ros::ok()) {

    // check whether the teraranger stopped sending data
    ros::Duration interval = ros::Time::now() - garmin_sensor.lastReceived;
    if (interval.toSec() > MAXIMAL_TIME_INTERVAL) {

      garmin_sensor.releaseSerialLine();

      ROS_WARN("[%s]: Garmin not responding, resetting connection...", ros::this_node::getName().c_str());

      // if establishing the new connection was successfull
      if (garmin_sensor.connectToSensor() == 1) {

        ROS_INFO("[%s]: New connection to Garmin was established.", ros::this_node::getName().c_str());
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

//}
