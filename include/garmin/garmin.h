#ifndef GARMIN_H_
#define GARMIN_H_

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Char.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

#include <string>

#include "garmin/serial_port.h"

#define BUFFER_SIZE 64

// static const char PRECISE_MODE = 'P';

class Garmin {
public:
  Garmin();
  virtual ~Garmin();

  uint8_t crc8(uint8_t *p, uint8_t len);
  void    serialDataCallback(uint8_t data);

  ros::ServiceServer netgun_arm;
  ros::ServiceServer netgun_safe;
  ros::ServiceServer netgun_fire;
  ros::ServiceServer uvled_start_left;
  ros::ServiceServer uvled_start_right;
  ros::ServiceServer uvled_stop;
  ros::ServiceServer board_switch;
  ros::ServiceServer beacon_on;
  ros::ServiceServer beacon_off;

  bool callbackNetgunSafe(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackNetgunArm(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackNetgunFire(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackUvLedStartLeft(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackUvLedStartRight(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackUvLedStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackBoardSwitch(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackBeaconOn(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackBeaconOff(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void fireTopicCallback(const std_msgs::BoolConstPtr &msg);

  void sendHeartbeat();

  bool loadParameters();
  void setMode(char c);

  uint8_t connectToSensor(void);
  void    releaseSerialLine(void);

  ros::Time lastReceived;

  ros::NodeHandle nh_;
  ros::Publisher  range_publisher_;
  ros::Publisher  range_publisher_up_;
  ros::Subscriber fire_subscriber; 

  serial_device::SerialPort *    serial_port_;
  boost::function<void(uint8_t)> serial_data_callback_function_;

  std::string portname_;

  bool enable_servo_;
  bool enable_uvleds_;
  bool enable_switch_;
  bool enable_beacon_;
};

#endif  // GARMIN_H_
