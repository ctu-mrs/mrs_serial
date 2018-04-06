#ifndef GARMIN_H_
#define GARMIN_H_

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Char.h>
#include <std_srvs/Trigger.h>

#include <string>

#include "garmin/serial_port.h"

#define BUFFER_SIZE 64

  //static const char PRECISE_MODE = 'P';

  class Garmin
  {
    public:
      Garmin();
      virtual ~Garmin();

      uint8_t crc8(uint8_t *p, uint8_t len);
      void serialDataCallback(uint8_t data);

      ros::ServiceServer netgun_arm;
      ros::ServiceServer netgun_safe;
      ros::ServiceServer netgun_fire;

      bool netgun_safeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
      bool netgun_armCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
      bool netgun_fireCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);


      bool loadParameters();
      void setMode(char c);

      uint8_t connectToSensor(void);
      void releaseSerialLine(void);

      ros::Time lastReceived;

      ros::NodeHandle nh_;
      ros::Publisher range_publisher_;

      serial_device::SerialPort * serial_port_;
      boost::function<void(uint8_t)> serial_data_callback_function_;

      std::string portname_;
  };

#endif  // GARMIN_H_
