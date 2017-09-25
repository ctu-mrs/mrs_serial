#ifndef GARMIN_H_
#define GARMIN_H_

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Char.h>

#include <string>

#include "garmin/serial_port.h"

#define BUFFER_SIZE 64

namespace garmin
{

  //static const char PRECISE_MODE = 'P';

  class Garmin
  {
    public:
      Garmin();
      virtual ~Garmin();

      uint8_t crc8(uint8_t *p, uint8_t len);
      void serialDataCallback(uint8_t data);

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

} // namespace garmin

#endif  // GARMIN_H_
