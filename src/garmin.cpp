#include <string>

#include "garmin/garmin.h"

#define MAXIMAL_TIME_INTERVAL 1

namespace garmin
{

  Garmin::Garmin()
  {
    // Get paramters
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("portname", portname_, std::string("/dev/ttyUSB0"));

    // Publishers
    range_publisher_ = nh_.advertise<sensor_msgs::Range>("range", 1);

    // Output loaded parameters to console for double checking
    ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
    ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());

    connectToSensor();
  }

  Garmin::~Garmin()
  {
  }

  uint8_t Garmin::connectToSensor(void) {

    // Create serial port
    serial_port_ = new serial_device::SerialPort();

    // Set callback function for the serial ports
    serial_data_callback_function_ = boost::bind(&Garmin::serialDataCallback, this, _1);
    serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);

    // Connect serial port
    if (!serial_port_->connect(portname_))
    {
      ROS_ERROR("Could not connect to sensor.");
      return 0;
    }

    ROS_INFO("Connected to sensor.");	

    lastReceived = ros::Time::now();

    return 1;
  }

  void Garmin::releaseSerialLine(void) {

    delete serial_port_;
  }

  void Garmin::serialDataCallback(uint8_t single_character)
  {
    static uint8_t input_buffer[BUFFER_SIZE];
    static int buffer_ctr = 0;
    static uint8_t crc_in = 0;
    static int payload_size = 0;
    static int receiver_state = 0;
    static int receiving_message = 0;


    if (receiving_message==1) {

      // expecting to receive the payload size
      if (receiver_state == 0) {

        // check the message length
        if (single_character >= 0 && single_character < 64) {

          payload_size = single_character;
          receiver_state = 1;
          crc_in += single_character;

          // the receiving message is over the buffer size
        } else {

          receiving_message = 0;
          receiver_state = 0;
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

          // just int16
          // input_buffer[0] message_id
          int16_t range = input_buffer[1] << 8;
          range |= input_buffer[2];

          if (range < 4000 && range >= 0)
          {

            sensor_msgs::Range range_msg;
            range_msg.field_of_view = 0.0523599; // +-3 degree
            range_msg.max_range = 40.0;
            range_msg.min_range = 0;
            range_msg.radiation_type = sensor_msgs::Range::INFRARED;
            range_msg.header.frame_id = "garmin_frame";
            range_msg.header.stamp = ros::Time::now();
            range_msg.range = range * 0.01; // convert to m
            range_publisher_.publish(range_msg);
            lastReceived = ros::Time::now();

          }
          ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(),range*0.01);

        } else {
          ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
          receiving_message = 0;
          receiver_state = 0;
        }
      }

    } else {
      // this character precedes every message
      if (single_character == 'b') {

        receiving_message = 1;
        receiver_state = 0;
        buffer_ctr = 0;
        crc_in = single_character;
      }
    }
  }

  void Garmin::setMode(char c)
  {
    serial_port_->sendChar(c);
  }

} // namespace garmin

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Garmin");
  garmin::Garmin garmin_sensor;
  ros::Rate loop_rate(1);

  while (ros::ok()) {

    // check whether the teraranger stopped sending data
    ros::Duration interval = ros::Time::now() - garmin_sensor.lastReceived;
    if (interval.toSec() > MAXIMAL_TIME_INTERVAL) {

      garmin_sensor.releaseSerialLine();

      ROS_WARN("Garmin not responding, resetting connection...");

      // if establishing the new connection was successfull	
      if (garmin_sensor.connectToSensor() == 1) {

        ROS_WARN("New connection to Garmin was established.");
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}