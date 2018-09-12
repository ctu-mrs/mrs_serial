#include <string>

#include "garmin/serial_port.h"
#include "garmin/garmin.h"

namespace serial_device
{

SerialPort::SerialPort() : serial_callback_function() {
  serial_thread_should_exit_ = false;
  serial_port_fd_            = 0;
}

SerialPort::~SerialPort() {
  disconnect();
}

bool SerialPort::connect(const std::string port) {

  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C

  serial_port_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serial_port_fd_ == -1) {
    ROS_ERROR("[%s] could not open serial port %s", ros::this_node::getName().c_str(), port.c_str());
    return false;

  } else {
    fcntl(serial_port_fd_, F_SETFL, 0);
  }

  struct termios newtio;
  bzero(&newtio, sizeof(newtio));  // clear struct for new port settings

  cfsetispeed(&newtio, B115200);  // Input port speed
  cfsetospeed(&newtio, B115200);  // Output port speed

  newtio.c_cflag &= ~PARENB;  // no parity bit
  newtio.c_cflag &= ~CSTOPB;  // 1 stop bit
  newtio.c_cflag &= ~CSIZE;   // Only one stop bit
  newtio.c_cflag |= CS8;      // 8 bit word

  newtio.c_iflag = 0;  // Raw output since no parity checking is done
  newtio.c_oflag = 0;  // Raw output
  newtio.c_lflag = 0;  // Raw input is unprocessed

  // |  coppied from MAVROS to possibly fix the issue with arduino  |
  newtio.c_iflag &= ~(IXOFF | IXON);
  newtio.c_cflag &= ~CRTSCTS;
  // | ----------------------------  ---------------------------- |

  tcflush(serial_port_fd_, TCIFLUSH);
  tcsetattr(serial_port_fd_, TCSANOW, &newtio);

  serial_thread_ = boost::thread(&SerialPort::serialThread, this);
  return true;
}

void SerialPort::disconnect() {
  serial_thread_should_exit_ = true;

  // TODO(lfr) wait for thread to finish
  try {

    close(serial_port_fd_);
  }
  catch (int e) {

    ROS_WARN("Error while closing the sensor serial line!");
  }
}

bool SerialPort::sendChar(const char c) {
  try {

    return write(serial_port_fd_, (const void*)&c, 1);
  }
  catch (int e) {

    ROS_WARN("Error while writing from sensor serial line!");
    return false;
  }
}

void SerialPort::setSerialCallbackFunction(boost::function<void(uint8_t)>* f) {
  serial_callback_function = f;
}

void SerialPort::serialThread() {
  uint8_t single_character;
  // Non read
  while (!serial_thread_should_exit_ && ros::ok()) {
    try {

      if (read(serial_port_fd_, &single_character, 1)) {

        (*serial_callback_function)(single_character);
      }
    }
    catch (int e) {
      ROS_WARN("Error while reading from sensor serial line!");
    }
    ros::Duration(0.0001).sleep();
  }
  return;
}

}  // namespace serial_device
