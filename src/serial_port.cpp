#include <string>

#include "serial_port.h"

namespace serial_port
{

/* SerialPort() //{ */

SerialPort::SerialPort() {
}

//}

/* ~SerialPort() //{ */

SerialPort::~SerialPort() {
  disconnect();
}

//}

/* checkConnected() //{ */

bool SerialPort::checkConnected() {

  struct termios tmp_newtio;
  int            serial_status = tcgetattr(serial_port_fd_, &tmp_newtio);

  if (serial_status == -1) {

    ROS_ERROR("[%s] Serial port disconected!", ros::this_node::getName().c_str());
    close(serial_port_fd_);
    return false;
  }

  return true;
}

//}

/* connect() //{ */

bool SerialPort::connect(const std::string port) {

  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C

  serial_port_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serial_port_fd_ == -1) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: could not open serial port %s", ros::this_node::getName().c_str(), port.c_str());
    return false;

  } else {
    fcntl(serial_port_fd_, F_SETFL, 0);
  }

  struct termios newtio;
  bzero(&newtio, sizeof(newtio));  // clear struct for new port settings

  cfsetispeed(&newtio, B230400);  // Input port speed
  cfsetospeed(&newtio, B230400);  // Output port speed

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

  newtio.c_cc[VTIME] = 0;  // Wait for up to VTIME*0.1s (1 decisecond), returning as soon as any data is received.
  newtio.c_cc[VMIN]  = 0;

  tcflush(serial_port_fd_, TCIFLUSH);
  tcsetattr(serial_port_fd_, TCSANOW, &newtio);

  setBlocking(serial_port_fd_, 1);

  return true;
}

//}

/* setBlocking //{ */

void SerialPort::setBlocking(int fd, int should_block) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    ROS_ERROR("error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;  // 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
    ROS_ERROR("error %d setting term attributes", errno);
}

//}

/* disconnect() //{ */

void SerialPort::disconnect() {

  // TODO(lfr) wait for thread to finish
  try {

    close(serial_port_fd_);
  }
  catch (int e) {

    ROS_WARN_THROTTLE(1.0, "Error while closing the sensor serial line!");
  }
}

//}

/* sendChar() //{ */

bool SerialPort::sendChar(const char c) {
  try {

    return write(serial_port_fd_, (const void*)&c, 1);
  }
  catch (int e) {

    ROS_WARN_THROTTLE(1.0, "Error while writing to serial line!");
    return false;
  }
}

//}

/* sendCharArray() //{ */

bool SerialPort::sendCharArray(uint8_t* buffer, int len) {
  try {

    bool ret_val = write(serial_port_fd_, buffer, len);
    tcflush(serial_port_fd_, TCOFLUSH);
    return ret_val;
  }
  catch (int e) {

    ROS_WARN_THROTTLE(1.0, "Error while writing to serial line!");
    return false;
  }
}

//}

/* read() //{ */
int SerialPort::readSerial(uint8_t* arr, int arr_max_size) {
  return read(serial_port_fd_, arr, arr_max_size);
}

//}

}  // namespace serial_port
