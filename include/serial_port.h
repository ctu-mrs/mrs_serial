#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <ros/package.h>
#include <ros/ros.h>
#include <stdio.h>    // Standard input/output definitions
#include <string.h>   // String function definitions
#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <errno.h>    // Error number definitions
#include <termios.h>  // POSIX terminal control definitions
#include <sys/ioctl.h>

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <string>

namespace serial_port
{

class SerialPort {
public:
  SerialPort();
  virtual ~SerialPort();

  bool connect(const std::string port, int baudrate);
  void disconnect();

  bool sendChar(const char c);
  bool sendCharArray(uint8_t* buffer, int len);

  void setBlocking(int fd, int should_block);

    bool checkConnected();

    int readSerial(uint8_t * arr, int arr_max_size);

    int      serial_port_fd_;
    uint8_t  input_buffer[1024];
    uint16_t input_it = 0;
  };

}  // namespace serial_port

#endif  // SERIAL_PORT_H_
