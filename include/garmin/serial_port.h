#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

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

namespace serial_device
{

class SerialPort {
public:
  SerialPort();
  virtual ~SerialPort();

  bool connect(const std::string port);
  void disconnect();

  bool sendChar(const char c);

  void setSerialCallbackFunction(boost::function<void(uint8_t)> *f);
  void serialThread();

  int           serial_port_fd_;
  boost::thread serial_thread_;
  bool          serial_thread_should_exit_;

  boost::function<void(uint8_t)> *serial_callback_function;
};

}  // namespace serial_device

#endif  // SERIAL_PORT_H_
