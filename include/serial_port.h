#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <mutex>
//#include <ros/package.h>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>    // Standard input/output definitions
#include <string>   // String function definitions
#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <errno.h>    // Error number definitions
#include <termios.h>  // POSIX terminal control definitions
#include <sys/ioctl.h>
#include <aio.h>

#include <string>

namespace serial_port {

    class SerialPort {
    public:
        SerialPort();

        void set_node(rclcpp::Node::SharedPtr nh);

        virtual ~SerialPort();

        bool connect(const std::string port, int baudrate);

        void disconnect();

        virtual bool sendChar(const char c);

        virtual bool sendCharArray(uint8_t *buffer, int len);

        void setBlocking(int fd, int should_block);

        bool checkConnected();

        virtual bool readChar(uint8_t *c);

        virtual int readSerial(uint8_t *arr, int arr_max_size);

        int serial_port_fd_;

        rclcpp::Node::SharedPtr nh_;
    };

    class SerialPortThreadsafe : public SerialPort {
    public:
        virtual bool sendChar(const char c) override {
            std::scoped_lock lck(mtx_);
            return SerialPort::sendChar(c);
        };

        virtual bool sendCharArray(uint8_t *buffer, int len) override {
            std::scoped_lock lck(mtx_);
            return SerialPort::sendCharArray(buffer, len);
        };

        virtual bool readChar(uint8_t *c) override {
            std::scoped_lock lck(mtx_);
            return SerialPort::readChar(c);
        };

        virtual int readSerial(uint8_t *arr, int arr_max_size) override {
            std::scoped_lock lck(mtx_);
            return SerialPort::readSerial(arr, arr_max_size);
        };

    private:
        std::mutex mtx_;
    };

}  // namespace serial_port

#endif  // SERIAL_PORT_H_
