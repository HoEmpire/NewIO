/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: zerom, Ryu Woon Jung (Leon) */

#include "dmotion/IO/dynamixel_sdk/port_handler.h"
#include "dmotion/Common/Utility/Utility.h"

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include <sys/select.h>

#include <sys/poll.h>
#include <iostream>

#define LATENCY_TIMER  1  // msec (USB latency timer)
                           // You should adjust the latency timer value. From the version Ubuntu 16.04.2, the default latency timer of the usb serial is '16 msec'.
                           // When you are going to use sync / bulk read, the latency timer should be loosen.
                           // the lower latency timer value, the faster communication speed.

                           // Note:
                           // You can check its value by:
                           // $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           //
                           // If you think that the communication is too slow, type following after plugging the usb in to change the latency timer
                           //
                           // Method 1. Type following (you should do this everytime when the usb once was plugged out or the connection was dropped)
                           // $ echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           // $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           //
                           // Method 2. If you want to set it as be done automatically, and don't want to do above everytime, make rules file in /etc/udev/rules.d/. For example,
                           // $ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"0\" > 99-dynamixelsdk-usb.rules
                           // $ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
                           // $ sudo udevadm control --reload-rules
                           // $ sudo udevadm trigger --action=add
                           // $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
                           //
                           // or if you have another good idea that can be an alternatives,
                           // please give us advice via github issue https://github.com/ROBOTIS-GIT/DynamixelSDK/issues

using namespace dynamixel;

PortHandler *PortHandler::getPortHandler(const char *port_name)
{
  return (PortHandler *)(new PortHandler(port_name));
}

PortHandler::PortHandler(const char *port_name)
  : socket_fd_(-1),
    baudrate_(DEFAULT_BAUDRATE_),
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte(0.0),
    poll_waiting_ticks(5000)
{
  is_using_ = false;
  setPortName(port_name);
}

bool PortHandler::readData1Byte(uint8_t* buffer, int length, double timeout)
{
  int rx_length = 0;

  this->setPacketTimeout(timeout);
  while(!this->isPacketTimeout())
  {
    rx_length += this->readPort(&buffer[rx_length], length-rx_length);
    if (rx_length == length)
      return true;
    //timer::delay_us(10);//TODO remove delay
  }
  // std::cout << "PortHandler::readData1Byte: incomplete packete, actual length " << rx_length << std::endl;
  return false;
}

bool PortHandler::openPort(const bool block)
{
  return setBaudRate(baudrate_, block);
}

void PortHandler::closePort()
{
  if(socket_fd_ != -1)
    close(socket_fd_);
  socket_fd_ = -1;
}

void PortHandler::clearPort()
{
  tcflush(socket_fd_, TCIOFLUSH);
}

void PortHandler::setPortName(const char *port_name)
{
  strcpy(port_name_, port_name);
}

char *PortHandler::getPortName()
{
  return port_name_;
}

// TODO: baud number ??
bool PortHandler::setBaudRate(const int baudrate,  const bool block)
{
  int baud = getCFlagBaud(baudrate);

  closePort();

  if(baud <= 0)   // custom baudrate
  {
    setupPort(B38400, block);
    baudrate_ = baudrate;
    return setCustomBaudrate(baudrate);
  }
  else
  {
    baudrate_ = baudrate;
    return setupPort(baud, block);
  }
}

int PortHandler::getBaudRate()
{
  return baudrate_;
}

int PortHandler::getBytesAvailable()
{
  int bytes_available;
  ioctl(socket_fd_, FIONREAD, &bytes_available);
  return bytes_available;
}

int PortHandler::readPort(uint8_t *packet, int length)
{
  timer::delay_us(10.0);//TODO add delay
  return read(socket_fd_, packet, length);
}

int PortHandler::writePort(uint8_t *packet, int length)
{
  return write(socket_fd_, packet, length);
}

void PortHandler::setPacketTimeout(uint16_t packet_length)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandler::setPacketTimeout(double msec)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = msec;
}

bool PortHandler::isPacketTimeout()
{
  if(getTimeSinceStart() > packet_timeout_)
  {
    packet_timeout_ = 0;
    return true;
  }
  return false;
}

double PortHandler::getCurrentTime()
{
	struct timespec tv;
	clock_gettime(CLOCK_REALTIME, &tv);
	return ((double)tv.tv_sec * 1000.0 + (double)tv.tv_nsec * 0.001 * 0.001);
}

double PortHandler::getTimeSinceStart()
{
  double time;

  time = getCurrentTime() - packet_start_time_;
  if(time < 0.0)
    packet_start_time_ = getCurrentTime();

  return time;
}

timespec
timespec_from_ms (const uint32_t millis)
{
  timespec time;
  time.tv_sec = millis / 1e3;
  time.tv_nsec = (millis - (time.tv_sec * 1e3)) * 1e6;
  return time;
}

bool PortHandler::waitUntilReadable(uint32_t timeout)
{
  // Setup a select call to block for serial data or a timeout
  fd_set readfds;
  FD_ZERO (&readfds);
  FD_SET (socket_fd_, &readfds);
  timespec timeout_ts (timespec_from_ms (timeout));
  int r = pselect (socket_fd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);

  if (r < 0) {
    // Select was interrupted
    if (errno == EINTR) {
      return false;
    }
    // Otherwise there was some error
    throw std::runtime_error("error with wait readable");
  }
  // Timeout occurred
  if (r == 0) {
    return false;
  }
  // This shouldn't happen, if r > 0 our fd has to be in the list!
  if (!FD_ISSET (socket_fd_, &readfds)) {
    throw std::runtime_error("select reports ready to read, but our fd isn't in the list, this shouldn't happen!");
  }
  // Data available to read.
  return true;
}

bool PortHandler::waitUntilReadablePoll()
{
  struct pollfd fds[1];
  fds[0].fd = socket_fd_;
  fds[0].events = POLLIN ;
  int pollrc = poll( fds, 1, poll_waiting_ticks);
  if (pollrc <= 0)
  {
      perror("poll");
      return false;
  }
  else
  {
      if( fds[0].revents & POLLIN )
      {
          std::cout << "reset from pollin" << std::endl;
          return true;
      }
  }

  return false;
}

bool PortHandler::setupPort(int cflag_baud, const bool block)
{
  struct termios newtio;

  if (!block)
  {
    socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY|O_NONBLOCK);
  }
  else
  {
    socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY);
  }

  if(socket_fd_ < 0)
  {
    printf("[PortHandler::SetupPort] Error opening serial port!\n");
    return false;
  }

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  newtio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag      = 0;
  newtio.c_lflag      = 0;
  newtio.c_cc[VTIME]  = 0;
  newtio.c_cc[VMIN]   = 0;

  // clean the buffer and activate the settings for the port
  fcntl(socket_fd_,F_SETFL,FNDELAY);
  tcflush(socket_fd_, TCIFLUSH);
  tcsetattr(socket_fd_, TCSANOW, &newtio);


  tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
  return true;
}

bool PortHandler::setCustomBaudrate(int speed)
{
  // try to set a custom divisor
  struct serial_struct ss;
  if(ioctl(socket_fd_, TIOCGSERIAL, &ss) != 0)
  {
    printf("[PortHandler::SetCustomBaudrate] TIOCGSERIAL failed!\n");
    return false;
  }

  ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
  ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;
  int closest_speed = ss.baud_base / ss.custom_divisor;

  if(closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100)
  {
    printf("[PortHandler::SetCustomBaudrate] Cannot set speed to %d, closest is %d \n", speed, closest_speed);
    return false;
  }

  if(ioctl(socket_fd_, TIOCSSERIAL, &ss) < 0)
  {
    printf("[PortHandler::SetCustomBaudrate] TIOCSSERIAL failed!\n");
    return false;
  }

  tx_time_per_byte = (1000.0 / (double)speed) * 10.0;
  return true;
}

int PortHandler::getCFlagBaud(int baudrate)
{
  switch(baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}
