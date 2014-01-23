/*
 * File: arduino-serial.cpp
 * Author: Benedict R. Gaster
 * Date: 20th January 2014
 * Desc: Simple Interface for reading and writing on Arduino's serial port
 *
 * Copyright 2014 Benedict R. Gaster
 * License: See the file license.
 */

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <error_exception.hpp>

namespace arduino {

class serial
{
private:
    int file_handle_;
    speed_t baudrate_;
    
public:
    serial(const char * serialport, int baud = B9600) :
	file_handle_(open(serialport, O_RDWR | O_NONBLOCK)),
	baudrate_(baud)
    {
	if (file_handle_ == -1) {
	    throw error_exception("serial: failed to open " + std::string(serialport));
	}

	struct termios toptions;
	if (tcgetattr(file_handle_, &toptions) < 0) {
	    throw error_exception("serial: Couldn't get term attributes");
	}

	switch(baud) {
	case 4800:   
	    baudrate_ = B4800;   
	    break;
	case 9600:   
	    baudrate_ = B9600;
	    break;
#ifdef B14400
	case 14400:  
	    baudrate_ = B14400;  
	    break;
#endif
	case 19200:  
	    baudrate_ = B19200;  
	    break;
#ifdef B28800
	case 28800:  
	    baudrate_ = B28800;  
	    break;
#endif
	case 38400:  
	    baudrate_ = B38400;  
	    break;
	case 57600:  
	    baudrate_ = B57600;  
	    break;
	case 115200: 
	    baudrate_ = B115200; 
	    break;
	}
	cfsetispeed(&toptions, baudrate_);
	cfsetospeed(&toptions, baudrate_);

	// 8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	// no flow control
	toptions.c_cflag &= ~CRTSCTS;
	
	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
	
	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	toptions.c_oflag &= ~OPOST; // make raw
	
	toptions.c_cc[VMIN]  = 0;
	toptions.c_cc[VTIME] = 0;
    
	tcsetattr(file_handle_, TCSANOW, &toptions);
	if( tcsetattr(file_handle_, TCSAFLUSH, &toptions) < 0) {
	    error_exception("serial: Couldn't set term attributes");
	}
    }
    
    ~serial()
    {
	close(file_handle_);
    }
    
    void write(std::string str) 
    {
	if (::write(file_handle_, str.c_str(), str.length()) != str.length()) {
	    error_exception("serial::write: couldn't write whole string\n"); 
	}
	
    }

    void write(uint8_t b)
    {
	if (::write(file_handle_, &b, 1) != 1) {
	    error_exception("serial::write: couldn't write byte\n"); 
	}
    }

    void read_until(std::string& str, char until, int timeout = 10000)
    {
	int i = 0;
	for (;;) {
	    char b;
	    int n = read(file_handle_, &b, 1);
	    
	    if (n == -1) {
		return; // failed to read
	    }

	    if (n == 0) {
		usleep( 1 * 1000);
		timeout--;
		continue;
	    }
	    
	    str.push_back(static_cast<char>(b));
	    i++;
	    
	    if (b == until || timeout <= 0) {
		break;	
	    }	    
	}
    }
    
    int flush()
    {
	sleep(2);
	return tcflush(file_handle_, TCIOFLUSH);
    }    
};

} // namespace arduino

