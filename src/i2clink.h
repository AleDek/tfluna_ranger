#ifndef _I2CLINK_H
#define _I2CLINK_H

#include <cstdlib>
#include <iostream>
#include <cstdint>
#include <stdint.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port

class I2cLink{

private:
	int file_i2c;

public:
	I2cLink();
	~I2cLink();

	
	bool OpenBus(const char* device);
	void Close();

	bool Begin(const uint8_t slave_address);
	bool Read( uint8_t* buffer, const int length );
	bool Write(uint8_t* buffer, const int length );

	
};

#endif