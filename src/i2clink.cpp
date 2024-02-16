#include "i2clink.h"

I2cLink::I2cLink(){
	this->file_i2c = 0;
}

I2cLink::~I2cLink(){
	this->Close();
};

bool I2cLink::OpenBus(const char* device){
	//----- OPEN THE I2C BUS -----
	
	if ((this->file_i2c = open(device, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		std::cout<<"Failed to open the i2c bus\n";
		return false;
	}
	return true;
}

bool I2cLink::Begin(const uint8_t slave_address){
	
	if (ioctl(this->file_i2c, I2C_SLAVE, slave_address) < 0)      //<<<<<The I2C address of the slave
	{
		//ERROR HANDLING; you can check errno to see what went wrong
		std::cout<<"Failed to acquire bus access and/or talk to slave.\n";
		return false;
	}
	return true;
}

void I2cLink::Close(){
	//----- CLOSE THE I2C BUS -----
	close(this->file_i2c);
}

bool I2cLink::Read( uint8_t* buffer, const int length ){
	//----- READ BYTES -----
	if (read(this->file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		//ERROR HANDLING: i2c transaction failed
		std::cout<<"Failed to read from the i2c bus.\n";
		return false;
	}
	//std::cout<<"Data read:"<< *buffer<<"\n";
	return true;

}

bool I2cLink::Write(uint8_t* buffer, const int length ){
	//----- WRITE BYTES -----
	if (write(this->file_i2c, buffer, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		/* ERROR HANDLING: i2c transaction failed */
		std::cout<<"Failed to write on the i2c bus.\n";
		return false;
	}
	//std::cout<<"send data succesfully\n";

	return true;
}
