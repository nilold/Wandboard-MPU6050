#include <iostream>
#include "MPU6050.hpp"

using namespace std;

int main(int argc, char const *argv[])
{
	const char device_name[] = "/dev/i2c-0";

	MPU mpu6050(device_name);

	while(1){
		mpu6050.CalculateAngles();
		cout<<"			X: "<<dec<<mpu6050.getAngleX()<<" Y: "<<dec<<mpu6050.getAngleY()<<endl;
		// usleep(1000);
	}

	mpu6050.disconnectI2c(device_name);
	
	return 0;
}