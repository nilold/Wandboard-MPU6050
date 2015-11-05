#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <iostream>
#include <stdint.h>
#include <math.h>
#include <chrono>
#include "MPU6050.hpp"

using namespace std;

void MPU::CalculateAngles()
{
	static float deltaT = 0;

	struct{
		float accel_x;
		float accel_y;
	}ang;

	mpu_data ax;
	mpu_data ay;
	mpu_data az;
	mpu_data gx;
	mpu_data gy;
	mpu_data gz;

	float gyro_x_rate = 0;
	float gyro_y_rate = 0;

	MPU::Receive(MPU6050_RA_GYRO_XOUT_H, &gx.high);
	MPU::Receive(MPU6050_RA_GYRO_XOUT_L, &gx.low);
	MPU::Receive(MPU6050_RA_GYRO_YOUT_H, &gy.high);
	MPU::Receive(MPU6050_RA_GYRO_YOUT_L, &gy.low);
	MPU::Receive(MPU6050_RA_ACCEL_XOUT_H, &ax.high);
	MPU::Receive(MPU6050_RA_ACCEL_XOUT_L, &ax.low);
	MPU::Receive(MPU6050_RA_ACCEL_YOUT_H, &ay.high);
	MPU::Receive(MPU6050_RA_ACCEL_YOUT_L, &ay.low);
	MPU::Receive(MPU6050_RA_ACCEL_ZOUT_H, &az.high);
	MPU::Receive(MPU6050_RA_ACCEL_ZOUT_L, &az.low);

	//Divide pela sensibilidade para obter o valor de velocidade angular, em graus/s
	gyro_x_rate = (float)gx.value/gyrosen;
	gyro_y_rate = (float)gy.value/gyrosen;

	ang.accel_x = ACC_M*atan(((float)ay.value) / (sqrt((float)ax.value*(float)ax.value + (float)az.value*(float)az.value)));
	ang.accel_y = -ACC_M*atan(((float)ax.value) / (sqrt((float)ay.value*(float)ay.value + (float)az.value*(float)az.value)));

	deltaT = getTime();

	angles.x = ( ALPHA*(angles.x + gyro_x_rate*deltaT) + (1-ALPHA)*(ang.accel_x));
	angles.y = ( ALPHA*(angles.y + gyro_y_rate*deltaT) + (1-ALPHA)*(ang.accel_y));

	cout<<" time: "<<(float)deltaT;
}

double MPU::getTime(){
	static std::chrono::high_resolution_clock::time_point t2;
	t2 = std::chrono::high_resolution_clock::now();

	static std::chrono::high_resolution_clock::time_point t1 = t2;
	
	static std::chrono::duration<double> dT;

	dT = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
	t1 = std::chrono::high_resolution_clock::now();

	return dT.count();
}

void MPU::Receive(uint8_t registerAddr, uint8_t* data) {
	uint8_t aux;
	if(aux = i2c_smbus_read_byte_data(i2cHandle, registerAddr) != -1)
		*data = aux;
	else
		cout<<"Erro ao ler do registrador 0x "<<hex<<(int)registerAddr<<endl;
}

void MPU::Transmit(int registerAddr, uint8_t data) {
	if(i2c_smbus_write_byte_data(i2cHandle, registerAddr, data) == -1)
		cout<<"Erro ao escrever "<<hex<<(int)data<<" no registrador 0x"<<(int)registerAddr<<endl;
}

void MPU::GetTemperature(float *temperature){

	mpu_data temp;

	MPU::Receive(MPU6050_RA_TEMP_OUT_H, &temp.high);
	MPU::Receive(MPU6050_RA_TEMP_OUT_L, &temp.low);

	*temperature = (float)temp.value/340 + 36.53;

}

float MPU::getAngleX(){
	return angles.x;
}
float MPU::getAngleY(){
	return angles.y;
}

MPU::MPU(const char* device_name){

	angles.x = 0;
	angles.y = 0;

	while(!MPU::connectI2C(device_name));

	MPU::Transmit(MPU6050_RA_SMPLRT_DIV, 0x07);	
	//Disable FSync, 256Hz DLPF
	MPU::Transmit(MPU6050_RA_CONFIG, 0x00);	
	//Disable gyro self tests, scale of 250 degrees/s
	MPU::Transmit(MPU6050_RA_GYRO_CONFIG, GYROSEN);	
	//Disable accel self tests, scale of +-2g, no DHPF
	MPU::Transmit(MPU6050_RA_ACCEL_CONFIG, ACCELSEN);
	//Freefall threshold of |0mg|
	MPU::Transmit(MPU6050_RA_FF_THR, 0x00);
	//Freefall duration limit of 0
	MPU::Transmit(MPU6050_RA_FF_DUR, 0x00);	
	//Motion threshold of 0mg
	MPU::Transmit(MPU6050_RA_MOT_THR, 0x00);	
	//Motion duration of 0s
	MPU::Transmit(MPU6050_RA_MOT_DUR, 0x00);	
	//Zero motion threshold
	MPU::Transmit(MPU6050_RA_ZRMOT_THR, 0x00);	
	//Zero motion duration threshold
	MPU::Transmit(MPU6050_RA_ZRMOT_DUR, 0x00);	
	//Disable sensor output to FIFO buffer
	MPU::Transmit(MPU6050_RA_FIFO_EN, 0x00);
	
	//AUX I2C setup
	//Sets AUX I2C to single master control, plus other config
	MPU::Transmit( MPU6050_RA_I2C_MST_CTRL, 0x00);
	//Setup AUX I2C slaves
	MPU::Transmit(MPU6050_RA_I2C_SLV0_ADDR, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV0_REG, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV0_CTRL, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV1_ADDR, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV1_REG, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV1_CTRL, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV2_ADDR, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV2_REG, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV2_CTRL, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV3_ADDR, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV3_REG, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV3_CTRL, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV4_ADDR, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV4_REG, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV4_DO, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV4_CTRL, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV4_DI, 0x00);

	//MPU6050_RA_I2C_MST_STATUS //Read-only
	//Setup INT pin and AUX I2C pass through
	MPU::Transmit(MPU6050_RA_INT_PIN_CFG, 0x00);
	//Enable data ready interrupt
	MPU::Transmit(MPU6050_RA_INT_ENABLE, 0x00);

	//Slave out, dont care
	MPU::Transmit(MPU6050_RA_I2C_SLV0_DO, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV1_DO, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV2_DO, 0x00);
	MPU::Transmit(MPU6050_RA_I2C_SLV3_DO, 0x00);
	//More slave config
	MPU::Transmit(MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
	//Reset sensor signal paths
	MPU::Transmit(MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
	//Motion detection control
	MPU::Transmit(MPU6050_RA_MOT_DETECT_CTRL, 0x00);
	//Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
	MPU::Transmit(MPU6050_RA_USER_CTRL, 0x00);
	//Sets clock source to gyro reference w/ PLL
	MPU::Transmit(MPU6050_RA_PWR_MGMT_1, 0b00000010);
	//Controls frequency of wakeups in accel low power mode plus the sensor standby modes
	MPU::Transmit(MPU6050_RA_PWR_MGMT_2, 0x00);
	MPU::Transmit(MPU6050_RA_FIFO_R_W, 0x00);
	connectionTest();
}


void MPU::connectionTest(){

	uint8_t whoAmI;
	float temperature;

	MPU::Receive(MPU6050_RA_WHO_AM_I, &whoAmI);
	cout<<"Who Am I : 0x"<<hex<<(int)whoAmI<<endl;
	GetTemperature(&temperature);
	cout<<"Temperatura : "<<temperature<<"ºC"<<endl;

}

bool MPU::connectI2C(const char* device_name){

	i2cHandle = open(device_name, O_RDWR | O_NOCTTY);
	while(i2cHandle == -1){
	usleep(1000);
	cout<<"Não conseguiu conectar com dispositivo: "<<device_name<<endl;
	i2cHandle = open(device_name, O_RDWR | O_NOCTTY);
	};
	// if(ioctl(i2cHandle, I2C_TENBIT, 0) == -1) std::cout<<"erro"<<std::endl;
	ioctl(i2cHandle, I2C_SLAVE, MPU6050_DEFAULT_ADDRESS);

	if(i2c_smbus_read_byte_data(i2cHandle, MPU6050_RA_WHO_AM_I) != MPU6050_DEFAULT_ADDRESS){
		cout<<"\nFalha ao comunicar com o MPU"<<endl;
		return false;
	}
	else{
		cout<<"\nConexão com o MPU bem sucedida com o periférico "<<device_name<<". I2C device: "<<i2cHandle<<endl;
		return true;
	}
}

void MPU::disconnectI2c(const char* device_name){
	if(close(i2cHandle) < 0)
		cout<<"Erro ao desconectar dispositivo do periférico I2C"<<endl;
	else cout<<"Dispositivo desconectado do periférico "<<device_name<<endl<<endl;
}