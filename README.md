# Wandboard-MPU6050
C++ code to use the MPU6050 with the Wandboard

I will use Jeff Rowberg's i2c devices library (https://github.com/jrowberg/i2cdevlib) and adapt it to use with the Wandboard (or any other embedded linux system with the native linux-i2c lib).

The actual code is very simple and can read raw accel and gyro values and calculate the MPU angle with the complementary filter. This code will be avaiable in the "Raw-Values" directory.

The goal, however, is to use the DMP to obatain a better result and that's why I will be using Jeff Rowberg`s library.
