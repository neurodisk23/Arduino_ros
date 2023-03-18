In this code, we use the Wire library to communicate with the MPU6500 using the I2C protocol. We first initialize the MPU6500 by setting the power management register to wake it up, and setting the full-scale ranges for the accelerometer and gyroscope.

In the loop function, we read the raw accelerometer and gyroscope data by requesting 6 bytes of data from the MPU6500 starting from the appropriate register address. We then convert the raw data from two's complement format to a signed integer, and print it to the Serial Monitor.

Note that the MPU6500 uses a 3.3V power supply, so you should use a level shifter if you're using a 5V Arduino board. Also, make sure to connect the SDA and SCL pins of the MPU6500 to the corresponding pins on the Arduino (A4 and A5 on most boards).
