# Arduino with BNO055
In this code, bno_055.ino , we first include the necessary libraries: Wire for I2C communication, Adafruit_Sensor for accessing the sensor data, Adafruit_BNO055 for the BNO055 sensor, and imumaths for mathematical operations on the sensor data.

In the setup function, we begin the serial communication and check if the BNO055 sensor is detected. We also set the use of the external crystal to improve sensor accuracy.

In the loop function, we first read the orientation vector data using the getEvent() function. We then use the getEvent() function again to read the accelerometer and gyro data separately, passing the Adafruit_BNO055::VECTOR_ACCELEROMETER and Adafruit_BNO055::VECTOR_GYROSCOPE constants respectively.

We then print the x, y, and z components of the accelerometer and gyro data to the serial monitor.

Note that this is just a basic example and you can modify the code to suit your specific needs.

### Pin connection 

 * VCC : 5v
 * GND : Gnd
 * SDA : A4
 * SCL : A5
