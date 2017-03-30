# Balance-Bot

# Balancing a robot on two wheels using gy-80 sensor

# Introduction about the sensor gy-80:
 The gy-80 is a sensor module consisting of four other smaller sensors which are listed as follows:
 
# Name of the Sensor, Sensor Id and I2C Address

 3 Axis Gyro	                         L3G4200D  	(0x69)	

 3 Axis Accelerometer	            ADXL345     (0x53)	
 
 3 Axis Magnetometer           MC5883L	    (0x1E)	
 
 Barometer + Thermometer    BMP085	    (0x77)	
 
 
 The detailed spesifications of the registers present on the chip of each of the four types of sensors are clearly available in their
 respectibe datasheets. For easier reference kindly refer to the links of the datasheets mentioned below:
 
 # Sensor                    and Datasheet Link
 
  3 Axis Gyro	              http://4tronix.co.uk/arduino/specs/L3G4200D.pdf
 
  3 Axis Accelerometer	     http://4tronix.co.uk/arduino/specs/ADXL345.pdf	
  
  3 Axis Magnetometer       http://4tronix.co.uk/arduino/specs/HMC5883L.pdf
  
  Barometer + Thermometer   http://4tronix.co.uk/arduino/specs/BMP085.pdf
  
  For getting hands-on about how to interface the registers of the gy-80 sensor module using Arduino(Interfacing with Arduino is easy),
  the following google drive link. These are sample codes that can be compiled and executed using an Arduino IDE. Please open the files   in a text editor for ease:
  
  https://drive.google.com/drive/folders/0B3LEs3IwtLvyQUQ3RnFhMlgtdjA?usp=sharing
    
# Components used in the construction of the balance bot:

-> Microcontroller-Arduino Mega 2560 and Arduino Uno

-> Motor Drivers(Cytron- 10 A)

-> gy-80

-> Xbee s2c series(2)

-> Xbee Module or adapter(2)

-> A joystick(Analog Stick)

-> Quadrature encoder motors(2)

-> Wheels

-> Fibre plates

-> Plastic and metal studs for joints

-> Metal screws of variable sizes. 
