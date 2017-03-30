
/*
   Functions: < void setup(), void motion() ,void pid() ,void kalman() ,void setup(), void loop()>
   Global Variables: <kalmanX,kalmanY,txf,accel,gyro,accX, accY, accZ,
                      gyroX, gyroY, gyroZ,tempRaw,gyroXangle, gyroYangle,
                      compAngleX, compAngleY,kalAngleX, kalAngleY,
                      timer,t1,t2,t3,pinA,pinB,pinC,pinD,setpoint,
                      kp,ki,kd,max_speed,error,p,i,d,pp
                      x,y,x_set,y_set,flagbuzzer,flag_read,buzzer>

*/

#include <Wire.h>
#include <Kalman.h>
#include <math.h>
//#include <SoftwareSerial.h>
#define RESTRICT_PITCH
#include "I2Cdev.h"
#include "ADXL345.h"
#include <L3G4200D.h>
//#include <SCILAB.h>
//#include <UART.h>
//Beginning of Auto generated function prototypes by Atmel Studio
void kalman();
void pid(double val);
void motion();
//End of Auto generated function prototypes by Atmel Studio



// class default I2C address is 0x53
// specific I2C addresses may be passed as a parameter here
// ALT low = 0x53 (default for SparkFun 6DOF board)
// ALT high = 0x1D

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

//SoftwareSerial xbee_send(10, 11);   // RX, TX
//Receives from the hardware serial, sends to software serial.
//Receives from software serial, sends to hardware serial.

//The circuit:
//* RX is digital pin 10 (connect to TX of other device)
//* TX is digital pin 11 (connect to RX of other device)

//struct txFrame txf;//a structure variable used to send all the data related to square wave generation and tilt angle measurement together in a frame

ADXL345 accel;//an object of ADXL345 class used to initialize the registers of  ADXL345 sensor and also to get the raw values as measured by the sensor

L3G4200D gyro;//an object of L3G4200D class used to initialize the registers of L3G4200D sensor and also to get the raw values as measured by the sensor

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
int16_t ax, ay, az;
int16_t avx, avy, avz;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
//uint32_t t1,t2;

// TODO: Make calibration routine
const int pinA = 4, pinB = 5, pinC = 8, pinD = 9; //motor pins pinA, pinB for right motor , pinC, pinD for left motor
int buzzer_pin = 3;
double setpoint = -6.94;
double kp = 10.00;
double ki = 0.00;
double kd = 0.00;
const int max_speed = 200;
double error = 0.0, p = 0.0, i = 0.0, d = 0.0, pp = 0.0;
unsigned long t1 = 0, t2 = 0, t3 = 0;
int x, y, buzzer;
float x_set, y_set;
bool flag_read = 1, flagbuzzer = 1;



/*
  ▪ * Function Name:<void setup()>
  ▪ * Input: <None>
  ▪ * Output: <None>
  ▪ * Logic: <It initializes the I2C devices and start their communication and also begins the serial communication.
    Also here it is used to initialize the registers of the accelerometer and gyroscope sensors.
    It does the initialization task every time the program is compiled and uploaded into the mmicrocontroller>
  ▪ * Example Call: <called automatically by the operating system>
  ▪ */


void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(115200);//to start the serial communication between arduino and connected devices
  Serial1.begin(57600);
  //Serial.println("     send_char_data    ");

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);
  pinMode(A3, INPUT); pinMode(A7, INPUT);

  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  accel.initialize();//to initialize the registers of the accelerometer sensor
  gyro.initialize();//to initialize the registers of the accelerometer sensor
  delay(1000);
}


/*
  ▪ * Function Name:<void kalman()>
  ▪ * Input: <None>
  ▪ * Output: <None>
  ▪ * Logic: < read the values from acclerometer ADLX435 sensor and gyroscope
             L3G4200D and implement kalman filter algorithm to find the combined tilt angle
             by combining value of both the sensor mentioned >
  ▪ * Example Call: <Called in void loop()>
  ▪ */

void kalman()
{

  accel.getAcceleration(&ax, &ay, &az);//getting the raw values as measured by the accelerometer sensor in ax,ay,az members of the accel object through the library method
  gyro.getAngularVelocity(&avx, &avy, &avz);//getting the raw values as measured by the gyroscope sensor in avx,avy,avz members of the gyro object through the library method

  accX = (double)ax;
  accY = (double)ay;
  accZ = (double)az;

  gyroX = (double)avx;
  gyroY = (double)avy;
  gyroZ = (double)avz;
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // atan2 outputs the value of -π to π (radians)
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  //  Serial.print(roll); Serial.print("\t");
  //  Serial.print(gyroXangle); Serial.print("\t");
  //  Serial.print(compAngleX); Serial.print("\t");
  //Serial.print(kalAngleX); Serial.print("\t   yyyyy    ");
  //
  //  Serial.print("\t");
  //
  //  Serial.print(pitch); Serial.print("\t");
  //  Serial.print(gyroYangle); Serial.print("\t");
  //  Serial.print(compAngleY); Serial.print("\t");
  //Serial.print(kalAngleY); Serial.print("\t");

}

/*
  ▪ * Function Name:<void pid(double)>
  ▪ * Input: <double val>   this is the tilt angle y send from the loop
  ▪ * Output: <None>
  ▪ * Logic: < read values of kp ki kd as  analogread . calculate the error
              in tilt angle and motor direction and speed with the pid algorithm>
  ▪ * Example Call: <Called in void loop() >
  ▪ */



void pid(double val)
{
  kp = (double)analogRead(A3) / 10.0; //read value from potentiometer
  ki = (double)analogRead(A4) / 100.0; //read value from potentiometer
  kd = (double)analogRead(A5) / 2.0; //read value from potentiometer
  setpoint = (double)analogRead(A7) / 100.0; //read value from potentiometer
  setpoint = -setpoint - y_set;                 // negetion is done to get the setpoint in negetive value
  //    Serial.print("setpoint");
  //    Serial.print(setpoint);
  //    Serial.print("\tKp:");
  //    Serial.print(kp);
  //    Serial.print("\tKi:");
  //    Serial.print(ki);
  //    Serial.print("\tKd:");
  //    Serial.println(kd);
  p = val - setpoint;             //proportional = current_value - setpoint
  if (abs(p) > 30) error = 0;
  else
  {
    i = (i + p) > 1500 ? 1500 : (i + p);       //integral = integral + proportional
    i = (i < -1500) ? -1500 : i;
    d = p - pp;                         //differtial = proportional - previous_proportional
    error = kp * p + kd * d + ki * i;
  }
  //      Serial.print("\t");
  //      Serial.print("p:");
  //      Serial.print(p);
  //      Serial.print("\t");
  //      Serial.print("i:");
  //      Serial.print(i);
  //      Serial.print("\t");
  //      Serial.print("d:");
  //      Serial.print(d);
  //      Serial.print("\t err");
  //      Serial.print(error);
  error = (error > 255) ? 255 : error;
  error = (error < -255) ? -255 : error; // limit the error calculated to -255 to 255 because atmega 2560 is a 8 bit controller
  //Serial.println(error);
  pp = p;                           //store the value proportional for next itteration
}

/*
  ▪ * Function Name:<void motion()>
  ▪ * Input: <None>
  ▪ * Output: <None>
  ▪ * Logic: < command the motor according to the error calculated in pid() >
  ▪ * Example Call: <Called in void loop()>
  ▪ */


void motion()
{
  //Serial.print(error);
  double s_right, s_left;                                     //variable for right motor and left motor
  float motion_matrix[2][2] = {{0.5, 0.5}, { -0.5, 0.5}};     //inverse kinematics matrix for calculation for the speed of motor using vectors
  //                        |   0.5   0.5 |
  //                        | - 0.5   0.5 |

  bool dl, dr;                                                //direction variable for motor left and motor right
  //Serial.print("     x_sett   "); Serial.print(x_set);
  //Serial.print("     y_sett   "); Serial.print(y_set);

  s_right = 24 * ( (motion_matrix[0][1] * x_set)) + error;    //calculation for speed of right motor i.e right_speed = yaw + forward ;
  s_left = 24 * ( (motion_matrix[1][1] * x_set)) - error;     //calculation for speed of right motor i.e left_speed = yaw + forward ;

  //Serial.print("     s_left   "); Serial.print(s_left);
  //Serial.print("     s_right   "); Serial.println(s_right);


  s_right = s_right > 254 ? 254 : s_right;                    //limiting the speed values for 0 to 255 for right motor
  s_right = s_right < -254 ? -254 : s_right;                  //limiting the speed values for 0 to -255 for right motor
  s_left = s_left > 254 ? 254 : s_left;                       //limiting the speed values for 0 to 255 for left motor
  s_left = s_left < -254 ? -254 : s_left;                     //limiting the speed values for 0 to -255 for left motor

  analogWrite(pinB, abs(s_right));                            //command the right motor
  analogWrite(pinD, abs(s_left));                             //command the right motor

  dr = s_right > 0 ? 1 : 0;                                   //to set the motor direction 0 for clockwise and 1 for counterclock
  dl = s_left > 0 ? 1 : 0;                                   //to set the motor direction 0 for clockwise and 1 for counterclock

  digitalWrite(pinA, dr), digitalWrite(pinC, dl);             //command the motor driver for perticular direction
}




/*
  ▪ * Function Name:<void loop()>
  ▪ * Input: <None>
  ▪ * Output: <None>
  ▪ * Logic: <All the operational tasks in the program such as those related to interfacing the sensors
    and implementing any logic so as to achieve a task are written her.
    The code written here repeats till the microcontroller is powered on>
  ▪ * Example Call: <Called automatically by the operating system>
  ▪ */


void loop()
{

  String str = "";                                //string  to store the data recieved  from xbee
  int data_flag = 0;

  if ((flag_read == 1) || (millis() - t1 > 35))
  {
    Serial1.write('y');
    flag_read = 0;
    t1 = millis();
    //Serial.print("   requested  ");
  }

  if ( Serial1.available() && (millis() - t1 > 25))
  {

    //Serial.print("   data recieved  ");
    while (Serial1.available())
    {
      char c = Serial1.read(); // read data from serial buffer
      Serial.write(c);          // write data to serial moniter
      if ((c == '_') && (data_flag == 0))
      {
        x = str.toInt();
        data_flag = 1;
        str = "";
      }
      else if ((c == '_') && (data_flag == 1))
      {
        y = str.toInt();
        data_flag = 0;
        str = "";
        c = Serial1.read();
        str += c;
        buzzer = str.toInt();
        str = "";
      }
      else str += c;

      //Serial.println(c);
    }
    flag_read = 1;
    //    Serial.print("   x  "); Serial.print(x);
    //    Serial.print("   y  "); Serial.print(y);
    //    Serial.print("   buzzer  "); Serial.print(buzzer);

  }

  if ((buzzer == 0) && (y == 5) && (flagbuzzer == 1) && (millis() - t3 < 1000))
    digitalWrite(3, 0), flagbuzzer = 0;

  else digitalWrite(3, 1), flagbuzzer = 1, t3 = millis();




  x_set = (x - 5);
  y_set = (y - 5) / 2;
  kalman();
  pid( kalAngleY );
  motion();

}



