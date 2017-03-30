/*
   Team Id: <eYRC-BB#3679>
   Author List: <Rahul Prasad, Sk Sajeed Hossain, Sibangi Panda, Amit Kumar>
   Filename: <send_data_joystic_balancebot>
   Theme: <Balance Bot>
   Functions: < void setup(), void loop() >
   Global Variables: <xbee_send,x,y,b,flagbuzzer>

*/
#include <math.h>
#include <SoftwareSerial.h>
//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio


SoftwareSerial xbee_send(10, 11);   // RX, TX   
/*
  ▪ * Function Name:<void setup()>
  ▪ * Input: <None>
  ▪ * Output: <None>
  ▪ * Logic: <It begins the serial communication between Arduino and the system.
  Also it initialises or starts the communicatioon of the xbee to transfer data to the xbee connected on the bot.
  Here three analog pins A3 A4 and A5 are defined as input  used for taking the analog values of the joystick(for x, y direction and joystick click function)
  It does the initialization task every time the program is compiled and uploaded into the mmicrocontroller>
  ▪ * Example Call: <called automatically by the operating system>
  ▪ */

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  xbee_send.begin(57600);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);
  pinMode(A1,INPUT);
  pinMode(3,OUTPUT);

}
int x,y,b;
bool flagbuzzer=1;
/*
  ▪ * Function Name:<void loop()>
  ▪ * Input: <None>
  ▪ * Output: <None>
  ▪ * Logic: <All the operational tasks in the program such as sending 
      the analog data values of the joystick(in x,y diretcion and the joystick click function)
      and implementing any logic so as to achieve a task are written her.
      The code written here repeats till the microcontroller is powered on>
  ▪ * Example Call: <Called automatically by the operating system>
  ▪ */
void loop() 
{
  // put your main code here, to run repeatedly: 
  String send_data="";  //holds  the concatenated data set of the joystick to be sent through xbee
  char char_data[6],send_char_data[60]; // char_data holds each of the three values of the joystick(x, y and press)
  //send_char_data- a character array to hold the concatenated string after conversion into character array
  x=analogRead(A4)/100;
  dtostrf(x, 1, 0, char_data);// built in function to convert a decimal into a string  
  send_data+=char_data;
  //Serial.print(char_data);Serial.print("\t");
  
  send_data+="_";
  
  y=analogRead(A5)/100;
  dtostrf(y, 1, 0, char_data);
  send_data+=char_data;
  //Serial.print(char_data);Serial.print("\t");

  send_data+="_";
  
  b=analogRead(A1)>100?1:0;
  dtostrf(b, 1, 0, char_data);
  send_data+=char_data;
  //Serial.print(char_data);Serial.print("\t");
  Serial.println(send_data);
  send_data.toCharArray(send_char_data, 10); //built in function to convert the concatenated string into a character array
  if(xbee_send.available())//checking for the availability of data on the serial buses (10,11) of arduino to which the xbee is connected
    {
      char c=xbee_send.read();Serial.println(c);
      if((c=='y'))//checking whether the character read from the serial bus is 'y' or not
      {
      xbee_send.write(send_char_data);//Accordingly writing the data to be sent on the serial bus which is received by other xbee on the bot
      Serial.println("    data sent.......    ");
      
      }
    }
  send_data="";
}
