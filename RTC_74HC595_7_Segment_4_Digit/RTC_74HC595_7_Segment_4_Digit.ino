// Define Connections 7_Segment to 74HC595
// Define Connections Digit to NPNC1815

// IN ROS Arduino program we need to include the ros.h header file
// Header file for any messages the we will be used 
// For this project I use Int16.h and Int16MultiArray.h
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>


/* 
  Showing number 0-9 on a Common Anode 7-segment LED display 
  Displays the numbers 0-9 on the display, with one second in between. 
    A 
   --- 
F |   | B 
  | G | 
   --- 
E |   | C 
  |   | 
   --- 
    D 
  This example code is in the public domain. 
  
  Q1  -  A 
  Q2  -  B 
  Q3  -  C 
  Q4  -  C 
  Q5  -  D 
  Q6  -  E 
  Q7  -  G 
  05 -  D1 
  06 -  D2 
  07 -  D3 
  08 -  D4 
 */  
 
// Patterns for characters 0,1,2,3,4,5,6,7,8,9
int Segment[10] = { B01111110,  //0 
                    B00110000,  //1
                    B01101101,  //2
                    B01111001,  //3
                    B00110011,  //4
                    B01011011,  //5
                    B01011111,  //6
                    B01110000,  //7
                    B01111111,  //8
                    B01111011,  //9
                    };
// Using Real_Time_Clock DS1302
#include <virtuabotixRTC.h>

// We need to instantiate the node handle which allows our programs to create publishers and subscribers
// The node handle also takes care of serial port communications
ros::NodeHandle nh;
// Creation of the Real Time Clock Object

// Now we instantiate a Publisher with a topic name chatter 
std_msgs:: Int32MultiArray minu;
ros:: Publisher chatter("chatter", &minu);


virtuabotixRTC myRTC(6, 7, 8);

// Setup Digit of 7_Segment
#define D1 2
#define D2 3
#define D3 4
#define D4 5
// ST_CP pin 12
const int latchPin = 10;
// SH_CP pin 11
const int clockPin = 11;
// DS pin 14
const int dataPin = 12;                
int digit1, digit2, digit3, digit4;

/*
 * For this project we want to change the change the parameter of second in the teminal
 *  In order to do it, we create a function which can be subscribe the data from the topic
 *  known as "RTC_74HC595_7_Segment_4_Digit"
 *  
 *  To change, in teminal we just
 *  rostopic pub second std_msgs/Int32 input_number (to change the second)
 *  rostopic pub minute std_msgs/Int32 input_number (to change the minute)
 *  rostopic pub hour std_msgs/Int32 input_number (to change the hour)
 *  
 *  
 *  The function that we need to create names input, it can subsribe the data in Int32 in seconds
 *  and it will convert it to minute and hours
 *  
 */


void input(const std_msgs::Int32&  msg){
  
  int data = msg.data;
  //myRTC.setDS1302Time(data,0,0,7,19,3,2022);
  if(data != myRTC.seconds){
    myRTC.setDS1302Time(data,myRTC.minutes,myRTC.hours,7,19,3,2022);
  }
  else{
    myRTC.setDS1302Time(myRTC.seconds,myRTC.minutes,myRTC.hours,7,19,3,2022);
  }
    
}

void input_minu(const std_msgs::Int32& msg){
  int data = msg.data;
  if(data != myRTC.minutes){
    myRTC.setDS1302Time(myRTC.seconds,data,myRTC.hours,7,19,3,2022);
  }
  else{
    myRTC.setDS1302Time(myRTC.seconds,myRTC.minutes,myRTC.hours,7,19,3,2022);
  }
}

void input_hr(const std_msgs::Int32& msg){
  int data = msg.data;
  if(data != myRTC.hours){
    myRTC.setDS1302Time(myRTC.seconds,myRTC.minutes,data,7,19,3,2022);
  }
  else{
    myRTC.setDS1302Time(myRTC.seconds,myRTC.minutes,myRTC.hours,7,19,3,2022);
  }
}


ros::Subscriber<std_msgs::Int32> sub("second",input);
ros::Subscriber<std_msgs::Int32> sub1("minute",input_minu);
ros::Subscriber<std_msgs::Int32> sub2("hour",input_hr);
void setup ()
{ 
  // In void setup function we need to initaislize our ros node handle 
  nh.initNode();
  // We advertise any topics beging published "chatter"
  nh.advertise(chatter);
  // We subscribe any topics that we wish to listen to 
  nh.subscribe(sub);
  nh.subscribe(sub1);
  nh.subscribe(sub2);
 
  // Set the current date, and time in the following format:
  // seconds, minutes, hours, day of the week, day of the month, month, year
  //                 (ss, mn, hh, 1week, DD, MM, YY);
  // myRTC.setDS1302Time(00, 02, 14, 7, 19, 3, 2022);
  //myRTC.setDS1302Time(data,,hh);
  
  // Setup pins as Outputs
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  // Setup pins Digit of 7_Segment as Outputs
  for(int i=2; i<6; i++){
    pinMode(i,OUTPUT);
  }
  // Open Serial for using data
  Serial.begin(57600);
}
void loop()
{
  // This allows for the update of variables for time or accessing the individual elements.
  myRTC.updateTime();
  Set_hour(myRTC.hours);
  Set_minute(myRTC.minutes);
  // Publish data in array of 3 elements
  long value[3] = {myRTC.hours,myRTC.minutes,myRTC.seconds};
  minu.data= value;
  minu.data_length = 3;
  // In the topic "chatter" we publish minu in array of 3 elements
  chatter.publish(&minu);
  // In this function "arduino loop function" we calls ros::spinOnce()
  // where all of the ros communication callbacks are handled.
  nh.spinOnce();

}
int Set_hour(int h){
      digit2 = h / 10;  
      digit1 = h % 10; 
      Set_num(digit2);
      digitalWrite(D1, HIGH);  
      digitalWrite(D2, LOW);
      digitalWrite(D3, LOW);  
      digitalWrite(D4, LOW); 
      delay(1);
      Set_num(digit1);
      digitalWrite(D1, LOW);  
      digitalWrite(D2, HIGH);
      digitalWrite(D3, LOW);  
      digitalWrite(D4, LOW);  
      delay(1);
      return Set_num(digit2), Set_num(digit1);
}
int Set_minute(int m){
      digit4 = m / 10;  
      digit3 = m % 10;
      Set_num(digit4);  
      digitalWrite(D1, LOW);  
      digitalWrite(D2, LOW);
      digitalWrite(D3, HIGH);  
      digitalWrite(D4, LOW);
      delay(1); 
      Set_num(digit3);    
      digitalWrite(D1, LOW);  
      digitalWrite(D2, LOW);
      digitalWrite(D3, LOW);  
      digitalWrite(D4, HIGH);
      delay(1);  

      return Set_num(digit4), Set_num(digit3) ;
}

int  Set_num(int num){
  
      // ST_CP LOW to keep LEDs from changing while reading serial data
      digitalWrite(latchPin, LOW);
      
      shiftOut(dataPin, clockPin, LSBFIRST, Segment[num]); 
      
      // ST_CP HIGH change LEDs
      digitalWrite(latchPin, HIGH);
}

int clock_read()
{
  int hr=0;
  int minu=0;
  int clock_data[2] = {0, 0};
  myRTC.updateTime();
  hr = Set_hour(myRTC.hours);
  minu = Set_minute(myRTC.minutes);
  return hr, minu;
}
