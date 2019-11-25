#include <ros.h>
#include <std_msgs/Int64.h>
#include <stdio.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>


// OLED pins
#define OLED_CLK    12
#define OLED_MOSI   11
#define OLED_RESET  10
#define OLED_DC     9
#define OLED_CS     8

Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// motor pins: enable 1 2 3 4, dir 1 2 3 4
int pin_numbers[] = {35,33,31,29,45,43,41,39};
char pin_data[] = {0,0,0,0,0,0,0,0};

// time vars
long long last_callback; // timestamp for last callback
int timeout = 100; // millis to timeout (->idle state)
int loop_rate = 100;

// display wrapper
void updateDisplay()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);

  // make title
  display.setCursor(8,4);
  display.print("MOTOR CTRL by Misza");
  display.setCursor(45,12);
  display.println("v0.5.0");

  // table titles
  display.setCursor(13,39);
  display.print("en");
  display.setCursor(7,51);
  display.print("dir");

  // print table with enable and dir for each motor
  for(int i=0; i<4; i++)
  {
    display.setTextColor(WHITE);
    display.setCursor(36+i*23,27);
    display.print("M");
    display.print(i+1);
    display.setCursor(41+i*23,39);
    display.print(pin_data[i]+0);
    display.setCursor(41+i*23,51);
    display.print(pin_data[i+4]+0);
  }
  // frame
  display.drawLine(1,1,126,1,2);
  display.drawLine(126,1,126,62,2);
  display.drawLine(126,62,1,62,2);
  display.drawLine(1,62,1,1,2);
  display.display();
}

// simple macro to update motor pins
void setMotors()
{
  for(int i=0; i<8; i++)
  {
    digitalWrite(pin_numbers[i],pin_data[i]);
  }
}

// ROS subscriber callback
void motor_callback( const std_msgs::Int64& action_msg )
{
  // just unpacks the msg and saves new timestamp
  int in_data = action_msg.data;
  for(int i=0; i<8; i++)
  {
    pin_data[7-i] = in_data&1;
    in_data = in_data>>1;
  }
  last_callback = millis();

  // special case: if 512-bit is on, effectively increase the timeout 10x
  if(in_data>0){ last_callback += 9*timeout; }
}


ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int64> pin_sub("pins", motor_callback);


void setup()
{
  // ROS init
  nh.initNode();
  nh.subscribe(pin_sub);

  // display init
  pinMode(13,OUTPUT);
  digitalWrite(13,1); // terrible hack feeding display VCC from pin 13
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.display();

  // motor pin init macro
  for(int i=0; i<8; i++)
  {
    pinMode(pin_numbers[i],OUTPUT);
  }
  last_callback = millis();
}

void loop()
{
  // failsafe to reset data if too long time has passed since last cb
  if(millis()-last_callback > timeout)
  {
    for(int i=0; i<8; i++)
    {
      pin_data[i] = 0;
    }
  }

  // update ROS, update motors, update display, sleep
  nh.spinOnce();
  setMotors();
  updateDisplay();
  delay(loop_rate);
}
