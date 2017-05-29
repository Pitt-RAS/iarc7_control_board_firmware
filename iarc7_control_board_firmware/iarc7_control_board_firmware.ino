/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
//#define USE_TEENSY_HW_SERIAL
#include <ros.h>
#include <iarc7_msgs/LandingGearContactsStamped.h>
#include <sensor_msgs/Range.h>
#include <Wire.h>
#include <VL53L0X.h>

//https://github.com/pololu/vl53l0x-arduino for VL53L0X library

VL53L0X sensor;

ros::NodeHandle nh;

iarc7_msgs::LandingGearContactsStamped foot_switches_state;
ros::Publisher foot_switches("landing_gear_contact_switches", &foot_switches_state);

sensor_msgs::Range range_msg;
ros::Publisher rangefinder_pub("sharp_rangefinder", &range_msg);

char frameid[] = "/sharp_rangefinder";

const int LS_LEFT=2;
const int LS_RIGHT=3;
const int LS_FRONT=4;
const int LS_BACK=5;

const int rate_hz = 50;
const int loop_delay = 1000/rate_hz;


void setup()
{
  nh.initNode();
  nh.advertise(foot_switches);
  nh.advertise(rangefinder_pub);

  pinMode(LS_LEFT, INPUT_PULLUP);
  pinMode(LS_RIGHT, INPUT_PULLUP);
  pinMode(LS_FRONT, INPUT_PULLUP);
  pinMode(LS_BACK, INPUT_PULLUP);

  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;

  range_msg.field_of_view = 0.3;
  range_msg.min_range = 0.01;
  range_msg.max_range = 0.800;
}

void loop()
{
  foot_switches_state.header.stamp = nh.now();
  foot_switches_state.left = !(digitalRead(LS_LEFT));
  foot_switches_state.right = !(digitalRead(LS_RIGHT));
  foot_switches_state.front = !(digitalRead(LS_FRONT));
  foot_switches_state.back = !(digitalRead(LS_BACK));
  foot_switches.publish( &foot_switches_state );

  range_msg.header.stamp = nh.now();
  range_msg.range = sensor.readRangeContinuousMillimeters()/1000.0;
  rangefinder_pub.publish(&range_msg);
  
  nh.spinOnce();
  delay(loop_delay);
}
