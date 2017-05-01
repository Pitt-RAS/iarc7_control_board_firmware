/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <iarc7_msgs/LandingGearContactsStamped.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;

iarc7_msgs::LandingGearContactsStamped foot_switches_state;
ros::Publisher foot_switches("landing_gear_contact_switches", &foot_switches_state);

sensor_msgs::Range range_msg;
ros::Publisher rangefinder_pub("sharp_rangefinder", &range_msg);

const int rangefinder_pin=14;

char frameid[] = "/sharp_rangefinder";

const int LS_LEFT=2;
const int LS_RIGHT=3;
const int LS_FRONT=4;
const int LS_BACK=5;

const int rate_hz = 50;
const int loop_delay = 1000/rate_hz;

/*
 * getRange() - samples the analog input from the ranger
 * and converts it into meters.  
 */
float getRange(int pin_num){
    int sample;
    // Get data
    sample = analogRead(pin_num)/4;
    // if the ADC reading is too low, 
    //   then we are really far away from anything
    if(sample < 10)
        return 254;     // max range
    // Magic numbers to get cm
    // Samples will need to be taken to conver this to a curve
    sample= 1309/(sample-3);
    return (sample - 1)/100; //convert to meters
}


void setup()
{
  nh.initNode();
  nh.advertise(foot_switches);

  pinMode(LS_LEFT, INPUT_PULLUP);
  pinMode(LS_RIGHT, INPUT_PULLUP);
  pinMode(LS_FRONT, INPUT_PULLUP);
  pinMode(LS_BACK, INPUT_PULLUP);

  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  // The field of view will need to be measured
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.01;
  range_msg.max_range = 0.15;
}

void loop()
{
  foot_switches_state.header.stamp = nh.now();
  foot_switches_state.left != digitalRead(LS_LEFT);
  foot_switches_state.right != digitalRead(LS_RIGHT);
  foot_switches_state.front != digitalRead(LS_FRONT);
  foot_switches_state.back != digitalRead(LS_BACK);
  foot_switches.publish( &foot_switches_state );

  range_msg.header.stamp = nh.now();
  range_msg.range = getRange(rangefinder_pin);
  rangefinder_pub.publish(&range_msg);
  
  nh.spinOnce();
  delay(loop_delay);
}
