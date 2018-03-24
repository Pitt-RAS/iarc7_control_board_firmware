#include <ros.h>
#include <SoftwareSerial.h>
#include "Bitcraze_PMW3901.h"
//#include <sensor_msgs/Range.h>
#include <iarc7_msgs/Nano.h>
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;
ros::NodeHandle nh;

SoftwareSerial SoftSrial(9, 7); // RX, TX
// Using digital pin 10 for chip select
Bitcraze_PMW3901 flow(10);

iarc7_msgs::Nano sensors_msgs;
ros::Publisher sensors_pub("nano_data", &sensors_msgs);

void setup()
{
    // Don't forget to run roscore!
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(sensors_pub);
    //nh.advertise(rangefinder_pub);
    
    SoftSrial.begin(115200);
    Serial.begin(115200);
    if (!flow.begin()) {
      //Serial.println("Initialization of the flow sensor failed");
      //while(1) { }
     }
    Wire.begin();
    sensor.init();
    sensor.setTimeout(500);
    sensor.startContinuous();

}

int16_t deltaX,deltaY;
int voltagePin = A0;

void loop()
{

    static unsigned long current_time = micros();
    
    if(SoftSrial.available() >= 9)
    { 
          while(0x59 != SoftSrial.read());
          if(0x59 == SoftSrial.read())
          //if((0x59 == SoftSrial.read()) && (0x59 == SoftSrial.read())) //Byte1 & Byte2
          {
              unsigned int t1 = SoftSrial.read(); //Byte3
              unsigned int t2 = SoftSrial.read(); //Byte4
              t2 <<= 8;
              t2 += t1;
              if(t2 < 1000)
              {
//                Serial.print(t2);
//                Serial.print('\t');
//                Serial.print('\n');
                  sensors_msgs.long_range = (float)(t2/100.0);
                  sensors_msgs.msg_received = nh.now();
              }
              t1 = SoftSrial.read(); //Byte5
              t2 = SoftSrial.read(); //Byte6
              t2 <<= 8;
              t2 += t1;

              for(int i=0; i<3; i++)
              {
                  SoftSrial.read(); ////Byte7,8,9
              }
             
              if(sensor.checkReady())
              {
                  sensors_msgs.short_range = sensor.readRangeContinuousMillimeters()/1000.0;
                  sensors_msgs.short_range_offset = micros() - current_time;
                               
              }
              else
              {
                sensors_msgs.short_range_offset = 0;
              }
                
              flow.readMotionCount(&deltaX, &deltaY);
      
              sensors_msgs.deltaX = deltaX;
              sensors_msgs.deltaY = deltaY;
              sensors_msgs.flow_board_offset = micros() - current_time;

              int voltage_sensor_average = 20;
              uint32_t ADCCountsVolts = 0;

              
              for(int i = 0; i < voltage_sensor_average; i++) {
                ADCCountsVolts += analogRead(voltagePin);
                delayMicroseconds(50);
              }
              
              sensors_msgs.battery_offset = micros() - current_time;
              
              ADCCountsVolts = ADCCountsVolts*5*4.9*12.4/voltage_sensor_average;

              sensors_msgs.battery_voltage = (float)ADCCountsVolts/(1024*14.16);
              
              //Serial.print("\n");
              //Serial.print("Two\n");
              //Serial.print(long_offset);
              sensors_pub.publish(&sensors_msgs);
              //Serial.print("X: ");
              //Serial.print(deltaX);
              //Serial.print(", Y: ");
              //Serial.print(deltaY);
              //Serial.print("\n");
              current_time = micros();
//
              //Serial.print(loop_check3);
              //Serial.print("\n");
            
              nh.spinOnce();
          }
       }
    }
    

