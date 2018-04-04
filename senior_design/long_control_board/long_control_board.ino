#include <Servo.h> 
#include <ros.h>
#include <SoftwareSerial.h>
#include "Bitcraze_PMW3901.h"
//#include <sensor_msgs/Range.h>
#include <iarc7_msgs/Nano.h>
#include <iarc7_msgs/FlightControllerStatus.h>
#include <iarc7_msgs/Float64ArrayStamped.h>
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;
ros::NodeHandle nh;

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

void getFcStatus(iarc7_msgs::FlightControllerStatus fc_status);
void turnSideRotors(iarc7_msgs::Float64ArrayStamped esc_commands);
int convolute_with_lp(float samples[]);

SoftwareSerial SoftSrial(9, 7); // RX, TX
// Using digital pin 10 for chip select
Bitcraze_PMW3901 flow(10);

iarc7_msgs::Nano sensors_msgs;
ros::Publisher sensors_pub("nano_data", &sensors_msgs);

int minPulse = 900;
int minThrottle = 1000;
int maxThrottle = 2000;


Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

void setup()
{
    // Don't forget to run roscore!
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(sensors_pub);
    
    
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
    

    esc1.attach(A0, minPulse, maxThrottle);
    esc2.attach(A1, minPulse, maxThrottle);
    esc3.attach(A2, minPulse, maxThrottle);
    esc4.attach(A3, minPulse, maxThrottle);
    esc1.writeMicroseconds(minPulse);
    

    delay(2000);
    
}

int16_t deltaX,deltaY;
int batteryPin = A7;
int currentPin = A6;
uint16_t volt_samples;
float voltage_samples[5];

void loop()
{
    if(SoftSrial.available() >= 9)
    { 

          while(0x59 != SoftSrial.read());
          if(0x59 == SoftSrial.read())
          {
              
              unsigned int t1 = SoftSrial.read(); //Byte3
              unsigned int t2 = SoftSrial.read(); //Byte4
              
              t2 <<= 8;
              t2 += t1;
              
              if(t2 < 1000)
              {
                  sensors_msgs.long_range = (float)(t2/100.0);
              }
              
              sensors_msgs.msg_received = nh.now();
              unsigned long current_time = micros();
              
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
              
              int voltage_sample_number = 5;

              float ADCCountsVolts = 0;

//              for(int i = voltage_sample_number-2; i > -1 ; i = i-1) {
//                  voltage_samples[i+1] = voltage_samples[i];
//              }

              for(int i = 0; i < voltage_sample_number - 1; i++){
                  voltage_samples[i+1] = voltage_samples[i];
              }

              for(int i = 0 ; i < 10 ; i++)
              {
                volt_samples += analogRead(batteryPin);
                delayMicroseconds(100);
              }
              
              voltage_samples[0] = volt_samples / 10;
              volt_samples = 0;              

              //Serial.print(ADCCountsVolts/voltage_sensor_average);
              //Serial.print(", ");
                       
              sensors_msgs.battery_offset = micros() - current_time;

              ADCCountsVolts = convolute_with_lp(voltage_samples);

              ADCCountsVolts = (ADCCountsVolts*0.07984)- 20.58;
              
              if(ADCCountsVolts < 0)
              {
                ADCCountsVolts = 0;
              }

              sensors_msgs.battery_voltage = ADCCountsVolts;
              //Serial.print(sensors_msgs.battery_voltage);
              //Serial.println();

              sensors_pub.publish(&sensors_msgs);

              nh.spinOnce();
          }
       }
   
    }

void getFcStatus(iarc7_msgs::FlightControllerStatus fc_status)
{
    
}


void turnSideRotors(iarc7_msgs::Float64ArrayStamped esc_commands)
{

    
}


/*
 * ADCVolts = ADCVal * 5/1024
 * ADCVolts = (ADCVolts - 1.23)*1.23
 * 
 */
 
int convolute_with_lp(float samples[])
{

    return (0.0102 * samples[0] + 0.1177 * samples[1] + 0.3721 * samples[2] 
    + 0.3721 * samples[3] + 0.1177 * samples[4] + 0.0102*samples[5]);
  
}

