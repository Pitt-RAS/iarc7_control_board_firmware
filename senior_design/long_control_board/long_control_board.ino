#include <Servo.h> 
#include <ros.h>
#include <SoftwareSerial.h>
#include "Bitcraze_PMW3901.h"
#include <iarc7_msgs/Nano.h>
#include <iarc7_msgs/ESCCommand.h>
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;
ros::NodeHandle nh;

#define CONVERT_TO_PWM(x) ((((float)x/255)*(1000))+1000)

int runSideRotors(iarc7_msgs::ESCCommand esc_commands);
int convolute_with_lp(float samples[]);

SoftwareSerial SoftSrial(9, 7); // RX, TX
// Using digital pin 10 for chip select
Bitcraze_PMW3901 flow(10);

iarc7_msgs::Nano sensors_msgs;
ros::Publisher sensors_pub("nano_data", &sensors_msgs);

ros::Subscriber<iarc7_msgs::ESCCommand> sub("esc_commands", &runSideRotors);

int minPulse = 900;
int minThrottle = 1000;
int maxThrottle = 2000;

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



  for(int i = 0; i < 3000; i++) {
    PORTC = PORTC | B11100000;
    delayMicroseconds(120);
    PORTC = PORTC & B00011111;
    delayMicroseconds(1880);
  }

    // To start up, the ESC's must be sent a min pulse for a short period of time.
    delay(5000);
    
}

int16_t deltaX,deltaY;
int batteryPin = A7;
int currentPin = A6;
float voltage_samples[6];
const int filter_order = 5;
float taps[filter_order+1] = {0.0102, 0.1177, 0.3721, 0.3721, 0.1177, 0.0102};

long flow_wait = millis();


void loop()
{

    //esc_front.writeMicroseconds(1500);
    if(SoftSrial.available() >= 9)
    { 
          delayMicroseconds(3);
      
          while(0x59 != SoftSrial.read());
          if(0x59 == SoftSrial.read())
          {
              Serial.println("We have started reading");
              unsigned int t1 = SoftSrial.read(); //Byte3
              unsigned int t2 = SoftSrial.read(); //Byte4
              
              t2 <<= 8;
              t2 += t1;
              
              if(t2 < 200)
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

              if(sensors_msgs.short_range > 300)
              {
                sensors_msgs.short_range = 0;
              }


              if(millis() - flow_wait > 33)
              {
                flow.readMotionCount(&deltaX, &deltaY);
                flow_wait = millis();
                sensors_msgs.deltaX = deltaX;
                sensors_msgs.deltaY = deltaY;
                sensors_msgs.flow_board_offset = micros() - current_time;
              }
              else {
                sensors_msgs.flow_board_offset = 0;
              }

              int num_adc_samples = 2;
              uint32_t raw_adc = 0;
              for(int i = 0 ; i < num_adc_samples ; i++)
              {
                raw_adc += analogRead(batteryPin);
                delayMicroseconds(100);
              }

              float final_raw_adc = (float)raw_adc/(float)num_adc_samples;

              for(int i = 0; i < filter_order; i++){
                  voltage_samples[i+1] = voltage_samples[i];
              }

              voltage_samples[0] = final_raw_adc;
                       
              sensors_msgs.battery_offset = micros() - current_time;

              float final_voltage = (convolute_with_lp(voltage_samples)*0.0749)- 17.4;
              
              if(final_voltage < 0)
              {
                final_voltage = 0;
              }

              sensors_msgs.battery_voltage = final_voltage/3;

              sensors_pub.publish(&sensors_msgs);

              nh.spinOnce();
          }
       }
   
    }

int runSideRotors(iarc7_msgs::ESCCommand esc_commands)
{
    // The first bit corresponds to A0, second to A1, etc.
    PORTC = PORTC | B10000000;
    delayMicroseconds(esc_commands.front_motor_PWM);
    PORTC = PORTC & B01111111;
    
    PORTC = PORTC | B01000000;
    delayMicroseconds(esc_commands.back_motor_PWM);
    PORTC = PORTC & B10111111;
    
    PORTC = PORTC | B00100000;
    delayMicroseconds(esc_commands.back_motor_PWM);
    PORTC = PORTC & B11011111;
    
    PORTC = PORTC | B00010000;
    delayMicroseconds(esc_commands.back_motor_PWM);
    PORTC = PORTC & B11101111;

    // Probably delayed up to at least 400 us at this point, so 
    // only delay 1.4 ms

    delayMicroseconds(1400);

}

int convolute_with_lp(float samples[])
{

    float sum = 0;
    for(int i = 0; i < filter_order + 1; i++) {
      sum+=samples[i]*taps[i];
    }
    return sum;
}

