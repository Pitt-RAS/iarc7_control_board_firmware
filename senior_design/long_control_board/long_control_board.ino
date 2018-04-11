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

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

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

Servo esc_front;
Servo esc_back;
Servo esc_left;
Servo esc_right;

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
    

    esc_front.attach(A0, minPulse, maxThrottle);
    esc_back.attach(A1, minPulse, maxThrottle);
    esc_left.attach(A2, minPulse, maxThrottle);
    esc_right.attach(A3, minPulse, maxThrottle);
    esc_front.writeMicroseconds(minPulse);
    esc_back.writeMicroseconds(minPulse);
    esc_left.writeMicroseconds(minPulse);
    esc_right.writeMicroseconds(minPulse);

    // To start up, the ESC's must be sent a min pulse for a short period of time.
    delay(2000);
    
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

    for(int i = 0; i < 255; i++)
    {
      Serial.print(i);
      Serial.println();
      Serial.print(CONVERT_TO_PWM(i));
      Serial.println();
    }
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

              int num_adc_samples = 10;
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

              sensors_msgs.battery_voltage = final_voltage;

              sensors_pub.publish(&sensors_msgs);

              nh.spinOnce();
          }
       }
   
    }

int runSideRotors(iarc7_msgs::ESCCommand esc_commands)
{
    esc_front.writeMicroseconds(CONVERT_TO_PWM(esc_commands.front_motor_PWM));
    esc_back.writeMicroseconds(CONVERT_TO_PWM(esc_commands.back_motor_PWM));
    esc_left.writeMicroseconds(CONVERT_TO_PWM(esc_commands.left_motor_PWM));
    esc_right.writeMicroseconds(CONVERT_TO_PWM(esc_commands.right_motor_PWM));
      
}

int convolute_with_lp(float samples[])
{

    float sum = 0;
    for(int i = 0; i < filter_order + 1; i++) {
      sum+=samples[i]*taps[i];
    }
    return sum;
}

