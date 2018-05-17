#include <Servo.h> 
#include <ros.h>
#include <SoftwareSerial.h>
#include <iarc7_msgs/Nano.h>
#include <iarc7_msgs/ESCCommand.h>
#include <Wire.h>

// https://github.com/bitcraze/Bitcraze_PMW3901
#include "Bitcraze_PMW3901.h"

// https://github.com/pololu/vl53l1x-st-api-arduino
#include <vl53l1_api.h>

VL53L1_Dev_t vl53l1x;

ros::NodeHandle nh;

int runSideRotors(const iarc7_msgs::ESCCommand& esc_commands);
int convolute_with_lp(float samples[]);

SoftwareSerial SoftSrial(9, 7); // RX, TX
// Using digital pin 10 for chip select
Bitcraze_PMW3901 flow(10);

iarc7_msgs::Nano sensors_msgs;
ros::Publisher sensors_pub("nano_data", &sensors_msgs);

ros::Subscriber<iarc7_msgs::ESCCommand> sub("esc_commands", &runSideRotors);

unsigned long last_esc_update = 0;
unsigned long last_run_side_rotors = 0;

void setup()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(sensors_pub);
    nh.subscribe(sub);

    // Set up the pwm output pins
    DDRC = DDRC | B00001111;

    analogReference(EXTERNAL);

    SoftSrial.begin(115200);
    Serial.begin(115200);

    flow.begin();

    // Initialize VL53L1X
    Wire.begin();
    Wire.setClock(400000);
    vl53l1x.I2cDevAddr = 0x52;

    VL53L1_software_reset(&vl53l1x);
    VL53L1_WaitDeviceBooted(&vl53l1x);
    VL53L1_DataInit(&vl53l1x);
    VL53L1_StaticInit(&vl53l1x);
    VL53L1_SetDistanceMode(&vl53l1x, VL53L1_DISTANCEMODE_LONG);
    VL53L1_SetMeasurementTimingBudgetMicroSeconds(&vl53l1x, 50000);
    VL53L1_SetInterMeasurementPeriodMilliSeconds(&vl53l1x, 50);
    VL53L1_StartMeasurement(&vl53l1x);

    // To start up, the ESC's must be sent a min pulse for a short period of time.
    for(int i = 0; i < 1000; i++) {
        PORTC = PORTC | B00001111;

        // The intended delay is 120 us. The -4 is to make sure that the pulse width is 
        // no longer than 120 us.
        delayMicroseconds(120-4);
        PORTC = PORTC & B11110000;
        delayMicroseconds(1500);
    }

}

int16_t deltaX,deltaY;
int batteryPin = A7;
int currentPin = A6;
float voltage_samples[6];
const int filter_order = 5;

// Kaiser window filter (with linear phase)
float taps[filter_order+1] = {0.0102, 0.1177, 0.3721, 0.3721, 0.1177, 0.0102};

unsigned long flow_wait = micros();

unsigned long raw_adc = 0;
int num_adc_samples = 0;

// A pulse width of 125 us corresponds to 0 throttle.
uint8_t front_PWM = 125;
uint8_t back_PWM = 125;
uint8_t right_PWM = 125;
uint8_t left_PWM = 125;

unsigned long long_range_reading_time = 0;
unsigned long short_range_reading_time = 0;
unsigned long flow_board_reading_time = 0;

unsigned long last_long_range_reading_time = 0;

void loop()
{
    // We avoid checking the software serial buffer at a  
    // rate higher than the update rate of the long range lidar
    if(micros() - last_long_range_reading_time > 5000) {
      while(SoftSrial.available() < 9){};
      last_long_range_reading_time = micros();

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
              long_range_reading_time = last_long_range_reading_time;
          }

          for(int i=0; i<5; i++)
          {
              SoftSrial.read(); ////Byte5-9
          }
        }
      }
    }

    uint8_t vl53l1x_ready;
    int vl53l1x_status = VL53L1_GetMeasurementDataReady(&vl53l1x,
                                                        &vl53l1x_ready);
    if (vl53l1x_status == VL53L1_ERROR_NONE && vl53l1x_ready == 1) {
        VL53L1_RangingMeasurementData_t ranging_data;
        vl53l1x_status = VL53L1_GetRangingMeasurementData(&vl53l1x,
                                                          &ranging_data);
        if (vl53l1x_status == VL53L1_ERROR_NONE) {
            sensors_msgs.short_range = ranging_data.RangeMilliMeter / 1000.0;
            short_range_reading_time = micros();
            VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x);
        }
    }

    // Purposefully check the flow sensor at a rate lower than the 
    // max update rate to allow it to accumulate longer
    if(micros() - flow_wait > 20000)
    {
      flow_wait = micros();
      flow.readMotionCount(&deltaX, &deltaY);
      sensors_msgs.deltaX = deltaX;
      sensors_msgs.deltaY = deltaY;
      flow_board_reading_time = micros();
    }

    if(micros() - last_run_side_rotors > 300000) {
      front_PWM = 125;
      back_PWM = 125;
      left_PWM = 125;
      right_PWM = 125;
    }
    
    if(micros() - last_esc_update > 20000) {
      last_esc_update = micros();
      updateESC();
    }

    static unsigned long last_serial_update = 0;
    if(micros() - last_serial_update > 10000) {
      last_serial_update = micros();

      sensors_msgs.msg_received = nh.now();

      unsigned long current_time = micros();

      if(long_range_reading_time > 0) {
        sensors_msgs.long_range_offset = (current_time - long_range_reading_time)/10;
        long_range_reading_time = 0;
      }
      else {
        sensors_msgs.long_range_offset = 0;
      }

      if(short_range_reading_time > 0) {
        sensors_msgs.short_range_offset = (current_time - short_range_reading_time)/10;
        short_range_reading_time = 0;
      }
      else {
        sensors_msgs.short_range_offset = 0;
      }

      if(flow_board_reading_time > 0) {
        sensors_msgs.flow_board_offset = (current_time - flow_board_reading_time)/10;
        flow_board_reading_time = 0;
      }
      else {
        sensors_msgs.flow_board_offset = 0;
      }

      float final_raw_adc = (float)raw_adc/(float)num_adc_samples;
      raw_adc = 0;
      num_adc_samples = 0;

      for(int i = filter_order; i > 0; i--){
          voltage_samples[i] = voltage_samples[i-1];
      }

      voltage_samples[0] = final_raw_adc;
               
      float final_voltage = final_raw_adc * 0.0343 - 15.7;
      if(final_voltage < 0)
      {
        final_voltage = 0;
      }

      sensors_msgs.battery_voltage = final_voltage;

      sensors_pub.publish(&sensors_msgs);
      nh.spinOnce();
    }

    raw_adc += analogRead(batteryPin);
    num_adc_samples++;
    delayMicroseconds(100);
    nh.spinOnce();
}

void updateESC() {
    noInterrupts();
    // The first bit corresponds to A0, second to A1, etc.
    PORTC = PORTC | B00000001;
    precise_delay(front_PWM);
    PORTC = PORTC & B11111110;
    interrupts();
    
    const int inter_pulse_delay = 100;
    delayMicroseconds(inter_pulse_delay);
    
    noInterrupts();
    PORTC = PORTC | B00000010;
    precise_delay(back_PWM);
    PORTC = PORTC & B11111101;
    interrupts();
    
    delayMicroseconds(inter_pulse_delay);
    
    noInterrupts();
    PORTC = PORTC | B00000100;
    precise_delay(right_PWM);
    PORTC = PORTC & B11111011;
    interrupts();
    
    delayMicroseconds(inter_pulse_delay);
    
    noInterrupts();
    PORTC = PORTC | B00001000;
    precise_delay(left_PWM);
    PORTC = PORTC & B11110111;
    interrupts();
}

// The ESC protocol requires that we send out a pulse every 20 ms and not every time
// we get an update from the main computer.
int runSideRotors(const iarc7_msgs::ESCCommand& esc_commands)
{
  last_run_side_rotors = micros();
  front_PWM = esc_commands.front_motor_PWM;
  back_PWM = esc_commands.back_motor_PWM;
  right_PWM = esc_commands.right_motor_PWM;
  left_PWM = esc_commands.left_motor_PWM;
}

// For the battery voltage readings
int convolute_with_lp(float samples[])
{
    float sum = 0;
    for(int i = 0; i < filter_order + 1; i++) {
      sum+=samples[i]*taps[i];
    }
    return sum;
}

void precise_delay(uint8_t delay) {
  // 125-Offset to make sure pulse length is right
  _delay_us(125-4);
  switch(delay) {
    case 250 :
      _delay_us(1);
    case 249 :
      _delay_us(1);
    case 248 :
      _delay_us(1);
    case 247 :
      _delay_us(1);
    case 246 :
      _delay_us(1);
    case 245 :
      _delay_us(1);
    case 244 :
      _delay_us(1);
    case 243 :
      _delay_us(1);
    case 242 :
      _delay_us(1);
    case 241 :
      _delay_us(1);
    case 240 :
      _delay_us(1);
    case 239 :
      _delay_us(1);
    case 238 :
      _delay_us(1);
    case 237 :
      _delay_us(1);
    case 236 :
      _delay_us(1);
    case 235 :
      _delay_us(1);
    case 234 :
      _delay_us(1);
    case 233 :
      _delay_us(1);
    case 232 :
      _delay_us(1);
    case 231 :
      _delay_us(1);
    case 230 :
      _delay_us(1);
    case 229 :
      _delay_us(1);
    case 228 :
      _delay_us(1);
    case 227 :
      _delay_us(1);
    case 226 :
      _delay_us(1);
    case 225 :
      _delay_us(1);
    case 224 :
      _delay_us(1);
    case 223 :
      _delay_us(1);
    case 222 :
      _delay_us(1);
    case 221 :
      _delay_us(1);
    case 220 :
      _delay_us(1);
    case 219 :
      _delay_us(1);
    case 218 :
      _delay_us(1);
    case 217 :
      _delay_us(1);
    case 216 :
      _delay_us(1);
    case 215 :
      _delay_us(1);
    case 214 :
      _delay_us(1);
    case 213 :
      _delay_us(1);
    case 212 :
      _delay_us(1);
    case 211 :
      _delay_us(1);
    case 210 :
      _delay_us(1);
    case 209 :
      _delay_us(1);
    case 208 :
      _delay_us(1);
    case 207 :
      _delay_us(1);
    case 206 :
      _delay_us(1);
    case 205 :
      _delay_us(1);
    case 204 :
      _delay_us(1);
    case 203 :
      _delay_us(1);
    case 202 :
      _delay_us(1);
    case 201 :
      _delay_us(1);
    case 200 :
      _delay_us(1);
    case 199 :
      _delay_us(1);
    case 198 :
      _delay_us(1);
    case 197 :
      _delay_us(1);
    case 196 :
      _delay_us(1);
    case 195 :
      _delay_us(1);
    case 194 :
      _delay_us(1);
    case 193 :
      _delay_us(1);
    case 192 :
      _delay_us(1);
    case 191 :
      _delay_us(1);
    case 190 :
      _delay_us(1);
    case 189 :
      _delay_us(1);
    case 188 :
      _delay_us(1);
    case 187 :
      _delay_us(1);
    case 186 :
      _delay_us(1);
    case 185 :
      _delay_us(1);
    case 184 :
      _delay_us(1);
    case 183 :
      _delay_us(1);
    case 182 :
      _delay_us(1);
    case 181 :
      _delay_us(1);
    case 180 :
      _delay_us(1);
    case 179 :
      _delay_us(1);
    case 178 :
      _delay_us(1);
    case 177 :
      _delay_us(1);
    case 176 :
      _delay_us(1);
    case 175 :
      _delay_us(1);
    case 174 :
      _delay_us(1);
    case 173 :
      _delay_us(1);
    case 172 :
      _delay_us(1);
    case 171 :
      _delay_us(1);
    case 170 :
      _delay_us(1);
    case 169 :
      _delay_us(1);
    case 168 :
      _delay_us(1);
    case 167 :
      _delay_us(1);
    case 166 :
      _delay_us(1);
    case 165 :
      _delay_us(1);
    case 164 :
      _delay_us(1);
    case 163 :
      _delay_us(1);
    case 162 :
      _delay_us(1);
    case 161 :
      _delay_us(1);
    case 160 :
      _delay_us(1);
    case 159 :
      _delay_us(1);
    case 158 :
      _delay_us(1);
    case 157 :
      _delay_us(1);
    case 156 :
      _delay_us(1);
    case 155 :
      _delay_us(1);
    case 154 :
      _delay_us(1);
    case 153 :
      _delay_us(1);
    case 152 :
      _delay_us(1);
    case 151 :
      _delay_us(1);
    case 150 :
      _delay_us(1);
    case 149 :
      _delay_us(1);
    case 148 :
      _delay_us(1);
    case 147 :
      _delay_us(1);
    case 146 :
      _delay_us(1);
    case 145 :
      _delay_us(1);
    case 144 :
      _delay_us(1);
    case 143 :
      _delay_us(1);
    case 142 :
      _delay_us(1);
    case 141 :
      _delay_us(1);
    case 140 :
      _delay_us(1);
    case 139 :
      _delay_us(1);
    case 138 :
      _delay_us(1);
    case 137 :
      _delay_us(1);
    case 136 :
      _delay_us(1);
    case 135 :
      _delay_us(1);
    case 134 :
      _delay_us(1);
    case 133 :
      _delay_us(1);
    case 132 :
      _delay_us(1);
    case 131 :
      _delay_us(1);
    case 130 :
      _delay_us(1);
    case 129 :
      _delay_us(1);
    case 128 :
      _delay_us(1);
    case 127 :
      _delay_us(1);
    case 126 :
      _delay_us(1);
    case 125 :
      _delay_us(0);
  }
}

