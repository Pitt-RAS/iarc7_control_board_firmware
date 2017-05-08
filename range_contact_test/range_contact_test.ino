#include <Arduino.h>

const int rangefinder_pin=A2;

const bool SWITCH_TEST = false;
const bool FITTED = false;

const int LS_LEFT  = 2;
const int LS_RIGHT = 3;
const int LS_FRONT = 4;
const int LS_BACK  = 5;

bool switch_left;
bool switch_right;
bool switch_front;
bool switch_back;

float range;

//y = 7263.9 * x^(-0.627)   R² = 0.98277

void setup()
{
    Serial.begin(9600);
    
    pinMode(LS_LEFT, INPUT_PULLUP);
    pinMode(LS_RIGHT, INPUT_PULLUP);
    pinMode(LS_FRONT, INPUT_PULLUP);
    pinMode(LS_BACK, INPUT_PULLUP);

    /*
    // The field of view will need to be measured
    range_msg.field_of_view = 0.01;
    range_msg.min_range = 0.01;
    range_msg.max_range = 0.15;*/
}

void loop()
{
    switch_left  != digitalRead(LS_LEFT);
    switch_right != digitalRead(LS_RIGHT);
    switch_front != digitalRead(LS_FRONT);
    switch_back  != digitalRead(LS_BACK);
  
    range = getRange(rangefinder_pin);
    
    if (SWITCH_TEST)
    {
        Serial.print("LEFT:\t");
        Serial.println(switch_left);
        Serial.print("RIGHT:\t");
        Serial.println(switch_right);
        Serial.print("FRONT:\t");
        Serial.println(switch_front);
        Serial.print("BACK:\t");
        Serial.println(switch_back);
    }
    else 
    {
        if (FITTED)
        {
            Serial.print("RANGE:\t");
            Serial.println(range);
        }
        else
        {
            Serial.print("RANGE OUTPUT:\t");
            Serial.println(analogRead(rangefinder_pin));
        }
    }   
    delay(100);
}

double getRange(int pin_num)
{
    int sample;

    sample = analogRead(pin_num);
    
    float milli_volts = toMVolts(sample);
    
    float c = 7263.9;
    
    return pow(c/milli_volts, 1.594896332);
}

float toMVolts(float raw_input)
{
    return (1000.0 *  3.3 * (raw_input / 1023.0));
}


