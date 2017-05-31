#include <Bounce2.h>
#include <Arduino.h>

const int rangefinder_pin=A2;

const bool SWITCH_TEST = true;
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

Bounce left;
Bounce right;
Bounce front;
Bounce back;

//y = 4178.8 * x^(-0.655) fit 1
//y = 3824.5 * x^(-0.63) fit 2

void setup()
{
    Serial.begin(9600);
    
    pinMode(LS_LEFT, INPUT_PULLUP);
    pinMode(LS_RIGHT, INPUT_PULLUP);
    pinMode(LS_FRONT, INPUT_PULLUP);
    pinMode(LS_BACK, INPUT_PULLUP);

    // After setting up the button, setup the object
    left.attach(LS_LEFT);
    left.interval(1);
    
    right.attach(LS_RIGHT);
    right.interval(1);
    
    front.attach(LS_FRONT);
    front.interval(1);
    
    back.attach(LS_BACK);
    back.interval(1);
}

void loop()
{
    left.update();
    right.update();
    front.update();
    back.update();
    
    switch_left  = left.read();
    switch_right = right.read();
    switch_front = front.read();
    switch_back  = back.read();
  
    range = getRange(rangefinder_pin);
    
    if (SWITCH_TEST){
        Serial.print("LEFT:\t");
        Serial.println(switch_left);
        Serial.print("RIGHT:\t");
        Serial.println(switch_right);
        Serial.print("FRONT:\t");
        Serial.println(switch_front);
        Serial.print("BACK:\t");
        Serial.println(switch_back);
    }
    else {
        if (FITTED){
            Serial.print("RANGE:\t");
            Serial.println(range);
        }
        else {
            Serial.print("RANGE OUTPUT:\t");
            Serial.println(analogRead(rangefinder_pin));
        }
    } 
    delay(100);
}

double getRange(int pin_num)
{
    float c = 4178.8;
    float power = 1/0.655;
    
    int sample = analogRead(pin_num);

    return pow(c/sample, power);
}


