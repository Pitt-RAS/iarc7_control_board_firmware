
// 1hz lower than the update rate of the teensy
const unsigned long rate_hz = 10;
const unsigned long loop_delay = 1000/rate_hz;

const unsigned long HEART_BEAT_HALF_PERIOD = 500;
const int HEART_BEAT_PIN = 13;
unsigned long delay_counter = 0;
bool heart = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(4800);
}

void loop() {
  // put your main code here, to run repeatedly:

  unsigned long start_time = millis();

  float val = analogRead(A7);
  float volts = val * (5.0/1024.0) * ((56.0+10.0)/10.0)*(12.02/12.31);

  float adjusted_range_volts = volts - 15.0;

  if(adjusted_range_volts < 0)
    adjusted_range_volts = 0;

  float raw_float = adjusted_range_volts * 255.0 / 10.0;

  byte raw = 0;
  if(raw_float > 255.0)
    raw = 255;
  else
    raw = raw_float;

  Serial.write(raw);

  if(millis() - start_time < loop_delay) {
    delay(loop_delay - (millis() - start_time));
  }

  // Blink heartbeat pin
  if(millis() - delay_counter > HEART_BEAT_HALF_PERIOD) {
    heart = heart ^ 1;
    digitalWrite(HEART_BEAT_PIN, heart);
    delay_counter = millis();
  }

}
