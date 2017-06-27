
// 1hz lower than the update rate of the teensy
const int rate_hz = 29;
const int loop_delay = 1000/rate_hz;

const int HEART_BEAT_HALF_PERIOD = 500;
const int HEART_BEAT_PIN = 13;
long delay_counter = 0;
bool heart = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(4800);
}

void loop() {
  // put your main code here, to run repeatedly:

  int start_time = millis();

  float val = analogRead(A7);
  float volts = val * (5.0/1024.0) * ((56.0+10.0)/10.0)*(12.02/12.31);

  Serial.println(volts);

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
