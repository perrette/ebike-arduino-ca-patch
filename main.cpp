#include <Arduino.h>

// put function declarations here:
// int myFunction(int, int);

#define PIN_DIRECTION_IN 3
#define PIN_SPEED_IN 4
#define PIN_TORQUE_IN A0
#define PIN_THROTTLE_OUT 5
#define PIN_THROTTLE_CHECK_IN A1
#define ANALOG_TO_VOLT_IN 5. / 1023.
#define VOLT_TO_ANALOG_OUT 255. / 5.

// PAS DEVICE SETTINGS (voir Cycle Analyst PAS Device Settings : https://www.ebikes.ca/documents/CA3-1_settings.html#PasDTag)
#define BASE_TORQUE 1.7
#define MAX_TORQUE 3.0
#define POLES 48
#define TORQUE_SCALE 100 // Nm / V

// PAS CONFIG SETTINGS (voir Cycle Analyst PAS Config Settings : https://www.cycleanalyst.com/pas-config-settings/)
#define SPEED_START_THRES = 0.3      // sec / pole
#define SPEED_STOP_THRES = 0.19      // sec / pole
#define TORQUE_ASSIST_AVERAGING = 48 // poles

// Phaserunner settings
#define THROTTLE_OUT_NEUTRAL_VOLTAGE 0.8 // V

#define Throttle_Value TCA0.SINGLE.CMP2 // Raw PWM Throttle Value Range : [0-1023]

// LOOP OPTIMIZATION SETTINGS (unused)
#define Throttle_Value TCA0.SINGLE.CMP2 // Raw PWM Throttle Value Range : [0-1023]

// uint16_t counter;
// uint16_t last_speed_state;
bool moving_forward;
// unsigned long last_pole_time;
// unsigned long current_speed;
// unsigned long avg_speed;

// unsigned long speeds[TORQUE_ASSIST_AVERAGING];
uint64_t POLES_SINCE_START_AVG = 0;
uint64_t TIME_SINCE_START_AVG = 0;
uint64_t START_AVG_TIME = 0;

// Global state variables
int direction;
int speed;
float torque;
float throttle_out;

void update_inputs()
{
  direction = digitalRead(PIN_DIRECTION_IN);
  speed = digitalRead(PIN_SPEED_IN);
  torque = analogRead(PIN_TORQUE_IN) * ANALOG_TO_VOLT_IN;
}

void init_counter()
{
  // counter = 0;
  // last_speed_state = LOW;
  // last_pole_time = millis();
  moving_forward = false;
}

void start_moving()
{
  // counter = 0;
  // last_speed_state = LOW;
  // last_pole_time = millis();
  POLES_SINCE_START_AVG = 0;
  TIME_SINCE_START_AVG = 0;
  START_AVG_TIME = millis();
  moving_forward = true;
}

void stop_moving()
{
  moving_forward = false;
  throttle_out = THROTTLE_OUT_NEUTRAL_VOLTAGE;
}

void check_direction()
{

  direction = digitalRead(PIN_DIRECTION_IN);

  if (direction == LOW)
  {
    stop_moving();
  }
}

void check_speed()
{

  check_direction(); // we check the direction to update the moving_forward state

  if (direction == HIGH && !moving_forward)
  {
    start_moving();
  }

  speed = digitalRead(PIN_SPEED_IN);
  // only update speed and counter if we are moving forward, otherwise we are not interested in the speed and counter values
  if (!moving_forward)
  {
    return;
  }
  // we are moving forward

  // // counter++ ;
  // unsigned long current_time = millis();
  // if (last_speed_state == LOW && digitalRead(PIN_SPEED_IN) == HIGH) {
  //   current_speed = 1000. / (current_time - last_pole_time); // Speed in poles per second
  //   last_pole_time = current_time;
  // }
  // last_speed_state = digitalRead(PIN_SPEED_IN);
}

void setup()
{
  pinMode(PIN_DIRECTION_IN, INPUT);
  pinMode(PIN_SPEED_IN, INPUT);
  pinMode(PIN_TORQUE_IN, INPUT);
  pinMode(PIN_THROTTLE_OUT, OUTPUT);
  pinMode(PIN_THROTTLE_CHECK_IN, INPUT);
  Serial.begin(115200);

  init_counter();
  attachInterrupt(digitalPinToInterrupt(PIN_DIRECTION_IN), check_direction, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED_IN), check_speed, CHANGE);

  // TCA0.SINGLE.CTRLA = 0;         // Disable TimerA during init
  // TCA0.SINGLE.CTRLB = B01000011; // Single Slope PWM linked to D5 (https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-09-DataSheet-DS40002173C.pdf) Page 207
  // TCA0.SINGLE.PER = 1023;        // Max TimerA Value = 1023 so max frequency is : 1/(1023/16Mhz) = ~15.6Khz
  // Throttle_Value = 0;            // Init first throttle value
  // TCA0.SINGLE.CTRLA = 1;         // Enable TimerA
}

void loop()
{
  update_inputs();
  float torque_cursor = (torque - BASE_TORQUE) / (MAX_TORQUE - BASE_TORQUE);
  // float throttle_out = torque_cursor * (torque_cursor > 0 ? 1 : 0) * (direction == HIGH ? 1 : 0) * (speed == HIGH ? 1 : 0);
  float throttle_out = torque;

  analogWrite(PIN_THROTTLE_OUT, int(throttle_out * VOLT_TO_ANALOG_OUT + 0.5)); // Convert back to 0-255 range for PWM output
  // Throttle_Value = int(throttle_out / ANALOG_TO_VOLT_IN + 0.5); // Convert back to 0-1023 range for PWM output

  float throttle_check = analogRead(PIN_THROTTLE_CHECK_IN) * ANALOG_TO_VOLT_IN;

  Serial.print(">");
  Serial.print("DIRECTION_IN:");
  Serial.print(direction);
  Serial.print(",");
  Serial.print("SPEED_IN:");
  Serial.print(speed);
  Serial.print(",");
  Serial.print("TORQUE_IN:");
  Serial.print(torque);
  Serial.print(",");
  Serial.print("THROTTLE_OUT:");
  Serial.print(throttle_out);
  Serial.print(",");
  Serial.print("THROTTLE_CHECK_IN:");
  Serial.print(throttle_check);
  Serial.println(); // Writes \r\n
  delay(100);
}