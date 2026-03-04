#include <Arduino.h>

#define PIN_DIRECTION_IN 3
#define PIN_SPEED_IN 4
#define PIN_TORQUE_IN A0
#define PIN_THROTTLE_OUT 5
#define PIN_THROTTLE_CHECK_IN A1
#define ANALOG_TO_VOLT_IN 5. / 1023.
#define VOLT_TO_ANALOG_OUT 255. / 5.

// Phaserunner settings
#define THROTTLE_OUT_BRAKE_MAX_VOLTAGE 0    // V
#define THROTTLE_OUT_BRAKE_OFF_VOLTAGE 0.85 // V
#define THROTTLE_OUT_NEUTRAL_VOLTAGE 1      // V
#define THROTTLE_OUT_MIN_VOLTAGE 1.1        // V
#define THROTTLE_OUT_MAX_VOLTAGE 3.8        // V

// PAS DEVICE SETTINGS (voir Cycle Analyst PAS Device Settings :
// https://www.ebikes.ca/documents/CA3-1_settings.html#PasDTag)
#define BASE_TORQUE 1.7
#define MAX_TORQUE 3.0
#define POLES 48
#define TORQUE_SCALE 100 // Nm / V

// PAS CONFIG SETTINGS (voir Cycle Analyst PAS Config Settings :
// https://www.cycleanalyst.com/pas-config-settings/)
// Start when period < threshold; stop when period > threshold. STOP > START for hysteresis.
#define SPEED_START_THRES 0.19f   // sec/pole — start pedaling when period below this
#define SPEED_STOP_THRES 0.3f    // sec/pole — stop pedaling when period above this
#define TORQUE_ASSIST_AVERAGING POLES  // number of poles for sliding average

// THROTTLE IN SETTINGS
#define THROTTLE_IN_MAX_VOLTAGE 3.8        // V
#define THROTTLE_IN_MIN_VOLTAGE 1.1        // V
#define THROTTLE_IN_NEUTRAL_VOLTAGE 1      // V
#define THROTTLE_IN_BRAKE_MAX_VOLTAGE 0    // V
#define THROTTLE_IN_BRAKE_OFF_VOLTAGE 0.85 // V

// ANALOG E-BRAKE SETTINGS
#define ANALOG_BRAKE_MAX_VOLTAGE 4.5    // V
#define ANALOG_BRAKE_OFF_VOLTAGE 1      // V

// Power at which throttle reaches max (W). Throttle maps [0, TORQUE_ASSIST_P_MAX_W] -> [MIN, max_effective].
#define TORQUE_ASSIST_P_MAX_W 250.0f

// LOOP OPTIMIZATION SETTINGS (unused)
#define Throttle_Value TCA0.SINGLE.CMP2 // Raw PWM Throttle Value Range : [0-1023]

// These parameters will be set by the user in the settings menu
int TORQUE_ASSIST_MAX_PERCENT = 100;  // % — modulates max assist throttle
int THROTTLE_IN_MAX_PERCENT = 100;    // %

// --- ISR state (micros(); keep ISR under ~500 µs) ---
volatile unsigned long last_pole_time_us = 0;
volatile unsigned long period_us = 0;
volatile bool new_pole = false;
volatile bool first_pole_speed = false;

// --- Pedaling state ---
volatile int direction = LOW;
bool moving_forward = false;

// --- Sliding window of timestamps (updated in loop when new_pole) ---
unsigned long pole_timestamps[TORQUE_ASSIST_AVERAGING];
uint8_t write_index = 0;
uint8_t pole_count = 0;

// --- Loop state ---
float torque_V = 0.0f;
float throttle_out_V = 0.0f;

void init_counter() {
  moving_forward = false;
  first_pole_speed = false;
  pole_count = 0;
  write_index = 0;
}

void start_moving() {
  pole_count = 0;
  write_index = 0;
  moving_forward = true;
}

void stop_moving() {
  moving_forward = false;
  first_pole_speed = false;
  throttle_out_V = THROTTLE_OUT_NEUTRAL_VOLTAGE;
}

// ISR: keep minimal (< 500 µs). Only direction + timestamp/period.
void check_direction() {
  direction = digitalRead(PIN_DIRECTION_IN);
  if (direction == LOW) {
    stop_moving();
  }
}

// ISR: one edge per pole (RISING). Only update period_us and last_pole_time_us.
void check_speed() {
  if (digitalRead(PIN_DIRECTION_IN) == LOW)
    return;
  if (!moving_forward)
    start_moving();
  unsigned long now = micros();
  if (!first_pole_speed) {
    first_pole_speed = true;
    last_pole_time_us = now;
    return;
  }
  period_us = now - last_pole_time_us;
  last_pole_time_us = now;
  new_pole = true;
}

void setup() {
  pinMode(PIN_DIRECTION_IN, INPUT);
  pinMode(PIN_SPEED_IN, INPUT);
  pinMode(PIN_TORQUE_IN, INPUT);
  pinMode(PIN_THROTTLE_OUT, OUTPUT);
  pinMode(PIN_THROTTLE_CHECK_IN, INPUT);
  Serial.begin(115200);

  init_counter();
  attachInterrupt(digitalPinToInterrupt(PIN_DIRECTION_IN), check_direction,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED_IN), check_speed, RISING);
}

void loop() {
  torque_V = analogRead(PIN_TORQUE_IN) * ANALOG_TO_VOLT_IN;

  unsigned long period_us_local = period_us;
  int dir = direction;

  // Sliding window: on new pole, add timestamp (in loop to avoid long ISR)
  if (new_pole) {
    new_pole = false;
    pole_timestamps[write_index] = last_pole_time_us;
    write_index = (write_index + 1) % TORQUE_ASSIST_AVERAGING;
    if (pole_count < TORQUE_ASSIST_AVERAGING)
      pole_count++;
  }

  float period_s = (period_us_local > 0) ? (period_us_local / 1000000.0f) : 0.0f;

  // Pedaling state: direction backward -> stop
  if (dir == LOW) {
    stop_moving();
  } else if (moving_forward) {
    // Timeout: no pole for 2 * SPEED_STOP_THRES
    unsigned long timeout_us = (unsigned long)(2.0f * SPEED_STOP_THRES * 1000000.0f);
    if ((micros() - last_pole_time_us) > timeout_us)
      stop_moving();
    else if (period_s > SPEED_STOP_THRES)
      stop_moving();
  }
  if (moving_forward && period_s > 0.0f && period_s < SPEED_START_THRES) {
    // Already started; keep pedaling
  } else if (!moving_forward && period_s > 0.0f && period_s < SPEED_START_THRES && dir == HIGH) {
    start_moving();
  }

  if (!moving_forward || dir == LOW) {
    throttle_out_V = THROTTLE_OUT_NEUTRAL_VOLTAGE;
  } else {
    // Torque (Nm), clamp to [0, TORQUE_SCALE]
    float torque_Nm = (torque_V - BASE_TORQUE) / (MAX_TORQUE - BASE_TORQUE) * (float)TORQUE_SCALE;
    if (torque_Nm < 0.0f) torque_Nm = 0.0f;
    if (torque_Nm > (float)TORQUE_SCALE) torque_Nm = (float)TORQUE_SCALE;

    // Average period from timestamp window (s)
    float avg_period_s = period_s;
    if (pole_count >= 2) {
      uint8_t oldest_ix = (pole_count < TORQUE_ASSIST_AVERAGING) ? 0 : write_index;
      uint8_t newest_ix = (pole_count < TORQUE_ASSIST_AVERAGING)
        ? (pole_count - 1)
        : (write_index + TORQUE_ASSIST_AVERAGING - 1) % TORQUE_ASSIST_AVERAGING;
      unsigned long span_us = pole_timestamps[newest_ix] - pole_timestamps[oldest_ix];
      avg_period_s = (float)span_us / (float)(pole_count - 1) / 1000000.0f;
    }
    if (avg_period_s <= 0.0f) avg_period_s = period_s;

    // omega = 2*pi / (POLES * T_pole)
    float omega_rad_s = (avg_period_s > 0.0f)
      ? (2.0f * 3.14159265f / ((float)POLES * avg_period_s))
      : 0.0f;
    float P_W = torque_Nm * omega_rad_s;

    // Throttle range with modulation
    float throttle_max_effective = THROTTLE_OUT_MIN_VOLTAGE +
      (THROTTLE_OUT_MAX_VOLTAGE - THROTTLE_OUT_MIN_VOLTAGE) * (float)TORQUE_ASSIST_MAX_PERCENT / 100.0f;
    throttle_out_V = THROTTLE_OUT_MIN_VOLTAGE +
      (P_W / TORQUE_ASSIST_P_MAX_W) * (throttle_max_effective - THROTTLE_OUT_MIN_VOLTAGE);
    if (throttle_out_V < THROTTLE_OUT_MIN_VOLTAGE) throttle_out_V = THROTTLE_OUT_MIN_VOLTAGE;
    if (throttle_out_V > throttle_max_effective) throttle_out_V = throttle_max_effective;
  }

  // Clamp and PWM (use constants only)
  float v = throttle_out_V;
  if (v < THROTTLE_OUT_MIN_VOLTAGE) v = THROTTLE_OUT_MIN_VOLTAGE;
  if (v > THROTTLE_OUT_MAX_VOLTAGE) v = THROTTLE_OUT_MAX_VOLTAGE;
  int pwm = (int)(v * VOLT_TO_ANALOG_OUT + 0.5f);
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  analogWrite(PIN_THROTTLE_OUT, pwm);

  float throttle_check = analogRead(PIN_THROTTLE_CHECK_IN) * ANALOG_TO_VOLT_IN;
  Serial.print(">DIR:");
  Serial.print(dir);
  Serial.print(",MOV:");
  Serial.print(moving_forward ? 1 : 0);
  Serial.print(",TORQUE_V:");
  Serial.print(torque_V);
  Serial.print(",P:");
  Serial.print(pole_count);
  Serial.print(",THR:");
  Serial.print(throttle_out_V);
  Serial.print(",CHK:");
  Serial.println(throttle_check);
  delay(100);
}
