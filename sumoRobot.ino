#include <TimerOne.h>
#define echoPin 2
#define trigPin 7
#define echo_int 0
#define TIMER_US 50
#define TICK_COUNTS 4000

volatile long echo_start = 0;
volatile long echo_end = 0;
volatile long echo_duration = 0;
volatile int trigger_time_count = 0;

void echoSen() {
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Timer1.initialize(TIMER_US);
  Timer1.attachInterrupt(timerIsr);
  attachInterrupt(digitalPinToInterrupt(echoPin), echo_interrupt, CHANGE);
}
void loop() {
}

void timerIsr() {
  trigger_pulse();
}

void trigger_pulse() {
  static volatile int state = 0;

  if (!(--trigger_time_count)) {
    trigger_time_count = TICK_COUNTS;
    state = 1;
  }

  switch (state) {
    case 0:
      break;

    case 1:
      digitalWrite(trigPin, HIGH);
      state = 2;
      break;

    case 2:
    default:
      digitalWrite(trigPin, LOW);
      state = 0;
      break;
  }
}

void echo_interrupt() {
  switch (digitalRead(echoPin)) {
    case HIGH:
      echo_end = 0;
      echo_start = micros();
      break;

    case LOW:
      echo_end = micros();
      echo_duration = echo_end - echo_start;
      break;
  }
  if (echo_duration / 58 <= 45) {
    Serial.println(echo_duration / 58);
  }
}
