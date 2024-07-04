#include "Adafruit_VL53L0X.h"
#include <loopTimer.h>
#include <PinChangeInterrupt.h>

// #define TRACKER_FRONT 18

#define FRONT_ADDRESS 0x30
#define LEFT_ADDRESS 0x31
#define RIGHT_ADDRESS 0x32

#define SHT_FRONT 30
#define SHT_LEFT 52
#define SHT_RIGHT 31

#define A2_PIN 8
#define MOTOR_B2_PIN 9

#define B1_PIN 5
#define A1_PIN 7

#define PWM_MOTOR_1 4
#define PWM_MOTOR_2 13

#define EN_PIN_1 A0
#define EN_PIN_2 A1

#define MOTOR_1 0
#define MOTOR_2 1

// ___PORT MANIPULATION___
#define SHT_FRONT_D30_PC7 7
#define SHT_LEFT_D52_PB1 1
#define SHT_RIGHT_D31_PC6 6

#define A2_PIN_D8_PH5 5
#define MOTOR_B2_PIN_D9_PH6 6

#define B1_PIN_D5_PE3 3
#define A1_PIN_D7_PH4 4

#define PWM_MOTOR_1_D4_PG5 5
#define PWM_MOTOR_2_D13_PB7 7

#define EN_PIN_1_A0_PF0 0
#define EN_PIN_2_A1_PF1 1


const byte VL53L0X_INTERRUPT_FRONT_PIN = 18;
const byte VL53L0X_INTERRUPT_RIGHT_PIN = 11;
const byte VL53L0X_INTERRUPT_LEFT_PIN = 10;

// #define TOP_D18_PD3 3
#define INTERRUPT_FRONT_D18_PD3 3
#define INTERRUPT_RIGHT_D11_PB5 5
#define INTERRUPT_LEFT_D10_PB4 4
bool flag = false;

// objects for the vl53l0x
Adafruit_VL53L0X front = Adafruit_VL53L0X();
Adafruit_VL53L0X left = Adafruit_VL53L0X();
Adafruit_VL53L0X right = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measureFront;
VL53L0X_RangingMeasurementData_t measureLeft;
VL53L0X_RangingMeasurementData_t measureRight;

void setID() {
  // all reset
  // digitalWrite(SHT_FRONT, LOW);
  // digitalWrite(SHT_LEFT, LOW);

  PORTC = B00000000;
  // PORTB = B00000000;
  PORTB = ~(_BV(1)) & PORTB;

  _delay_ms(10);
  // all unreset
  // digitalWrite(SHT_FRONT, HIGH);
  // digitalWrite(SHT_LEFT, HIGH);
  PORTC = B11000000;
  // PORTC = (_BV(SHT_FRONT_D30_PC7));
  PORTB = (_BV(1)) | PORTB;
  _delay_ms(10);

  // activating front and resetting left
  // digitalWrite(SHT_FRONT, HIGH);
  // digitalWrite(SHT_LEFT, LOW);
  PORTC = B10000000;
  // PORTC = (_BV(SHT_FRONT_D30_PC7));
  PORTB = ~(_BV(1)) & PORTB;

  // initing front
  if (!front.begin(FRONT_ADDRESS)) {
    // Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }

  _delay_ms(10);

  // Serial.println("Set GPIO Config so if range is lower the LowThreshold "
  //                "trigger Gpio Pin ");
  front.setGpioConfig(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                      VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
                      VL53L0X_INTERRUPTPOLARITY_LOW);

  // Set Interrupt Treashholds
  // Low reading set to 250mm  High Set to 350mm
  FixPoint1616_t LowThreashHold1 = (500 * 65536.0);
  FixPoint1616_t HighThreashHold1 = (501 * 65536.0);
  // Serial.println("Set Interrupt Threasholds... ");
  front.setInterruptThresholds(LowThreashHold1, HighThreashHold1, false);
  front.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);
  front.startMeasurement();

  // front.startRangeContinuous();
  // activating left
  // digitalWrite(SHT_LEFT, HIGH);
  PORTB = (_BV(1)) | PORTB;
  _delay_ms(10);


  //initing left
  if (!left.begin(LEFT_ADDRESS)) {
    // Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }

  // // Serial.println("Set GPIO Config so if range is lower the LowThreshold "
  // //                "trigger Gpio Pin ");
  // left.setGpioConfig(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
  //                    VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
  //                    VL53L0X_INTERRUPTPOLARITY_LOW);

  // // Set Interrupt Treashholds
  // // Low reading set to 100mm  High Set to 350mm
  // FixPoint1616_t LowThreashHold = (350 * 65536.0);
  // FixPoint1616_t HighThreashHold = (351 * 65536.0);
  // // Serial.println("Set Interrupt Threasholds... ");
  // left.setInterruptThresholds(LowThreashHold, HighThreashHold, false);
  // left.startRangeContinuous();
  // _delay_ms(10);

  // activating right
  PORTC = B11000000;
  //initing left
  if (!right.begin(RIGHT_ADDRESS)) {
    // Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }

  // // Serial.println("Set GPIO Config so if range is lower the LowThreshold "
  // //                "trigger Gpio Pin ");
  // right.setGpioConfig(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
  //                     VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
  //                     VL53L0X_INTERRUPTPOLARITY_LOW);

  // // Set Interrupt Treashholds
  // // Low reading set to 100mm  High Set to 350mm
  // FixPoint1616_t LowThreashHold2 = (350 * 65536.0);
  // FixPoint1616_t HighThreashHold2 = (351 * 65536.0);
  // // Serial.println("Set Interrupt Threasholds... ");
  // right.setInterruptThresholds(LowThreashHold2, HighThreashHold2, false);
  // right.startRangeContinuous();
  // _delay_ms(10);

  attachInterrupt(digitalPinToInterrupt(VL53L0X_INTERRUPT_FRONT_PIN), forward,
                  CHANGE);
  // attachPCINT(digitalPinToPCINT(VL53L0X_INTERRUPT_RIGHT_PIN), rightSen, CHANGE);
  // attachPCINT(digitalPinToPCINT(VL53L0X_INTERRUPT_LEFT_PIN), leftSen, CHANGE);
}

void read_dual_sensors() {


  front.rangingTest(&measureFront, false);
  // left.rangingTest(&measureLeft, false);
  // right.rangingTest(&measureRight, false);

  Serial.print(F("Front: "));
  Serial.print(measureFront.RangeMilliMeter);
  // // Serial.print(F(" Left: "));
  // // Serial.print(measureLeft.RangeMilliMeter);
  // // Serial.print(F(" Right: "));
  // // Serial.print(measureRight.RangeMilliMeter);
  Serial.println();
  if (measureFront.RangeMilliMeter >= 501) {
    // Serial.println("Tornado");
    OCR0A = 80;  // Set Output Compare Register A to 255 for 100% duty cycle on OC0A (pin 13)
    OCR0B = 80;  // Set Output Compare Register B to 255 for 100% duty cycle on OC0B (pin 4)
    // digitalWrite(A1_PIN, LOW);
    // digitalWrite(A2_PIN, HIGH);
    // PORTH = B00100000;
    // Clear PORTH and PORTE
    PORTH = 0x00;
    PORTE = 0x00;
    // Set the necessary bits in PORTH in a single instruction
    PORTH = _BV(A1_PIN_D7_PH4) | _BV(MOTOR_B2_PIN_D9_PH6);
    // digitalWrite(B1_PIN, HIGH);
    // digitalWrite(MOTOR_B2_PIN, LOW);
  }

  // else if (measureFront.RangeMilliMeter <= 250) {
  //   Serial.println("forward");
  //   // digitalWrite(A1_PIN, HIGH);
  //   // digitalWrite(A2_PIN, LOW);
  //   // digitalWrite(B1_PIN, HIGH);
  //   // digitalWrite(MOTOR_B2_PIN, LOW);
  //   PORTH = B00010000;
  //   PORTE = B00001000;
  // } else if (measureLeft.RangeMilliMeter <= 250) {
  //   if (measureLeft.RangeMilliMeter <= 250 && measureLeft.RangeMilliMeter > 100) {
  //     Serial.println("forwardRight");
  //     // digitalWrite(A1_PIN, LOW);
  //     // digitalWrite(A2_PIN, HIGH);
  //     // digitalWrite(B1_PIN, LOW);
  //     PORTH = B01100000;
  //     PORTE = B00000000;
  //   } else if (measureLeft.RangeMilliMeter <= 100) {
  //     Serial.println("forward");
  //     // digitalWrite(A1_PIN, HIGH);
  //     // digitalWrite(A2_PIN, LOW);
  //     // digitalWrite(B1_PIN, HIGH);
  //     // digitalWrite(MOTOR_B2_PIN, LOW);
  //     PORTH = B00010000;
  //     PORTE = B00001000;
  //     // delay_ms(5000);
  //   }
  // }
}

// void topISR() {
//   int state = (PIND & _BV(3)) >> 3;
//   if (state == HIGH) {
//     // Backward
//     // digitalWrite(A1_PIN, LOW);
//     // digitalWrite(A2_PIN, HIGH);
//     PORTH = B01100000;
//     // digitalWrite(B1_PIN, LOW);
//     // digitalWrite(MOTOR_B2_PIN, HIGH);
//     PORTE = B00100000;
//     for (volatile int i = 0; i < 5000; i++) {
//       Serial.println(i);
//     }
//   }
// }

void forward() {
  int stateForward = ((_BV(INTERRUPT_FRONT_D18_PD3)) & PIND) >> INTERRUPT_FRONT_D18_PD3;
  // if (digitalRead(VL53L0X_INTERRUPT_RIGHT_PIN) == L
  // if (digitalRead(VL53L0X_INTERRUPT_RIGHT_PIN) == LOW) {OW) {
  // Serial.print(F("Front: "));
  // Serial.print(measureFront.RangeMilliMeter);
  // // Serial.print(F(" Left: "));
  // // Serial.print(measureLeft.RangeMilliMeter);
  // // Serial.print(F(" Right: "));
  // // Serial.print(measureRight.RangeMilliMeter);
  // Serial.println();
  if (stateForward == LOW) {
    OCR0A = 255;  // Set Output Compare Register A to 255 for 100% duty cycle on OC0A (pin 13)
    OCR0B = 255;  // Set Output Compare Register B to 255 for 100% duty cycle on OC0B (pin 4)
    Serial.println("Forward!!!");
    // PORTH = B00010000;
    // PORTH = _BV(A1_PIN_D7_PH4);
    // // PORTE = B00101000;
    // PORTE = _BV(B1_PIN_D5_PE3) | PORTE;
    PORTH = B00000000;
    PORTE = B00000000;
    PORTH = _BV(A2_PIN_D8_PH5) | _BV(MOTOR_B2_PIN_D9_PH6);
  }
}

// void rightSen() {
//   int stateRight = ((_BV(INTERRUPT_RIGHT_D11_PB5)) & PINB) >> INTERRUPT_RIGHT_D11_PB5;
//   // Serial.println("State forward" + stateForward);
//   // if (digitalRead(VL53L0X_INTERRUPT_RIGHT_PIN) == L
//   // if (digitalRead(VL53L0X_INTERRUPT_RIGHT_PIN) == LOW) {OW) {
//   if (stateRight == LOW) {
//     flag =  true;
//     OCR0A = 80;  // Set Output Compare Register A to 255 for 100% duty cycle on OC0A (pin 13)
//     OCR0B = 80;  // Set Output Compare Register B to 255 for 100% duty cycle on OC0B (pin 4)
//     Serial.println("RightSen Works!");
//     PORTH = 0x00;
//     PORTE = 0x00;
//     PORTH = _BV(A2_PIN_D8_PH5);
//     PORTE = _BV(B1_PIN_D5_PE3);
//   }
// }

// void leftSen() {
//   int stateLeft = (PINB & (_BV(INTERRUPT_LEFT_D10_PB4))) >> INTERRUPT_LEFT_D10_PB4;
//   // Serial.println(stateLeft);
//   if (stateLeft == LOW) {
//     OCR0A = 80;  // Set Output Compare Register A to 255 for 100% duty cycle on OC0A (pin 13)
//     OCR0B = 80;  // Set Output Compare Register B to 255 for 100% duty cycle on OC0B (pin 4)
//     Serial.println("LeftSen Works!");
//     PORTH = 0x00;
//     PORTE = 0x00;
//     PORTH = _BV(A1_PIN_D7_PH4) | _BV(MOTOR_B2_PIN_D9_PH6);
//     flag = false;
//   }
// }
// PORTD, PORTB, PORTJ is INPUT

void setup() {
  Serial.begin(115200);
  // pinMode(A1_PIN, OUTPUT);
  // pinMode(A2_PIN, OUTPUT);

  // pinMode(B1_PIN, OUTPUT);
  // pinMode(MOTOR_B2_PIN, OUTPUT);

  // pinMode(PWM_MOTOR_1, OUTPUT);
  // pinMode(PWM_MOTOR_2, OUTPUT);

  // pinMode(EN_PIN_1, OUTPUT);
  // pinMode(EN_PIN_2, OUTPUT);

  // pinMode(TRACKER_FRONT, INPUT);


  // Set pin 7, 8, 9 as output
  DDRH = B01110000;
  // DDRH = (_BV(A1_PIN_D7_PH4)) | (_BV(A2_PIN_D8_PH5)) | (_BV(MOTOR_B2_PIN_D9_PH6));

  // Set pin 5 as output
  DDRE = B00001000;
  // DDRE = _BV(B1_PIN_D5_PE3);

  // Set pin 4 as output
  DDRG = B00100000;
  // DDRG = _BV(PWM_MOTOR_1_D4_PG5);

  // Set pin 13, 52 as output
  DDRB = B10000010;
  // DDRB = (_BV(PWM_MOTOR_2_D13_PB7)) | (_BV(SHT_LEFT_D52_PB1));

  // Set A0, A1 as output
  DDRF = B00000011;
  // DDRF = (_BV(EN_PIN_1_A0_PF0)) | (_BV(EN_PIN_2_A1_PF1));

  // set pin 18 as input
  // PORTD = (_BV(TOP_D18_PD3));

  // set pin 18, 11, 15 as input
  // PORTE = B00100000;
  PORTD = B00001000;
  PORTB = B00110000;
  // PORTJ = B00000001;
  // PORTB = _BV(INTERRUPT_RIGHT_D11_PB5);

  // attachInterrupt(digitalPinToInterrupt(TRACKER_FRONT), topISR, RISING);
  // Configure Timer 0 for Fast PWM mode
  TCCR0A = (_BV(WGM00)) | (_BV(WGM01)) | (_BV(COM0A1)) | (_BV(COM0B1));
  int myEraser = 7;       // this is 111 in binary and is used as an eraser
  TCCR0B &= ~myEraser;    // this operation (AND plus NOT), set the three bits in TCCR2B to 0
  int myPrescaler = 2;    // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.
  TCCR0B |= myPrescaler;  //this operation (OR), replaces the last three bits in TCCR2B with our new value 011

  // wait until serial port opens for native USB devices
  while (!Serial) { delay(1); }

  // Set pin 30, 31 as output
  // pinMode(SHT_FRONT, OUTPUT);
  DDRC = B11000000;
  // DDRC = _BV(SHT_FRONT_D30_PC7);

  // Set pin 52 as output
  // pinMode(SHT_LEFT, OUTPUT);

  // Serial.println(F("Shutdown pins inited..."));

  // digitalWrite(SHT_FRONT, LOW);
  PORTC = B00000000;
  // PORTC = ~(_BV(SHT_FRONT_D30_PC7)) & PORTC;

  // digitalWrite(SHT_LEFT, LOW);
  PORTB = ~(_BV(SHT_LEFT_D52_PB1)) & PORTB;

  // attachInterrupt(digitalPinToInterrupt(VL53L0X_INTERRUPT_FRONT_PIN), forward,
  //                 FALLING);
  // attachPCINT(digitalPinToPCINT(VL53L0X_INTERRUPT_RIGHT_PIN), rightSen, CHANGE);
  // attachPCINT(digitalPinToPCINT(VL53L0X_INTERRUPT_LEFT_PIN), leftSen, CHANGE);

  // Serial.println(F("Starting..."));
  setID();
  // digitalWrite(EN_PIN_1, HIGH);
  // digitalWrite(EN_PIN_2, HIGH);
  PORTF = (_BV(EN_PIN_1_A0_PF0)) | (_BV(EN_PIN_2_A1_PF1));
  // analogWrite(PWM_MOTOR_1, 255);
  // analogWrite(PWM_MOTOR_2, 255);

  OCR0A = 80;  // Set Output Compare Register A to 255 for 100% duty cycle on OC0A (pin 13)
  OCR0B = 80;  // Set Output Compare Register B to 255 for 100% duty cycle on OC0B (pin 4)

  // sen1.start(20);
  // sen1.start(15);
  // xTaskCreate(tornado, "Tornado", 200, NULL, 1, NULL);
  // xTaskCreate(forward, "Forward", 200, NULL, 1, NULL);
}

void loop() {
  // loopTimer.check(Serial);
  // Serial.println(digitalRead(TRACKER_FRONT));
  read_dual_sensors();
}