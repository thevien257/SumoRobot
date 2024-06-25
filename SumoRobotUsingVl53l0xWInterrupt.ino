#include "Adafruit_VL53L0X.h"
#include "millisDelay.h"
#include <BufferedOutput.h>
#include <loopTimer.h>
#include <PinChangeInterrupt.h>


millisDelay sen1;
millisDelay sen2;
createBufferedOutput(bufferedOut, 80, DROP_UNTIL_EMPTY);

int top = 18;

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 30
#define SHT_LOX2 52
const byte VL53LOX_InterruptPinForward = 3;
const byte VL53LOX_InterruptPinRight = 12;

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

#define BRAKE 0
#define CW 1
#define CCW 2
#define CS_THRESHOLD 15  // Definition of safety current (Check: "1.3 Monster Shield Example").

#define A2_PIN 8
#define MOTOR_B2_PIN 9

#define B1_PIN 5
#define A1_PIN 7


#define PWM_MOTOR_1 4
#define PWM_MOTOR_2 13

#define CURRENT_SEN_1 A2
#define CURRENT_SEN_2 A3

#define EN_PIN_1 A0
#define EN_PIN_2 A1

#define MOTOR_1 0
#define MOTOR_2 1

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  // digitalWrite(SHT_LOX1, LOW);
  // digitalWrite(SHT_LOX2, LOW);

  PORTC = B00000000;
  PORTB = B00000000;
  _delay_ms(10);
  // all unreset
  // digitalWrite(SHT_LOX1, HIGH);
  // digitalWrite(SHT_LOX2, HIGH);
  PORTC = B10000000;
  PORTB = B00000010;
  _delay_ms(10);

  // activating LOX1 and resetting LOX2
  // digitalWrite(SHT_LOX1, HIGH);
  // digitalWrite(SHT_LOX2, LOW);
  PORTC = B10000000;
  PORTB = B00000000;
  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  lox1.startRangeContinuous();
  _delay_ms(10);

  // activating LOX2
  // digitalWrite(SHT_LOX2, HIGH);
  PORTB = B00000010;
  _delay_ms(10);

  //initing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }

  Serial.println("Set GPIO Config so if range is lower the LowThreshold "
                 "trigger Gpio Pin ");
  lox2.setGpioConfig(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                     VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
                     VL53L0X_INTERRUPTPOLARITY_LOW);

  // Set Interrupt Treashholds
  // Low reading set to 50mm  High Set to 100mm
  FixPoint1616_t LowThreashHold = (250 * 65536.0);
  FixPoint1616_t HighThreashHold = (350 * 65536.0);
  Serial.println("Set Interrupt Threasholds... ");
  lox2.setInterruptThresholds(LowThreashHold, HighThreashHold, true);

  lox2.startRangeContinuous();
}

volatile bool toggleState = false;


void read_dual_sensors() {

  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);  // pass in 'true' to get debug data printout!

  if (measure1.RangeMilliMeter >= 350 && measure2.RangeMilliMeter >= 350) { 
    Serial.println("Tornado");
    // digitalWrite(A1_PIN, LOW);
    // digitalWrite(A2_PIN, HIGH);
    PORTH = B00100000;
    PORTE = B00001000;
    // digitalWrite(B1_PIN, HIGH);
    // digitalWrite(MOTOR_B2_PIN, LOW);
  }
  // else if (measure1.RangeMilliMeter <= 250) {
  //   Serial.println("forward");
  //   // digitalWrite(A1_PIN, HIGH);
  //   // digitalWrite(A2_PIN, LOW);
  //   // digitalWrite(B1_PIN, HIGH);
  //   // digitalWrite(MOTOR_B2_PIN, LOW);
  //   PORTH = B00010000;
  //   PORTE = B00001000;
  // } else if (measure2.RangeMilliMeter <= 250) {
  //   if (measure2.RangeMilliMeter <= 250 && measure2.RangeMilliMeter > 100) {
  //     Serial.println("forwardRight");
  //     // digitalWrite(A1_PIN, LOW);
  //     // digitalWrite(A2_PIN, HIGH);
  //     // digitalWrite(B1_PIN, LOW);
  //     PORTH = B01100000;
  //     PORTE = B00000000;
  //   } else if (measure2.RangeMilliMeter <= 100) {
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

void topISR() {
  int state = (PIND & 1 << 3) >> 3;
  if (state == HIGH) {
    // Backward
    toggleState = true;
    // digitalWrite(A1_PIN, LOW);
    // digitalWrite(A2_PIN, HIGH);
    PORTH = B01100000;
    // digitalWrite(B1_PIN, LOW);
    // digitalWrite(MOTOR_B2_PIN, HIGH);
    PORTE = B00000000;
    for (volatile int i = 0; i < 5000; i++) {
      Serial.println(i);
    }
  }
}

void forward() {
  if (digitalRead(VL53LOX_InterruptPinRight) == LOW) {
    Serial.println("forward!!!");
    PORTH = B00010000;
    PORTE = B00001000;
  }
}


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

  // pinMode(top, INPUT);


  // Set pin 7, 8, 9 as output
  DDRH = B01110000;

  // Set pin 5 as output
  DDRE = B00001000;

  // Set pin 4 as output
  DDRG = B00100000;

  // Set pin 13, 52 as output
  DDRB = B10000010;

  // Set A0, A1 as output
  DDRF = B00000011;

  // set pin 18 as input
  PORTD = B00001000;

  attachInterrupt(digitalPinToInterrupt(top), topISR, RISING);
  // Configure Timer 0 for Fast PWM mode
  TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1);
  int myEraser = 7;       // this is 111 in binary and is used as an eraser
  TCCR0B &= ~myEraser;    // this operation (AND plus NOT), set the three bits in TCCR2B to 0
  int myPrescaler = 2;    // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.
  TCCR0B |= myPrescaler;  //this operation (OR), replaces the last three bits in TCCR2B with our new value 011

  // wait until serial port opens for native USB devices
  while (!Serial) { delay(1); }

  // Set pin 30 as output
  // pinMode(SHT_LOX1, OUTPUT);
  DDRC = B10000000;

  // Set pin 52 as output
  // pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  // digitalWrite(SHT_LOX1, LOW);
  PORTC = B00000000;
  // digitalWrite(SHT_LOX2, LOW);
  PORTB = B00000000;
  pinMode(VL53LOX_InterruptPinForward, INPUT);
  pinMode(VL53LOX_InterruptPinRight, INPUT);

  // attachInterrupt(digitalPinToInterrupt(VL53LOX_InterruptPin), forward,
  //                 FALLING);
  attachPCINT(digitalPinToPCINT(VL53LOX_InterruptPinRight), forward, CHANGE);
  Serial.println(F("Both in reset mode...(pins are low)"));


  Serial.println(F("Starting..."));
  setID();
  // digitalWrite(EN_PIN_1, HIGH);
  // digitalWrite(EN_PIN_2, HIGH);
  PORTF = B00000011;
  // analogWrite(PWM_MOTOR_1, 255);
  // analogWrite(PWM_MOTOR_2, 255);

  OCR0A = 255;  // Set Output Compare Register A to 255 for 100% duty cycle on OC0A (pin 13)
  OCR0B = 255;  // Set Output Compare Register B to 255 for 100% duty cycle on OC0B (pin 4)

  // sen1.start(20);
  // sen1.start(15);
  // xTaskCreate(tornado, "Tornado", 200, NULL, 1, NULL);
  // xTaskCreate(forward, "Forward", 200, NULL, 1, NULL);
}

void loop() {
  loopTimer.check(Serial);
  Serial.println(digitalRead(top));
  read_dual_sensors();
}
