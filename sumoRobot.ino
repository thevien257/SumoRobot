#include <TimerOne.h>
#define echoPin 2
#define trigPin 7
#define echo_int 0
#define TIMER_US 50 // 50uS
#define TICK_COUNTS 4000 // 200mS

// Sử dụng Interrupt để không phụ thuộc vào delay, làm cản trở hệ thống -> Xử lý song song

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
  Timer1.initialize(TIMER_US); // Khởi tạo Timer Interrupt
  Timer1.attachInterrupt(timerIsr); // Hàm Timer Interrupt này sẽ được gọi mỗi 50 uS để bật/tắt phát xung tín hiệu của cảm biến
  attachInterrupt(digitalPinToInterrupt(echoPin), echo_interrupt, CHANGE); // Khi nhận được tín hiệu từ xung trả về của cảm biến (Khi có đối tượng ở trước mặt) thì hàm Interrupt này sẽ được kích hoạt ngay (để điều khiển Motor chạy tới)
}
void loop() {
}

void timerIsr() {
  trigger_pulse();
}

// Phát xung tín hiệu
void trigger_pulse() {
  static volatile int state = 0;

  if (!(--trigger_time_count)) { // Mỗi 200mS thì sẽ bật Trigger Pin để phát xung
    trigger_time_count = TICK_COUNTS;
    state = 1;
  }

  switch (state) {
    case 0:
      break;

    case 1:
      digitalWrite(trigPin, HIGH); // Bật trigger pin để phát xung
      state = 2;
      break;

    case 2: // Sau 50uS thì tắt Trigger Pin
    default:
      digitalWrite(trigPin, LOW);
      state = 0;
      break;
  }
}

// Nhận xung tín hiệu
void echo_interrupt() {
  switch (digitalRead(echoPin)) { // Đọc xung tín hiệu
    case HIGH: // Nếu xung lên mức cao (mức 1)
      echo_end = 0;
      echo_start = micros(); // Thời gian đầu tiên mà mức xung lên cao
      break;

    case LOW:
      echo_end = micros(); // Thời gian khi mà mức xung về lại mức thấp (mức 0)
      echo_duration = echo_end - echo_start; // Tính ra được thời gian của xung
      break;
  }
  if (echo_duration / 58 <= 45) { // Dựa vào thời gian để tính ra được khoảng cách của đối tượng
    Serial.println(echo_duration / 58);
  }
}
