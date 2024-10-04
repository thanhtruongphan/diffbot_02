// modified for arduino mega & motor GA25-370 130rpm 12v

#include <Arduino.h>
#include <unity.h>

// Khai báo các chân điều khiển động cơ sử dụng L298
// banh 1
const int motorPin1 = 44;  // Chân điều khiển chiều 1
const int motorPin2 = 45;  // Chân điều khiển chiều 2
const int enablePin = 4;  // Chân PWM để điều khiển tốc độ

// // banh 2
// const int motorPin1 = 46;  // Chân điều khiển chiều 1
// const int motorPin2 = 47;  // Chân điều khiển chiều 2
// const int enablePin = 13;  // Chân PWM để điều khiển tốc độ

// Lớp điều khiển động cơ sử dụng L298
class L298MotorController {
  public:
    L298MotorController(int pin1, int pin2, int enPin) : _pin1(pin1), _pin2(pin2), _enPin(enPin) {}

    void begin() {
      pinMode(_pin1, OUTPUT);
      pinMode(_pin2, OUTPUT);
      pinMode(_enPin, OUTPUT);
      stop();
    }

    void setSpeed(int speed) {
      if (speed > 0) {
        digitalWrite(_pin1, HIGH);
        digitalWrite(_pin2, LOW);
        analogWrite(_enPin, speed);
      } else if (speed < 0) {
        digitalWrite(_pin1, LOW);
        digitalWrite(_pin2, HIGH);
        analogWrite(_enPin, -speed);  // Điều khiển PWM với giá trị tuyệt đối của tốc độ
      } else {
        stop();
      }
    }

    void stop() {
      digitalWrite(_pin1, LOW);
      digitalWrite(_pin2, LOW);
      analogWrite(_enPin, 0);
    }

  private:
    int _pin1, _pin2, _enPin;
};

// Khởi tạo bộ điều khiển động cơ
L298MotorController motorController(motorPin1, motorPin2, enablePin);

void motorSweep(L298MotorController& motorCtrl) {
  int i;
  for (i = 0; i < 255; i++) {
    motorCtrl.setSpeed(i);
    delay(10);
  }
  for (i = 255; i != 0; i--) {
    motorCtrl.setSpeed(i);
    delay(10);
  }

  Serial.print("tock");

  for (i = 0; i > -255; i--) {
    motorCtrl.setSpeed(i);
    delay(10);
  }
  for (i = -255; i != 0; i++) {
    motorCtrl.setSpeed(i);
    delay(10);
  }

  Serial.print("tech");
  motorCtrl.setSpeed(0);
  delay(1000);
}

void setup() {
  delay(2000);
  UNITY_BEGIN();

  Serial.begin(9600);  // Khởi tạo giao tiếp Serial
  Serial.println("L298MotorController Test");

  motorController.begin();  // Khởi tạo bộ điều khiển L298
}

uint8_t cycle = 0;
uint8_t max_loop_cycles = 1;

void loop() {
  if (cycle < max_loop_cycles) {
    Serial.print("tick");

    motorSweep(motorController);  // Điều khiển động cơ theo hàm motorSweep
  } else {
    motorController.setSpeed(0);
    delay(1000);
    UNITY_END();
  }

  cycle++;
}
