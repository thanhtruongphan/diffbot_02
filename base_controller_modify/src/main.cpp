#include "diffbot_base_config.h"
#include "base_controller.h"
#include "motor_l298_controller.h"

// Khởi tạo motor controller
L298MotorController motor_controller_right(MOTOR_RIGHT_DIR1_PIN, MOTOR_RIGHT_DIR2_PIN, MOTOR_RIGHT_PWM_PIN);
L298MotorController motor_controller_left(MOTOR_LEFT_DIR1_PIN, MOTOR_LEFT_DIR2_PIN, MOTOR_LEFT_PWM_PIN);

// Khởi tạo BaseController (đã loại bỏ ROS)
BaseController<L298MotorController, int> base_controller(&motor_controller_left, &motor_controller_right); 

void setup() {
  Serial.begin(57600); // Khởi tạo Serial với baud rate 115200
  Serial.println("Arduino started");

  // Khởi tạo encoder
  base_controller.initEncoders(ENCODER_RESOLUTION); 
}

void loop() {
  // Đọc dữ liệu từ serial
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    // Parse dữ liệu và điều khiển động cơ
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
      int speed_left = data.substring(1, commaIndex).toInt();
      int speed_right = data.substring(commaIndex + 1, data.length() - 1).toInt();
      base_controller.setSpeed(speed_left, speed_right);
    }
  }

  // Đọc encoder 
  base_controller.readEncoders(); 

  // Gửi dữ liệu encoder qua serial (chưa có IMU)
  long ticks_left = base_controller.getEncoderTicksLeft();
  long ticks_right = base_controller.getEncoderTicksRight();
  Serial.print("<");
  Serial.print(ticks_left);
  Serial.print(",");
  Serial.print(ticks_right);
  Serial.println(">"); 

  delay(10); // Delay một chút để tránh gửi dữ liệu quá nhanh
}