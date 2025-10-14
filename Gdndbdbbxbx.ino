// V2.0 - PID Line Follower

#include <Wire.h>
#include "Adafruit_TCS34725.h"

// --- Motor Pins ---
#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

// --- Sensor Pins ---
#define SENSOR_1_PIN 4 // Cảm biến ngoài cùng bên trái (S1)
#define SENSOR_2_PIN 3 // Cảm biến trong bên trái (S2)
#define SENSOR_3_PIN 1 // Cảm biến trong bên phải (S3)
#define SENSOR_4_PIN 0 // Cảm biến ngoài cùng bên phải (S4)

// --- LED Pins ---
#define W_LED_ON 20
#define IR_LED_ON 21

// --- CÀI ĐẶT CƠ BẢN ---
#define threshold 2800  // QUAN TRỌNG: Giá trị này phải được CALIBRATE lại!
#define MAX_SPEED 200   // Tốc độ tối đa mà motor có thể chạy (giới hạn 0-255)
#define BASE_SPEED 150  // Tốc độ cơ bản khi đi thẳng

// --- HẰNG SỐ PID - ĐÂY LÀ PHẦN QUAN TRỌNG NHẤT CẦN TUNE! ---
float Kp = 35;  // Tăng Kp để robot phản ứng mạnh hơn. Tăng quá sẽ bị rung lắc.
float Ki = 0;   // Dùng để xóa sai số nhỏ, tạm thời không dùng.
float Kd = 20;  // Tăng Kd để giảm rung lắc khi Kp cao, giúp robot ổn định hơn.

// --- Biến cho PID ---
int error = 0;
int lastError = 0;
float integral = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PWM_PIN_L_A, OUTPUT);
  pinMode(PWM_PIN_L_B, OUTPUT);
  pinMode(PWM_PIN_R_A, OUTPUT);
  pinMode(PWM_PIN_R_B, OUTPUT);

  pinMode(W_LED_ON, OUTPUT);
  pinMode(IR_LED_ON, OUTPUT);
  digitalWrite(W_LED_ON, 0);
  digitalWrite(IR_LED_ON, 1);
  
  analogWrite(W_LED_ON, 150); // Bật đèn cho cảm biến màu
  
  // Chờ 3 giây để đặt robot vào vị trí
  delay(3000); 
}

void loop() {
  // BƯỚC 1: Đọc giá trị cảm biến và tính toán "error" (sai số)
  bool s[4];
  s[0] = analogRead(SENSOR_1_PIN) > threshold; // Ngoài cùng trái
  s[1] = analogRead(SENSOR_2_PIN) > threshold; // Trong trái
  s[2] = analogRead(SENSOR_3_PIN) > threshold; // Trong phải
  s[3] = analogRead(SENSOR_4_PIN) > threshold; // Ngoài cùng phải

  // Gán giá trị sai số cho từng trường hợp
  // error = 0: ở giữa, error > 0: lệch sang trái, error < 0: lệch sang phải
  if      (s[0]==0 && s[1]==1 && s[2]==1 && s[3]==0) error = 0;   // 0110 -> Giữa line
  else if (s[0]==0 && s[1]==1 && s[2]==0 && s[3]==0) error = 1;   // 0100 -> Lệch trái nhẹ
  else if (s[0]==0 && s[1]==0 && s[2]==1 && s[3]==0) error = -1;  // 0010 -> Lệch phải nhẹ
  else if (s[0]==1 && s[1]==1 && s[2]==0 && s[3]==0) error = 2;   // 1100 -> Lệch trái
  else if (s[0]==0 && s[1]==0 && s[2]==1 && s[3]==1) error = -2;  // 0011 -> Lệch phải
  else if (s[0]==1 && s[1]==0 && s[2]==0 && s[3]==0) error = 3;   // 1000 -> Lệch trái nhiều
  else if (s[0]==0 && s[1]==0 && s[2]==0 && s[3]==1) error = -3;  // 0001 -> Lệch phải nhiều
  // Trường hợp mất line (0000)
  else if (s[0]==0 && s[1]==0 && s[2]==0 && s[3]==0) {
    if (lastError > 0) error = 4;   // Nếu lần cuối lệch trái, thì giờ rẽ trái gắt
    else error = -4;                // Nếu lần cuối lệch phải, thì giờ rẽ phải gắt
  }

  // BƯỚC 2: TÍNH TOÁN PID
  // P: Thành phần tỉ lệ
  float proportional = error;
  // I: Thành phần tích phân (tạm thời không dùng Ki=0)
  integral = integral + error;
  // D: Thành phần vi phân
  float derivative = error - lastError;

  // Tổng hợp lại thành một giá trị điều chỉnh
  float correction = Kp * proportional + Ki * integral + Kd * derivative;

  // Cập nhật lastError cho vòng lặp tiếp theo
  lastError = error;

  // BƯỚC 3: TÍNH TOÁN TỐC ĐỘ CHO 2 BÁNH XE
  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  // Giới hạn tốc độ trong khoảng 0 đến MAX_SPEED
  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  // BƯỚC 4: ĐIỀU KHIỂN MOTOR
  setMotorSpeeds(leftSpeed, rightSpeed);

  // In ra để debug và tune
  Serial.print("Error: "); Serial.print(error);
  Serial.print("\t Correction: "); Serial.print(correction);
  Serial.print("\t Left: "); Serial.print(leftSpeed);
  Serial.print("\t Right: "); Serial.println(rightSpeed);
}

// Hàm điều khiển motor cho gọn
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Bánh trái
  analogWrite(PWM_PIN_L_A, leftSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  // Bánh phải
  analogWrite(PWM_PIN_R_A, rightSpeed);
  analogWrite(PWM_PIN_R_B, 0);
}
