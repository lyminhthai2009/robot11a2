// V2.0 - PID Integrated

#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Cảm biến màu
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// --- Chân điều khiển động cơ ---
#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

// --- Chân cảm biến dò line (Analog) ---
#define SENSOR_1_PIN 4 // Cảm biến ngoài cùng bên trái
#define SENSOR_2_PIN 3 // Cảm biến trong bên trái
#define SENSOR_3_PIN 1 // Cảm biến trong bên phải
#define SENSOR_4_PIN 0 // Cảm biến ngoài cùng bên phải

// --- Chân điều khiển LED ---
#define W_LED_ON 20
#define IR_LED_ON 21

// --- Cài đặt cơ bản ---
#define threshold 3000 // Ngưỡng để xác định vạch đen, cần cân chỉnh theo môi trường
#define TCS_SENSOR     // Bật/tắt cảm biến màu

// =================================================================
// --- CÀI ĐẶT THÔNG SỐ PID ---
// Đây là những con số quan trọng nhất bạn cần tinh chỉnh
// =================================================================
float Kp = 35; // Tỷ lệ: Phản ứng với sai số hiện tại. Tăng Kp để robot phản ứng nhanh hơn.
float Ki = 0.05; // Tích phân: Khử sai số tích lũy theo thời gian. Giúp robot đi thẳng ở tâm.
float Kd = 25; // Vi phân: Giảm dao động, giúp robot ổn định khi vào cua.

// --- Tốc độ cơ bản của Robot ---
int baseSpeed = 150; // Tốc độ nền của robot (từ 0 đến 255)

// --- Các biến toàn cục cho PID ---
float error = 0;
float lastError = 0;
float P, I, D;
float motorSpeedCorrection;

void setup()
{
  Serial.begin(115200);

  pinMode(PWM_PIN_L_A, OUTPUT);
  pinMode(PWM_PIN_L_B, OUTPUT);
  pinMode(PWM_PIN_R_A, OUTPUT);
  pinMode(PWM_PIN_R_B, OUTPUT);

  pinMode(W_LED_ON, OUTPUT);
  pinMode(IR_LED_ON, OUTPUT);
  digitalWrite(W_LED_ON, 0);
  digitalWrite(IR_LED_ON, 1);

#ifdef TCS_SENSOR
  if (tcs.begin())
  {
    Serial.println("TCS34725 found");
  }
  else
  {
    Serial.println("No TCS34725 found ... check your connections");
    while (millis() < 10000)
    {
      digitalWrite(W_LED_ON, !digitalRead(W_LED_ON));
      delay(50);
    }
  }
  digitalWrite(W_LED_ON, 0);
#endif

  analogWrite(W_LED_ON, 150); // Bật đèn trắng
}

void loop()
{
  // --- Đọc giá trị cảm biến màu (nếu có) ---
  uint16_t r, g, b, c;
#ifdef TCS_SENSOR
  tcs.getRawData(&r, &g, &b, &c);
  // Nếu phát hiện màu đỏ, dừng lại
  if (r > (g * 1.6) && r > (b * 1.6))
  {
    stop();
    return;
  }
#endif

  // --- Đọc giá trị 4 cảm biến dò line ---
  bool s1 = analogRead(SENSOR_1_PIN) > threshold; // 1 = Trắng, 0 = Đen
  bool s2 = analogRead(SENSOR_2_PIN) > threshold;
  bool s3 = analogRead(SENSOR_3_PIN) > threshold;
  bool s4 = analogRead(SENSOR_4_PIN) > threshold;

  // --- Tính toán "Sai số" (Error) từ các cảm biến ---
  // Gán trọng số cho từng vị trí để xác định độ lệch so với tâm
  // Ví dụ: 0b0110 (giữa) -> error = 0. 0b0010 (lệch trái) -> error = -1. 0b1000 (lệch phải xa) -> error = 3
  if      (!s1 && !s2 && !s3 && !s4) { error = lastError; } // Mất line, giữ nguyên hướng rẽ cuối cùng
  else if (!s1 && !s2 && !s3 &&  s4) { error = -3; } // Lệch trái nhiều
  else if (!s1 && !s2 &&  s3 && !s4) { error = -1; } // Lệch trái ít
  else if (!s1 && !s2 &&  s3 &&  s4) { error = -2; }
  else if (!s1 &&  s2 && !s3 && !s4) { error =  1; } // Lệch phải ít
  else if (!s1 &&  s2 &&  s3 && !s4) { error =  0; } // Ở giữa, chuẩn!
  else if (!s1 &&  s2 &&  s3 &&  s4) { error = -1; } // Khúc cua nhẹ
  else if ( s1 && !s2 && !s3 && !s4) { error =  3; } // Lệch phải nhiều
  else if ( s1 && !s2 &&  s3 && !s4) { error =  0; } // Dành cho các trường hợp đặc biệt
  else if ( s1 &&  s2 && !s3 && !s4) { error =  2; }
  else if ( s1 &&  s2 &&  s3 && !s4) { error =  1; } // Khúc cua nhẹ
  else if ( s1 && s2 && s3 && s4) { error = 0; } // Gặp vạch ngang, đi thẳng

  // --- Thuật toán PID ---
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  
  // Giới hạn thành phần I để tránh "Integral Windup" (khi I tăng quá lớn)
  if (I > 300) I = 300;
  if (I < -300) I = -300;

  motorSpeedCorrection = (Kp * P) + (Ki * I) + (Kd * D);

  // --- Tính toán tốc độ cuối cùng cho mỗi động cơ ---
  int leftSpeed = baseSpeed + motorSpeedCorrection;
  int rightSpeed = baseSpeed - motorSpeedCorrection;

  // Giới hạn tốc độ trong khoảng cho phép của PWM (-255 đến 255)
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Gửi tín hiệu điều khiển tới động cơ
  setMotorSpeeds(leftSpeed, rightSpeed);

  // In ra Serial để theo dõi và tinh chỉnh
  Serial.print("Error: "); Serial.print(error);
  Serial.print("\t P: "); Serial.print(P);
  Serial.print("\t I: "); Serial.print(I);
  Serial.print("\t D: "); Serial.print(D);
  Serial.print("\t Correction: "); Serial.print(motorSpeedCorrection);
  Serial.print("\t Left: "); Serial.print(leftSpeed);
  Serial.print("\t Right: "); Serial.println(rightSpeed);
}

// --- Hàm điều khiển động cơ (có thể chạy lùi nếu tốc độ là số âm) ---
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  // Động cơ trái
  if (leftSpeed > 0) {
    analogWrite(PWM_PIN_L_A, leftSpeed);
    analogWrite(PWM_PIN_L_B, 0);
  } else {
    analogWrite(PWM_PIN_L_A, 0);
    analogWrite(PWM_PIN_L_B, -leftSpeed);
  }

  // Động cơ phải
  if (rightSpeed > 0) {
    analogWrite(PWM_PIN_R_A, rightSpeed);
    analogWrite(PWM_PIN_R_B, 0);
  } else {
    analogWrite(PWM_PIN_R_A, 0);
    analogWrite(PWM_PIN_R_B, -rightSpeed);
  }
}

// --- Hàm dừng robot ---
void stop()
{
  setMotorSpeeds(0, 0);
  Serial.println("STOP");
}
