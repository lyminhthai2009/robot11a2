// V1.2 - Improved Line Following Logic

#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// --- Motor Pins ---
#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

// --- Sensor Pins ---
#define SENSOR_1_PIN 4 // Cảm biến ngoài cùng bên trái
#define SENSOR_2_PIN 3 // Cảm biến trong bên trái
#define SENSOR_3_PIN 1 // Cảm biến trong bên phải
#define SENSOR_4_PIN 0 // Cảm biến ngoài cùng bên phải

// --- LED Pins ---
#define W_LED_ON 20
#define IR_LED_ON 21

// --- IMPORTANT: CALIBRATE THIS VALUE! ---
// GIÁ TRỊ NÀY CẦN ĐƯỢC HIỆU CHỈNH. XEM HƯỚNG DẪN BÊN TRÊN
#define threshold 2800 // Ví dụ sau khi calibrate

#define TCS_SENSOR // Bật/tắt cảm biến màu

// --- Tốc độ cơ bản, nên bắt đầu với giá trị thấp (ví dụ 90-110) ---
int baseSpeed = 110;

int last_error = 0; // Biến "trí nhớ" để xử lý khi mất line

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
  digitalWrite(IR_LED_ON, 1); // Bật LED hồng ngoại cho cảm biến line

#ifdef TCS_SENSOR
  if (tcs.begin()) {
    Serial.println("TCS34725 found");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (millis() < 5000) { // Nháy đèn trong 5 giây nếu không tìm thấy
      digitalWrite(W_LED_ON, !digitalRead(W_LED_ON));
      delay(100);
    }
  }
#endif
  
  digitalWrite(W_LED_ON, 0);
  analogWrite(W_LED_ON, 150); // Bật đèn trắng cho cảm biến màu
}

void loop()
{
  // Đọc cảm biến màu để phát hiện vạch đỏ (vạch dừng)
#ifdef TCS_SENSOR
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  // Nếu màu đỏ chiếm ưu thế rõ rệt
  if (r > (g * 1.5) && r > (b * 1.5) && c > 1000) { // Thêm điều kiện c > 1000 để tránh nhiễu
    stop();
    Serial.println("Red line detected! Stopping.");
    delay(5000); // Dừng 5 giây
    return; // Dừng vòng lặp tạm thời
  }
#endif

  // Đọc giá trị từ 4 cảm biến line
  int s1 = analogRead(SENSOR_1_PIN) > threshold; // 1 = vạch đen, 0 = nền trắng
  int s2 = analogRead(SENSOR_2_PIN) > threshold;
  int s3 = analogRead(SENSOR_3_PIN) > threshold;
  int s4 = analogRead(SENSOR_4_PIN) > threshold;

  // Tạo một số nhị phân 4-bit từ trạng thái cảm biến
  // Ví dụ: 0110 nghĩa là 2 cảm biến giữa đang ở trên vạch đen
  uint8_t sensor_array = (s1 << 3) | (s2 << 2) | (s3 << 1) | s4;

  Serial.print("Sensors: ");
  Serial.print(s1); Serial.print(s2); Serial.print(s3); Serial.print(s4);
  Serial.print(" ("); Serial.print(sensor_array, BIN); Serial.print(") ");

  switch (sensor_array)
  {
    // --- Đi thẳng ---
    case 0b0110: // Ổn định, đi thẳng
      forward();
      last_error = 0;
      break;

    // --- Rẽ nhẹ ---
    case 0b0100: // Lệch phải một chút, rẽ trái nhẹ
      left();
      last_error = -1;
      break;
    case 0b0010: // Lệch trái một chút, rẽ phải nhẹ
      right();
      last_error = 1;
      break;

    // --- Rẽ vừa ---
    case 0b1100: // Lệch phải, rẽ trái
      left1();
      last_error = -2;
      break;
    case 0b0011: // Lệch trái, rẽ phải
      right1();
      last_error = 2;
      break;

    // --- Rẽ gắt (cua vuông) ---
    case 0b1000: // Lệch phải nhiều, rẽ trái gắt
      left2();
      last_error = -3;
      break;
    case 0b0001: // Lệch trái nhiều, rẽ phải gắt
      right2();
      last_error = 3;
      break;

    // --- Xử lý các trường hợp phức tạp hơn ---
    case 0b0111: // Hơi lệch trái ở ngã ba/cua rộng
      right();
      last_error = 2;
      break;
    case 0b1110: // Hơi lệch phải ở ngã ba/cua rộng
      left();
      last_error = -2;
      break;
      
    case 0b1111: // Gặp ngã tư hoặc vạch ngang
      forward(); // Đi thẳng qua
      break;

    // --- XỬ LÝ MẤT LINE (QUAN TRỌNG NHẤT) ---
    case 0b0000:
      if (last_error <= -2) { // Lần cuối cùng thấy line là ở bên trái -> robot đang ở bên phải line
        left2(); // Rẽ trái gắt để tìm lại line
      } else if (last_error >= 2) { // Lần cuối cùng thấy line là ở bên phải -> robot đang ở bên trái line
        right2(); // Rẽ phải gắt để tìm lại line
      } else {
        // Nếu trước đó đang đi thẳng, có thể đã qua vạch đích
        // Hoặc có thể đi lùi để tìm lại, nhưng đơn giản nhất là dừng
        stop(); 
      }
      break;

    default:
      // Các trường hợp không xác định (ví dụ 1010), có thể dừng lại hoặc đi thẳng
      forward();
      break;
  }
}

// === CÁC HÀM ĐIỀU KHIỂN MOTOR ===
// Tốc độ được điều chỉnh lại để ổn định hơn
void stop() {
  analogWrite(PWM_PIN_L_A, 0);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("stop");
}

void forward() {
  analogWrite(PWM_PIN_L_A, baseSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, baseSpeed);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("forward");
}

// Rẽ nhẹ
void left() {
  analogWrite(PWM_PIN_L_A, baseSpeed * 0.7);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, baseSpeed);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("left");
}

void right() {
  analogWrite(PWM_PIN_L_A, baseSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, baseSpeed * 0.7);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("right");
}

// Rẽ vừa
void left1() {
  analogWrite(PWM_PIN_L_A, baseSpeed * 0.4);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, baseSpeed);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("left1");
}

void right1() {
  analogWrite(PWM_PIN_L_A, baseSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, baseSpeed * 0.4);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("right1");
}

// Rẽ gắt (xoay tại chỗ)
void left2() {
  analogWrite(PWM_PIN_L_A, 0);
  analogWrite(PWM_PIN_L_B, baseSpeed * 0.8); // Bánh trái quay lùi
  analogWrite(PWM_PIN_R_A, baseSpeed);
  analogWrite(PWM_PIN_R_B, 0);           // Bánh phải quay tiến
  Serial.println("left2 (sharp)");
}

void right2() {
  analogWrite(PWM_PIN_L_A, baseSpeed);
  analogWrite(PWM_PIN_L_B, 0);           // Bánh trái quay tiến
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, baseSpeed * 0.8); // Bánh phải quay lùi
  Serial.println("right2 (sharp)");
}
