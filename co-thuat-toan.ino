// V4.0 - Advanced PID with Sensor Filtering and Improved Error Calculation
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Cảm biến màu (Không thay đổi)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// --- Chân điều khiển (Kiểm tra lại cho đúng với robot của bạn) ---
#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

#define SENSOR_1_PIN 4 // Trái nhất
#define SENSOR_2_PIN 3
#define SENSOR_3_PIN 1
#define SENSOR_4_PIN 0 // Phải nhất

#define W_LED_ON 20
#define IR_LED_ON 21
#define TCS_SENSOR

// =================================================================
// <<< QUAN TRỌNG: THAY CÁC GIÁ TRỊ CỦA BẠN VÀO ĐÂY >>>
// Sử dụng code cũ có hiệu chỉnh 10 giây chạy 1 lần để lấy giá trị này
// =================================================================
unsigned int sensorMin[4] = {135, 128, 141, 138}; // Dán các giá trị Min (trên vạch đen) của bạn vào đây
unsigned int sensorMax[4] = {876, 882, 879, 891}; // Dán các giá trị Max (trên nền trắng) của bạn vào đây

// =================================================================
// --- BỘ THÔNG SỐ ĐỂ BẮT ĐẦU TINH CHỈNH ---
// =================================================================

// --- XỬ LÝ CẢM BIẾN ---
// Hệ số lọc cho cảm biến (0.0 - 1.0). Càng nhỏ càng mượt nhưng phản ứng chậm hơn.
float sensorFilterAlpha = 0.6;

// --- HỆ SỐ PID ---
float Kp = 2.35;
float Ki = 0.0001; // Luôn tinh chỉnh Ki sau cùng
float Kd = 18;

// --- TỐC ĐỘ ---
int maxSpeed = 255; // Tốc độ tối đa
int minSpeed = 100;  // Tốc độ tối thiểu khi vào cua gắt

// --- BỘ LỌC CHO DERIVATIVE (D) ---
float dFilterAlpha = 0.35;

// --- Các biến toàn cục ---
unsigned int rawSensorValues[4];
float filteredSensorValues[4] = {0, 0, 0, 0};
float error = 0;
float lastError = 0;
float P, I, D;
float filteredD = 0;

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

#ifdef TCS_SENSOR
  if (tcs.begin()) { Serial.println("TCS34725 found"); }
  else { Serial.println("No TCS34725 found"); }
#endif

  analogWrite(W_LED_ON, 150);
  
  Serial.println("Robot da san sang! V4.0 - Advanced Sensor Processing");
}

void loop() {
  // 1. Tính toán sai số vị trí từ các cảm biến đã được lọc
  error = calculateErrorPosition();

  // 2. Thuật toán PID
  P = error;
  I = I + error;
  float rawD = error - lastError;
  filteredD = (dFilterAlpha * rawD) + ((1.0 - dFilterAlpha) * filteredD);
  D = filteredD;
  lastError = error;
  
  // Giới hạn thành phần I để tránh tích lũy quá mức (Integral Windup)
  if (I > 3000) I = 3000;
  if (I < -3000) I = -3000;

  // 3. Tính toán hiệu chỉnh tốc độ
  float motorSpeedCorrection = (Kp * P) + (Ki * I) + (Kd * D);

  // 4. Tốc độ thích ứng
  int speedReduction = abs(motorSpeedCorrection) * 0.15; // Tinh chỉnh hệ số 0.15 nếu cần
  int currentSpeed = maxSpeed - speedReduction;
  currentSpeed = constrain(currentSpeed, minSpeed, maxSpeed);

  // 5. Điều khiển động cơ
  int leftSpeed = currentSpeed + motorSpeedCorrection;
  int rightSpeed = currentSpeed - motorSpeedCorrection;
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  setMotorSpeeds(leftSpeed, rightSpeed);
}


// === CÁC HÀM CHỨC NĂNG NÂNG CAO ===

void readAndFilterSensors() {
  // Đọc và map giá trị thô
  rawSensorValues[0] = map(analogRead(SENSOR_1_PIN), sensorMin[0], sensorMax[0], 1000, 0);
  rawSensorValues[1] = map(analogRead(SENSOR_2_PIN), sensorMin[1], sensorMax[1], 1000, 0);
  rawSensorValues[2] = map(analogRead(SENSOR_3_PIN), sensorMin[2], sensorMax[2], 1000, 0);
  rawSensorValues[3] = map(analogRead(SENSOR_4_PIN), sensorMin[3], sensorMax[3], 1000, 0);

  for(int i = 0; i < 4; i++){
    rawSensorValues[i] = constrain(rawSensorValues[i], 0, 1000);

    // Áp dụng bộ lọc thông thấp (Exponential Filter) cho từng cảm biến
    filteredSensorValues[i] = (sensorFilterAlpha * rawSensorValues[i]) + ((1.0 - sensorFilterAlpha) * filteredSensorValues[i]);
  }
}

float calculateErrorPosition() {
  readAndFilterSensors();

  float avg = 0;
  float sum = 0;
  
  // Sử dụng hệ trọng số đối xứng, tâm là 0
  avg = (float)(filteredSensorValues[0] * -3000) + (filteredSensorValues[1] * -1000) + 
        (filteredSensorValues[2] * 1000)  + (filteredSensorValues[3] * 3000);
        
  sum = filteredSensorValues[0] + filteredSensorValues[1] + filteredSensorValues[2] + filteredSensorValues[3];

  // Xử lý mất line
  if (sum < 50) { // Dùng ngưỡng nhỏ để an toàn hơn
    if (lastError > 1000) return 4000;      // Nếu lần cuối lệch trái nhiều -> rẽ phải cực mạnh
    else if (lastError < -1000) return -4000; // Nếu lần cuối lệch phải nhiều -> rẽ trái cực mạnh
    return lastError;
  }
  
  // Xử lý trường hợp đặc biệt ở cua gắt
  bool on_far_left = filteredSensorValues[0] > 800 && filteredSensorValues[1] < 200;
  if (on_far_left) return -4000;

  bool on_far_right = filteredSensorValues[3] > 800 && filteredSensorValues[2] < 200;
  if (on_far_right) return 4000;

  float position = avg / sum;
  return position;
}


// --- Các hàm điều khiển động cơ (Không thay đổi) ---
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    analogWrite(PWM_PIN_L_A, leftSpeed);
    analogWrite(PWM_PIN_L_B, 0);
  } else {
    analogWrite(PWM_PIN_L_A, 0);
    analogWrite(PWM_PIN_L_B, -leftSpeed);
  }

  if (rightSpeed > 0) {
    analogWrite(PWM_PIN_R_A, rightSpeed);
    analogWrite(PWM_PIN_R_B, 0);
  } else {
    analogWrite(PWM_PIN_R_A, 0);
    analogWrite(PWM_PIN_R_B, -rightSpeed);
  }
}

void stop() {
  setMotorSpeeds(0, 0);
}
