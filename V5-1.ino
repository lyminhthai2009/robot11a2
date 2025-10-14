// V5.2 - ĐÃ SỬA LỖI LOGIC CẢM BIẾN VÀ TINH CHỈNH
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Cảm biến màu
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// --- Chân điều khiển ---
#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

#define SENSOR_1_PIN A4 // Trái nhất
#define SENSOR_2_PIN A3
#define SENSOR_3_PIN A1
#define SENSOR_4_PIN A0 // Phải nhất

#define W_LED_ON 20
#define IR_LED_ON 21
#define ONBOARD_LED 13 // LED có sẵn trên board để báo hiệu

// =================================================================
// <<< CÁC THÔNG SỐ CẤU HÌNH VÀ TINH CHỈNH >>>
// =================================================================

// --- XỬ LÝ CẢM BIẾN ---
const float SENSOR_FILTER_ALPHA = 0.6;
const int LINE_LOST_THRESHOLD = 50;
const int INTERSECTION_THRESHOLD = 900;
const int FAR_SENSOR_THRESHOLD = 800;
const int NEAR_SENSOR_THRESHOLD = 200;

// --- HỆ SỐ PID (Đã được tinh chỉnh lại cho lần chạy đầu) ---
const float Kp = 1.8;     // Giảm Kp một chút để bớt "gắt"
const float Ki = 0.0001;  // Giữ nguyên Ki
const float Kd = 12;      // Giảm Kd đáng kể để bớt rung và ổn định hơn

// --- TỐC ĐỘ (Gợi ý giảm tốc độ để dễ tinh chỉnh ban đầu) ---
const int MAX_SPEED = 200; // Giảm tốc độ tối đa để PID dễ xử lý hơn
const int MIN_SPEED = 90;
const float SPEED_REDUCTION_FACTOR = 0.15;

// --- BỘ LỌC CHO DERIVATIVE (D) ---
const float D_FILTER_ALPHA = 0.35;

// --- GIỚI HẠN CHO INTEGRAL (I) ---
const int I_LIMIT = 3000;

// --- CÁC HẰNG SỐ SAI SỐ (ERROR) ---
const int WEIGHT_NEAR = 1000;
const int WEIGHT_FAR = 3000;
const int MAX_ERROR = 4000;

// =================================================================
// --- CÁC BIẾN TOÀN CỤC ---
// =================================================================
unsigned int sensorMin[4] = {1023, 1023, 1023, 1023};
unsigned int sensorMax[4] = {0, 0, 0, 0};

unsigned int rawSensorValues[4];
float filteredSensorValues[4] = {0, 0, 0, 0};
float error = 0;
float lastError = 0;
float P, I, D;
float filteredD = 0;

// =================================================================
// <<< HÀM SETUP VỚI QUY TRÌNH HIỆU CHỈNH ĐƯỢC CẢI TIẾN >>>
// =================================================================
void setup() {
  Serial.begin(115200);

  // --- Cài đặt pin ---
  pinMode(PWM_PIN_L_A, OUTPUT);
  pinMode(PWM_PIN_L_B, OUTPUT);
  pinMode(PWM_PIN_R_A, OUTPUT);
  pinMode(PWM_PIN_R_B, OUTPUT);
  pinMode(W_LED_ON, OUTPUT);
  pinMode(IR_LED_ON, OUTPUT);
  digitalWrite(W_LED_ON, 0);
  digitalWrite(IR_LED_ON, 1);
  pinMode(ONBOARD_LED, OUTPUT);

  if (tcs.begin()) { Serial.println("TCS34725 found"); }
  else { Serial.println("No TCS34725 found"); }

  analogWrite(W_LED_ON, 150);
  
  // --- SỬA LỖI LOGIC: QUÁ TRÌNH TỰ ĐỘNG HIỆU CHỈNH ĐÁNG TIN CẬY HƠN ---
  Serial.println("=============================================");
  Serial.println("BAT DAU KHOI DONG");
  Serial.println("Buoc 1: Dat robot len NEN TRANG.");
  
  // Chờ cho đến khi người dùng đặt robot lên nền trắng (giá trị analog thô sẽ thấp)
  // Ngưỡng 400 là an toàn cho nền trắng
  while(analogRead(SENSOR_1_PIN) > 400) {
    digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED)); // Nhấp nháy LED báo đang chờ
    delay(100);
  }

  Serial.println("OK! Da nhan dien nen trang.");
  Serial.println("Buoc 2: Dua cam bien trai nhat (So 1) vao VACH DEN de bat dau hieu chinh...");

  // Bây giờ, chờ cho đến khi cảm biến số 1 đọc được giá trị cao của vạch đen
  // Giá trị analog thô trên vạch đen thường cao. Ngưỡng 700 là an toàn.
  while(analogRead(SENSOR_1_PIN) < 700) {
    // Nhấp nháy LED nhanh hơn để báo hiệu đang chờ vạch đen
    digitalWrite(ONBOARD_LED, HIGH);
    delay(50);
    digitalWrite(ONBOARD_LED, LOW);
    delay(50);
  }
  
  // Bắt đầu quá trình hiệu chỉnh
  calibrateSensors();
  
  Serial.println("Robot da san sang! V5.2 - Ready to Race");
}


void loop() {
  // 0. (Ưu tiên cao nhất) Kiểm tra vạch màu đặc biệt
  checkSpecialLine();

  // 1. Tính toán sai số vị trí từ các cảm biến đã được lọc
  error = calculateErrorPosition();

  // 2. Thuật toán PID
  P = error;
  I = I + error;
  float rawD = error - lastError;
  filteredD = (D_FILTER_ALPHA * rawD) + ((1.0 - D_FILTER_ALPHA) * filteredD);
  D = filteredD;
  lastError = error;
  
  I = constrain(I, -I_LIMIT, I_LIMIT);

  // 3. Tính toán hiệu chỉnh tốc độ
  float motorSpeedCorrection = (Kp * P) + (Ki * I) + (Kd * D);

  // 4. Tốc độ thích ứng
  int speedReduction = abs(motorSpeedCorrection) * SPEED_REDUCTION_FACTOR;
  int currentSpeed = MAX_SPEED - speedReduction;
  currentSpeed = constrain(currentSpeed, MIN_SPEED, MAX_SPEED);

  // 5. Điều khiển động cơ
  int leftSpeed = currentSpeed + motorSpeedCorrection;
  int rightSpeed = currentSpeed - motorSpeedCorrection;
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  setMotorSpeeds(leftSpeed, rightSpeed);
}


// === CÁC HÀM CHỨC NĂNG (HÀM QUAN TRỌNG ĐÃ ĐƯỢC SỬA) ===

void calibrateSensors() {
  digitalWrite(ONBOARD_LED, HIGH); // Bật LED sáng liên tục trong quá trình hiệu chỉnh
  Serial.println("Bat dau hieu chinh trong 10 giay. Lia robot qua vach den va nen trang nhieu lan!");
  
  long startTime = millis();
  while(millis() - startTime < 10000) {
    unsigned int readings[4];
    readings[0] = analogRead(SENSOR_1_PIN);
    readings[1] = analogRead(SENSOR_2_PIN);
    readings[2] = analogRead(SENSOR_3_PIN);
    readings[3] = analogRead(SENSOR_4_PIN);

    for (int i = 0; i < 4; i++) {
      if (readings[i] > sensorMax[i]) sensorMax[i] = readings[i];
      if (readings[i] < sensorMin[i]) sensorMin[i] = readings[i];
    }
  }

  digitalWrite(ONBOARD_LED, LOW);
  Serial.println("Hieu chinh hoan tat!");
  Serial.println("Gia tri Min (Nen trang): ");
  Serial.print(sensorMin[0]); Serial.print(" "); Serial.print(sensorMin[1]); Serial.print(" ");
  Serial.print(sensorMin[2]); Serial.print(" "); Serial.println(sensorMin[3]);
  Serial.println("Gia tri Max (Vach den): ");
  Serial.print(sensorMax[0]); Serial.print(" "); Serial.print(sensorMax[1]); Serial.print(" ");
  Serial.print(sensorMax[2]); Serial.print(" "); Serial.println(sensorMax[3]);
  delay(3000); // Chờ 3 giây để bạn có thời gian đặt robot vào vạch xuất phát
}

void readAndFilterSensors() {
  // SỬA LỖI LOGIC QUAN TRỌNG NHẤT: Đảo ngược hàm map
  // Logic đúng: Nền trắng (giá trị analog thấp ~ sensorMin) -> map thành 0
  //             Vạch đen (giá trị analog cao ~ sensorMax) -> map thành 1000
  rawSensorValues[0] = map(analogRead(SENSOR_1_PIN), sensorMin[0], sensorMax[0], 0, 1000);
  rawSensorValues[1] = map(analogRead(SENSOR_2_PIN), sensorMin[1], sensorMax[1], 0, 1000);
  rawSensorValues[2] = map(analogRead(SENSOR_3_PIN), sensorMin[2], sensorMax[2], 0, 1000);
  rawSensorValues[3] = map(analogRead(SENSOR_4_PIN), sensorMin[3], sensorMax[3], 0, 1000);

  for(int i = 0; i < 4; i++){
    rawSensorValues[i] = constrain(rawSensorValues[i], 0, 1000); // Đảm bảo giá trị luôn trong khoảng 0-1000
    // Áp dụng bộ lọc thông thấp để làm mượt giá trị cảm biến
    filteredSensorValues[i] = (SENSOR_FILTER_ALPHA * rawSensorValues[i]) + ((1.0 - SENSOR_FILTER_ALPHA) * filteredSensorValues[i]);
  }
}

float calculateErrorPosition() {
  readAndFilterSensors();

  // Kiểm tra nếu cả 4 cảm biến đều trên vạch đen -> ngã rẽ
  bool is_intersection = filteredSensorValues[0] > INTERSECTION_THRESHOLD &&
                         filteredSensorValues[1] > INTERSECTION_THRESHOLD &&
                         filteredSensorValues[2] > INTERSECTION_THRESHOLD &&
                         filteredSensorValues[3] > INTERSECTION_THRESHOLD;
  if (is_intersection) {
    handleIntersection();
    return 0; // Khi ở ngã rẽ, không tính sai số
  }

  // Tính toán vị trí của line dựa trên trung bình có trọng số
  // Cảm biến bên trái cho sai số âm, bên phải cho sai số dương
  float avg = (float)(filteredSensorValues[0] * -WEIGHT_FAR) + (filteredSensorValues[1] * -WEIGHT_NEAR) + 
              (filteredSensorValues[2] * WEIGHT_NEAR)  + (filteredSensorValues[3] * WEIGHT_FAR);
  float sum = filteredSensorValues[0] + filteredSensorValues[1] + filteredSensorValues[2] + filteredSensorValues[3];

  // Xử lý trường hợp mất line
  if (sum < LINE_LOST_THRESHOLD) { 
    if (lastError > WEIGHT_NEAR) return MAX_ERROR;       // Nếu lần cuối cùng lệch phải, tiếp tục rẽ phải mạnh để tìm line
    else if (lastError < -WEIGHT_NEAR) return -MAX_ERROR; // Nếu lần cuối cùng lệch trái, tiếp tục rẽ trái mạnh
    return lastError; // Nếu đang ở giữa, giữ nguyên sai số cũ
  }
  
  // Xử lý trường hợp cua gắt (chỉ có cảm biến ngoài cùng thấy line)
  bool on_far_left = filteredSensorValues[0] > FAR_SENSOR_THRESHOLD && filteredSensorValues[1] < NEAR_SENSOR_THRESHOLD;
  if (on_far_left) return -MAX_ERROR;

  bool on_far_right = filteredSensorValues[3] > FAR_SENSOR_THRESHOLD && filteredSensorValues[2] < NEAR_SENSOR_THRESHOLD;
  if (on_far_right) return MAX_ERROR;

  return avg / sum;
}

void handleIntersection() {
  Serial.println("PHAT HIEN NGA RE!");
  setMotorSpeeds(150, 150); // Đi thẳng qua ngã rẽ
  delay(500); // Thời gian đi thẳng, cần điều chỉnh cho phù hợp với sân
}

void checkSpecialLine() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Điều kiện nhận biết màu đỏ (giá trị Red cao và trội hơn Green, Blue)
  if (r > 1000 && r > g * 1.5 && r > b * 1.5) { 
      Serial.println("VACH DO! DUNG LAI!");
      stop();
      delay(10000); // Dừng 10 giây
  }
}

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
