#include <QTRSensors.h>

// --- CÁC CHÂN ĐIỀU KHIỂN ĐỘNG CƠ ---
#define PWM_PIN_L_A 2  
#define PWM_PIN_L_B 10 
#define PWM_PIN_R_A 6  
#define PWM_PIN_R_B 5  

// --- CÁC CHÂN CẢM BIẾN ---
#define SENSOR_1_PIN 4 
#define SENSOR_2_PIN 3 
#define SENSOR_3_PIN 1 
#define SENSOR_4_PIN 0 

// ===================================================================
// === KHU VỰC TINH CHỈNH "PRO VIP" CHO CUỘC THI TỐC ĐỘ ===
// ===================================================================

// --- BƯỚC 1: TINH CHỈNH TỐC ĐỘ ---
const int TOC_DO_CO_BAN = 220; // Tốc độ khi vào cua. Giữ ở mức vừa phải để cua an toàn.
const int TOC_DO_MAX = 255;    // Tốc độ tối đa trên đường thẳng. ĐÂY LÀ TỐC ĐỘ BẠN MUỐN ĐẠT!

// --- BƯỚC 2: TINH CHỈNH LẠI PID CHO TỐC ĐỘ CAO ---
const float Kp = 0.25;   
const float Ki = 0.001; 
const float Kd = 0.8;   

// --- BƯỚC 3: NGƯỠNG PHÁT HIỆN ĐƯỜNG CONG ---
const int NGUONG_VAO_CUA = 200; 

// --- BƯỚC 4: TINH CHỈNH XỬ LÝ NGÃ TƯ --- // <<< MỚI
const int NGUONG_LINE_DEN = 800;  // Giá trị cảm biến đọc được để xác nhận là vạch đen. (Thường > 700)
const int TOC_DO_RE_NGA_TU = 180; // Tốc độ khi thực hiện rẽ tại ngã tư.
const int DELAY_QUA_NGA_TU = 100; // Thời gian chạy thẳng qua tâm ngã tư (ms). Tinh chỉnh cho robot của bạn.

// Giới hạn cho thành phần I để chống "Integral Windup"
const int I_MAX = 400;
const int I_MIN = -400;

// ===================================================================

QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

// Các biến cho thuật toán PID
uint16_t position;
float P, D, I = 0, previousError = 0;
int toc_do_trai, toc_do_phai;

void setup() {
  Serial.begin(115200);
  
  pinMode(PWM_PIN_L_A, OUTPUT);
  pinMode(PWM_PIN_L_B, OUTPUT);
  pinMode(PWM_PIN_R_A, OUTPUT);
  pinMode(PWM_PIN_R_B, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN}, SensorCount);
  delay(500);
  
  Serial.println("Bat dau Calibrate trong 5 giay...");
  digitalWrite(LED_BUILTIN, LOW);
  
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Calibrate hoan thanh! Robot se bat dau chay sau 2 giay.");
  delay(2000);
}

void loop() {
  robot_control();
}

// <<< HÀM NÀY ĐÃ ĐƯỢC THAY ĐỔI ĐỂ XỬ LÝ NGÃ TƯ >>>
void robot_control() {
  position = qtr.readLineBlack(sensorValues);
  
  // --- LOGIC PHÁT HIỆN VÀ XỬ LÝ NGÃ TƯ (ƯU TIÊN RẼ PHẢI) ---
  // Điều kiện: Cả 4 cảm biến đều nhìn thấy vạch đen
  if (sensorValues[0] > NGUONG_LINE_DEN && sensorValues[1] > NGUONG_LINE_DEN && 
      sensorValues[2] > NGUONG_LINE_DEN && sensorValues[3] > NGUONG_LINE_DEN) 
  {
    Serial.println("--> Nga tu! Uu tien re PHAI.");
    
    // BƯỚC 1: Chạy thẳng một đoạn ngắn để xe vào đúng tâm của ngã tư
    motor_drive(TOC_DO_RE_NGA_TU, TOC_DO_RE_NGA_TU);
    delay(DELAY_QUA_NGA_TU); // Rất quan trọng, cần tinh chỉnh

    // BƯỚC 2: Thực hiện rẽ phải tại chỗ (bánh trái tiến, bánh phải lùi)
    // Tiếp tục rẽ cho đến khi các cảm biến giữa tìm thấy lại vạch đen
    motor_drive(TOC_DO_RE_NGA_TU, -TOC_DO_RE_NGA_TU);
    
    // Đợi cho đến khi một trong hai cảm biến giữa (1 hoặc 2) nhìn thấy line mới
    do {
      qtr.readLineBlack(sensorValues);
    } while (sensorValues[1] < NGUONG_LINE_DEN && sensorValues[2] < NGUONG_LINE_DEN);

    // BƯỚC 3: Đặt lại các giá trị PID để tránh bị "giật" sau khi rẽ
    I = 0;
    previousError = 0;
    
    // Sau khi xử lý ngã tư, hàm sẽ kết thúc và vòng lặp tiếp theo sẽ chạy PID bình thường
  } 
  else 
  {
    // Nếu không phải ngã tư, chạy PID dò line như bình thường
    float error = 1500.0 - position;
    PID_Linefollow(error);
  }
}

void PID_Linefollow(float error) {
  P = error;
  I = I + error;
  
  if (I > I_MAX) I = I_MAX;
  if (I < I_MIN) I = I_MIN;
  
  D = error - previousError;
  
  float PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  
  previousError = error;

  int toc_do_hien_tai;
  if (abs(error) < NGUONG_VAO_CUA) {
    toc_do_hien_tai = TOC_DO_MAX;
  } else {
    toc_do_hien_tai = TOC_DO_CO_BAN;
  }

  toc_do_trai = toc_do_hien_tai - PID_value;
  toc_do_phai = toc_do_hien_tai + PID_value;

  toc_do_trai = constrain(toc_do_trai, -255, 255);
  toc_do_phai = constrain(toc_do_phai, -255, 255);

  motor_drive(toc_do_trai, toc_do_phai);
}

void motor_drive(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    analogWrite(PWM_PIN_L_A, leftSpeed);
    digitalWrite(PWM_PIN_L_B, LOW);
  } else {
    digitalWrite(PWM_PIN_L_A, LOW);
    analogWrite(PWM_PIN_L_B, -leftSpeed);
  }
  
  if (rightSpeed >= 0) {
    analogWrite(PWM_PIN_R_A, rightSpeed);
    digitalWrite(PWM_PIN_R_B, LOW);
  } else {
    digitalWrite(PWM_PIN_R_A, LOW);
    analogWrite(PWM_PIN_R_B, -rightSpeed);
  }
}

void motor_stop() {
  digitalWrite(PWM_PIN_L_A, LOW);
  digitalWrite(PWM_PIN_L_B, LOW);
  digitalWrite(PWM_PIN_R_A, LOW);
  digitalWrite(PWM_PIN_R_B, LOW);
}
