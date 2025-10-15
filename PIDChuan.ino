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
// Bắt đầu với các giá trị gợi ý này và tinh chỉnh dần dần.
const float Kp = 0.25;   // Tăng Kp để robot phản ứng nhanh hơn, bẻ lái gắt hơn.
const float Ki = 0.001; // Giữ Ki rất nhỏ hoặc bằng 0 để tránh tích lũy sai số lớn ở tốc độ cao.
const float Kd = 0.8;   // TĂNG MẠNH Kd. Đây là chìa khóa để giữ robot ổn định, chống rung lắc.

// --- BƯỚC 3: NGƯỠNG PHÁT HIỆN ĐƯỜNG CONG ---
// Nếu sai số (error) vượt quá ngưỡng này, robot sẽ hiểu là đang vào cua và giảm tốc.
const int NGUONG_VAO_CUA = 200; 

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

void robot_control() {
  position = qtr.readLineBlack(sensorValues);
  float error = 1500.0 - position;
  PID_Linefollow(error);
}

// *** HÀM PID ĐÃ ĐƯỢC NÂNG CẤP LÊN PHIÊN BẢN "PRO VIP" ***
void PID_Linefollow(float error) {
  P = error;
  I = I + error;
  
  // Chống "Integral Windup"
  if (I > I_MAX) I = I_MAX;
  if (I < I_MIN) I = I_MIN;
  
  D = error - previousError;
  
  float PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  
  previousError = error;

  // *** LOGIC ĐIỀU KHIỂN TỐC ĐỘ ĐỘNG ***
  int toc_do_hien_tai;
  if (abs(error) < NGUONG_VAO_CUA) {
    // Sai số nhỏ -> Đang trên đường thẳng -> BUNG TỐC ĐỘ!
    toc_do_hien_tai = TOC_DO_MAX;
  } else {
    // Sai số lớn -> Đang vào cua -> Giảm tốc để bám đường
    toc_do_hien_tai = TOC_DO_CO_BAN;
  }

  // Điều chỉnh tốc độ 2 bánh xe dựa trên giá trị PID và tốc độ hiện tại
  toc_do_trai = toc_do_hien_tai - PID_value;
  toc_do_phai = toc_do_hien_tai + PID_value;

  // Giới hạn tốc độ trong khoảng cho phép
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
