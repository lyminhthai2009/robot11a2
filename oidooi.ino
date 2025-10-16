#include <QTRSensors.h>

// --- CÁC CHÂN ĐIỀU KHIỂN ĐỘNG CƠ ---
#define PWM_PIN_L_A 2  
#define PWM_PIN_L_B 10 
#define PWM_PIN_R_A 6  
#define PWM_PIN_R_B 5  

// --- CÁC CHÂN CẢM BIẾN ---
// Sắp xếp từ TRÁI qua PHẢI (Sensor 1 là ngoài cùng bên trái)
#define SENSOR_1_PIN 4 
#define SENSOR_2_PIN 3 
#define SENSOR_3_PIN 1 
#define SENSOR_4_PIN 0 

// ===================================================================
// === KHU VỰC TINH CHỈNH "PRO VIP" V4.0 - TỐC ĐỘ TỐI ĐA ===
// ===================================================================

// --- BƯỚC 1: CHỌN TỐC ĐỘ KHÔNG ĐỔI ---
// Robot sẽ luôn chạy ở tốc độ này. Bắt đầu với giá trị thấp (VD: 200) để tinh chỉnh PID,
// sau đó tăng dần lên đến mức tối đa mà robot còn có thể bám đường.
const int TOC_DO_CHAY = 255; 

// --- BƯỚC 2: TINH CHỈNH PD CHO TỐC ĐỘ CAO ---
// Với tốc độ không đổi, Kp và Kd là chìa khóa VÀNG để vào cua mượt.
const float Kp = 0.35;  // Tăng Kp để phản ứng bẻ lái gắt hơn.
const float Ki = 0;     // Luôn bằng 0 ở chế độ này.
const float Kd = 1.5;   // TĂNG RẤT MẠNH Kd. Đây là thông số quan trọng nhất để giữ robot ổn định ở tốc độ cao.

// --- BƯỚC 3: TINH CHỈNH CHO NGÃ TƯ ---
const int NGUONG_CAM_BIEN_DEN = 800; 
const int TOC_DO_RE = 200; 
const int THOI_GIAN_DI_THANG = 120;

// ===================================================================

QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

// Các biến cho thuật toán PID
uint16_t position;
float P, D, previousError = 0;
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

  if (sensorValues[0] > NGUONG_CAM_BIEN_DEN && sensorValues[1] > NGUONG_CAM_BIEN_DEN &&
      sensorValues[2] > NGUONG_CAM_BIEN_DEN && sensorValues[3] > NGUONG_CAM_BIEN_DEN) 
  {
    re_phai();
  } 
  else
  {
    float error = 1500.0 - position;
    PID_Linefollow(error);
  }
}

// *** HÀM PID V4.0 - TỐI GIẢN HÓA CHO TỐC ĐỘ KHÔNG ĐỔI ***
void PID_Linefollow(float error) {
  P = error;
  D = error - previousError;
  
  float PID_value = (Kp * P) + (Kd * D);
  
  previousError = error;

  // Tốc độ 2 bánh xe giờ được tính trực tiếp từ tốc độ chạy không đổi
  toc_do_trai = TOC_DO_CHAY - PID_value;
  toc_do_phai = TOC_DO_CHAY + PID_value;

  // Giới hạn tốc độ trong khoảng cho phép của động cơ
  toc_do_trai = constrain(toc_do_trai, -255, 255);
  toc_do_phai = constrain(toc_do_phai, -255, 255);

  motor_drive(toc_do_trai, toc_do_phai);
}

void re_phai() {
  motor_drive(TOC_DO_RE, TOC_DO_RE);
  delay(THOI_GIAN_DI_THANG);

  motor_drive(TOC_DO_RE, -TOC_DO_RE);
  
  do {
    qtr.readLineBlack(sensorValues);
  } while (sensorValues[1] < NGUONG_CAM_BIEN_DEN);
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
