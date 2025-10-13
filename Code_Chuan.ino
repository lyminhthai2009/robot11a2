// V1.1

#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

#define left_motor_channel_a 0
#define left_motor_channel_b 1
#define right_motor_channel_a 2
#define right_motor_channel_b 3

#define SENSOR_1_PIN 4
#define SENSOR_2_PIN 3
#define SENSOR_3_PIN 1
#define SENSOR_4_PIN 0

#define W_LED_ON 20
#define IR_LED_ON 21

#define threshold 3000

#define TCS_SENSOR

float x = 1; // Kspeed
int mode;

int maxSpeed = 200;

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
  forward();
  delay(2000);
  stop();
  delay(2000);
  // analogWrite(W_LED_ON, 150);
  // delay(2000);
  // analogWrite(W_LED_ON, 20);
  // delay(2000);
  analogWrite(W_LED_ON, 150);
  // delay(5000);
}

void loop()
{

  uint16_t r, g, b, c;
#ifdef TCS_SENSOR
  tcs.getRawData(&r, &g, &b, &c);
#endif
  Serial.print("tcs: R = ");
  Serial.print(r);
  Serial.print("  G = ");
  Serial.print(g);
  Serial.print("  B = ");
  Serial.print(b);
  Serial.print("  C = ");
  Serial.print(c);
  Serial.print("  ");
  Serial.print("mode = ");
  Serial.print(x);
  Serial.print("  ");

  int sensor_1_state = analogRead(SENSOR_1_PIN);
  int sensor_2_state = analogRead(SENSOR_2_PIN);
  int sensor_3_state = analogRead(SENSOR_3_PIN);
  int sensor_4_state = analogRead(SENSOR_4_PIN);

  Serial.print("ir = \t");
  Serial.print(sensor_1_state);
  Serial.print("\t");
  Serial.print(sensor_2_state);
  Serial.print("\t");
  Serial.print(sensor_3_state);
  Serial.print("\t");
  Serial.print(sensor_4_state);

  uint8_t sensor_array = 0;

  // // sử dụng cho phiên bản mới
  // sensor_array += (sensor_4_state <= threshold);
  // sensor_array <<= 1;
  // sensor_array += (sensor_3_state <= threshold);
  // sensor_array <<= 1;
  // sensor_array += (sensor_2_state <= threshold);
  // sensor_array <<= 1;
  // sensor_array += (sensor_1_state <= threshold);

  // sử dụng cho phiên bản cũ
  sensor_array += (sensor_1_state <= threshold);
  sensor_array <<= 1;
  sensor_array += (sensor_2_state <= threshold);
  sensor_array <<= 1;
  sensor_array += (sensor_3_state <= threshold);
  sensor_array <<= 1;
  sensor_array += (sensor_4_state <= threshold);

  Serial.print("    ");
  Serial.println(sensor_array | 0x80, BIN);
#ifdef TCS_SENSOR
  if (r > (g * 1.6) && r > (b * 1.6))
  {
    stop();
    //x=0;
    return;
  }
#endif
  switch (sensor_array)
  {
  case 0b1000:
    left2();
    break;
  case 0b1100:
    left1();
    break;

  case 0b0001:
    right2();
    break;
  case 0b0011:
    right1();
    break;

  case 0b0110:
    forward();
    break;
  case 0b0100:
    left();
    break;
  case 0b0010:
    right();
    break;
  case 0b1110:
    left();
    break;
  case 0b0111:
    right();
    break;
  case 0b1111:
    // right2();
    // delay(200);
    // stop();
    break;

  case 0b0000:
    // forward();
    // delay(700);
    // stop();
    break;

    // default:
    //   forward();
    //   break;
  }

  // delay(2);
}

void stop()
{
  analogWrite(PWM_PIN_L_A, 0);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("stop");
}

void forward()
{
  analogWrite(PWM_PIN_L_A, 130 * x);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 130 * x);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("forward");
}

void backward()
{
  analogWrite(PWM_PIN_L_A, 0);
  analogWrite(PWM_PIN_L_B, 100 * x);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, 100 * x);
  Serial.println("backward");
}

void left()
{
  analogWrite(PWM_PIN_L_A, 100 * x);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 130 * x);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("left");
}

void right()
{
  analogWrite(PWM_PIN_L_A, 130 * x);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 100 * x);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("right");
}
void left1()
{
  analogWrite(PWM_PIN_L_A, 60 * x);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 100 * x);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("left1");
}

void right1()
{
  analogWrite(PWM_PIN_L_A, 100 * x);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 60 * x);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("right1");
}
void left2()
{
  analogWrite(PWM_PIN_L_A, 0);
  analogWrite(PWM_PIN_L_B, 50 * x);
  analogWrite(PWM_PIN_R_A, 100 * x);
  analogWrite(PWM_PIN_R_B, 0);
  Serial.println("left2");
}

void right2()
{
  analogWrite(PWM_PIN_L_A, 100 * x);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, 50 * x);
  Serial.println("right2");
}