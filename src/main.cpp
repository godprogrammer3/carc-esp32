#include <Arduino.h>
#include <ESP32Encoder.h>

// Encoder define
#define ENCODER_A_PIN 36
#define ENCODER_B_PIN 39
#define ENCODER2_A_PIN 34
#define ENCODER2_B_PIN 35
#define ENCODER3_A_PIN 32
#define ENCODER3_B_PIN 33
#define ENCODER4_A_PIN 25
#define ENCODER4_B_PIN 26

// Motor define
#define MOTOR_PWM_FREQ 1000
#define MOTOR_PWM_RESOLUTION 10
#define MOTOR_A_PIN 27
#define MOTOR_B_PIN 14
#define MOTOR_PWM_PIN 12
#define MOTOR2_A_PIN 9
#define MOTOR2_B_PIN 10
#define MOTOR2_PWM_PIN 13
enum MotorDirection
{
  clockwise,
  counterClockwise,
  stop,
  forceStop,
};
const int8_t motorPwmPins[] = {MOTOR_PWM_PIN, MOTOR2_PWM_PIN};
const int8_t motorAPins[] = {MOTOR_A_PIN, MOTOR2_A_PIN};
const int8_t motorBPins[] = {MOTOR_B_PIN, MOTOR2_B_PIN};

void motor(int channel, MotorDirection direction, float speed);

ESP32Encoder encoder;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

void setup()
{

  Serial.begin(115200);

  // Encoder
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachHalfQuad(ENCODER_A_PIN, ENCODER_B_PIN);
  encoder2.attachHalfQuad(ENCODER2_A_PIN, ENCODER2_B_PIN);
  encoder3.attachHalfQuad(ENCODER3_A_PIN, ENCODER3_B_PIN);
  encoder4.attachHalfQuad(ENCODER4_A_PIN, ENCODER4_B_PIN);

  // Motor
  for (int i = 0; i < 2; i++)
  {
    pinMode(motorPwmPins[i], OUTPUT);
    pinMode(motorAPins[i], OUTPUT);
    pinMode(motorBPins[i], OUTPUT);
    ledcSetup(i, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(motorPwmPins[i], i);
    motor(i, stop, 0);
  }
}

void loop()
{
  if (Serial.available())
  {
    String str = Serial.readString();
    Serial.printf("Input: %s\n", str.c_str());
    int values[3];
    int startIndex = 0;
    int endIndex = str.indexOf(",");
    values[0] = str.substring(startIndex, endIndex).toInt();

    startIndex = endIndex + 1;
    endIndex = str.indexOf(",", startIndex);

    values[1] = str.substring(startIndex, endIndex).toInt();

    startIndex = endIndex + 1;

    values[2] = str.substring(startIndex).toInt();

    motor(values[0], (MotorDirection)values[1], values[2] / 100.0);
  }

  // Serial.printf("Encoder count:");
  // Serial.printf(" %lld,", encoder.getCount());
  // Serial.printf(" %lld,", encoder2.getCount());
  // Serial.printf(" %lld,", encoder3.getCount());
  // Serial.printf(" %lld,", encoder4.getCount());
  // Serial.println();
  delay(1);
}

void motor(int channel, MotorDirection direction, float speed)
{
  if (direction == clockwise)
  {
    digitalWrite(motorAPins[channel], HIGH);
    digitalWrite(motorBPins[channel], LOW);
  }
  else if (direction == counterClockwise)
  {
    digitalWrite(motorAPins[channel], LOW);
    digitalWrite(motorBPins[channel], HIGH);
  }
  else if (direction == forceStop)
  {
    digitalWrite(motorAPins[channel], HIGH);
    digitalWrite(motorBPins[channel], HIGH);
  }
  else
  {
    digitalWrite(motorAPins[channel], LOW);
    digitalWrite(motorBPins[channel], LOW);
  }
  ledcWrite(channel, (int)(speed * 1024));
}