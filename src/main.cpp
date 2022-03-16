#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Adafruit_MCP23X17.h>

// Encoder define
#define ENCODER_A_PIN 34
#define ENCODER_B_PIN 35
#define ENCODER2_A_PIN 32
#define ENCODER2_B_PIN 33
#define ENCODER3_A_PIN 25
#define ENCODER3_B_PIN 26
#define ENCODER4_A_PIN 27
#define ENCODER4_B_PIN 14
#define ENCODER5_A_PIN 36
#define ENCODER5_B_PIN 39

// Motor define
#define MOTOR_PWM_FREQ 1000
#define MOTOR_PWM_RESOLUTION 10
#define MOTOR_A_PIN 1
#define MOTOR_B_PIN 0
#define MOTOR_PWM_PIN 19
#define MOTOR2_A_PIN 3
#define MOTOR2_B_PIN 2
#define MOTOR2_PWM_PIN 18
#define MOTOR3_A_PIN 5
#define MOTOR3_B_PIN 4
#define MOTOR3_PWM_PIN 5
#define MOTOR4_A_PIN 7
#define MOTOR4_B_PIN 6
#define MOTOR4_PWM_PIN 17
#define MOTOR5_A_PIN 8
#define MOTOR5_B_PIN 9
#define MOTOR5_PWM_PIN 23
#define MOTOR_PULSE_COUNTDOWN_TOLERENT 10
#define MOTOR_PULSE_COUNTDOWN_TIMEOUT 60

// Servo
#define SERVO_PWM_FREQ 50
#define SERVO_PWM_RESOLUTION 8
#define SERVO_PWM_PIN 16
#define SERVO2_PWM_PIN 4
#define SERVO3_PWM_PIN 0
#define SERVO_MIN 6
#define SERVO_MAX 20
#define SERVO2_MIN 6
#define SERVO2_MAX 41
#define SERVO3_MIN 20
#define SERVO3_MAX 25

enum MotorDirection
{
  Clockwise,
  CounterClockwise,
  Stop,
  ForceStop,
};
enum MotorChannel
{
  Front,
  Back,
  Left,
  Right,
  Grabber,
};
enum ServoChannel
{
  Lift,
  Rail,
  Barrier,
};
enum MinMax
{
  Min,
  Max,
};
const int8_t motorPwmPins[] = {MOTOR_PWM_PIN, MOTOR2_PWM_PIN, MOTOR3_PWM_PIN, MOTOR4_PWM_PIN, MOTOR5_PWM_PIN};
const int8_t motorAPins[] = {MOTOR_A_PIN, MOTOR2_A_PIN, MOTOR3_A_PIN, MOTOR4_A_PIN, MOTOR5_A_PIN};
const int8_t motorBPins[] = {MOTOR_B_PIN, MOTOR2_B_PIN, MOTOR3_B_PIN, MOTOR4_B_PIN, MOTOR5_B_PIN};
const int8_t servoPwmPins[] = {SERVO_PWM_PIN, SERVO2_PWM_PIN, SERVO3_PWM_PIN};

void motor(MotorChannel channel, MotorDirection direction, float speed);
void servo(ServoChannel channel, int pulseWidth);
int64_t getCurrentEncoder(MotorChannel channel);
void motorEncoderCountdown(MotorChannel channel, MotorDirection direction, float speed, int encoderTotal);

ESP32Encoder encoder;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;
ESP32Encoder encoder5;

Adafruit_MCP23X17 mcp;

void setup()
{

  Serial.begin(115200);

  // Encoder
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachHalfQuad(ENCODER_A_PIN, ENCODER_B_PIN);
  encoder2.attachHalfQuad(ENCODER2_A_PIN, ENCODER2_B_PIN);
  encoder3.attachHalfQuad(ENCODER3_A_PIN, ENCODER3_B_PIN);
  encoder4.attachHalfQuad(ENCODER4_A_PIN, ENCODER4_B_PIN);
  encoder5.attachHalfQuad(ENCODER5_A_PIN, ENCODER5_B_PIN);

  // IO Extender
  if (!mcp.begin_I2C())
  {
    // Serial.println("IO Extender Error");
    ESP.restart();
  }

  // Motor
  for (int i = 0; i < 5; i++)
  {
    pinMode(motorPwmPins[i], OUTPUT);
    mcp.pinMode(motorAPins[i], OUTPUT);
    mcp.pinMode(motorBPins[i], OUTPUT);
    ledcSetup(i, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(motorPwmPins[i], i);
    motor((MotorChannel)i, Stop, 0);
  }

  // Servo
  for (int i = 5; i < 8; i++)
  {
    ledcSetup(i, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttachPin(servoPwmPins[i], i);
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

    if (values[0] < 5)
    {
      motor((MotorChannel)values[0], (MotorDirection)values[1], values[2] / 100.0);
    }
    else
    {
      servo((ServoChannel)(values[0] - 5), values[1]);
    }
  }

  Serial.printf("Encoder count:");
  Serial.printf(" %lld,", encoder.getCount());
  Serial.printf(" %lld,", encoder2.getCount());
  Serial.printf(" %lld,", encoder3.getCount());
  Serial.printf(" %lld,", encoder4.getCount());
  Serial.printf(" %lld,", encoder5.getCount());
  Serial.println();
  delay(300);
}

void motor(MotorChannel channel, MotorDirection direction, float speed = 0)
{
  int _channel = (int)channel;
  if (direction == Clockwise)
  {
    mcp.digitalWrite(motorAPins[_channel], HIGH);
    mcp.digitalWrite(motorBPins[_channel], LOW);
  }
  else if (direction == CounterClockwise)
  {
    mcp.digitalWrite(motorAPins[_channel], LOW);
    mcp.digitalWrite(motorBPins[_channel], HIGH);
  }
  else if (direction == ForceStop)
  {
    mcp.digitalWrite(motorAPins[_channel], HIGH);
    mcp.digitalWrite(motorBPins[_channel], HIGH);
  }
  else
  {
    mcp.digitalWrite(motorAPins[_channel], LOW);
    mcp.digitalWrite(motorBPins[_channel], LOW);
  }
  ledcWrite(_channel, (int)(speed * 1024));
}

void servo(ServoChannel channel, int pulseWidth)
{

  int min, max;
  switch (channel)
  {
  case Lift:
    min = SERVO_MIN;
    max = SERVO_MAX;
    break;
  case Rail:
    min = SERVO2_MIN;
    max = SERVO2_MAX;
    break;
  case Barrier:
    min = SERVO3_MIN;
    max = SERVO3_MAX;
    break;
  default:
    return;
  }
  if (pulseWidth < min)
  {
    pulseWidth = min;
  }
  if (pulseWidth > max)
  {
    pulseWidth = max;
  }
  ledcWrite(((int)channel) + 4, pulseWidth);
}

void servo(ServoChannel channel, MinMax minMax)
{
  if (minMax == Min)
  {
    servo(channel, -1);
  }
  else
  {
    servo(channel, 999);
  }
}

void motorEncoderCountdown(MotorChannel channel, MotorDirection direction, float speed, int encoderTotal)
{
  int64_t saveEncoder = getCurrentEncoder(channel);
  unsigned long long int saveTime = millis();
  motor(channel, direction, speed);
  while (true)
  {
    if (millis() - saveTime > MOTOR_PULSE_COUNTDOWN_TIMEOUT * 1000)
    {
      motor(channel, ForceStop);
      return;
    }

    int64_t elapsedEncoder = abs(getCurrentEncoder(channel) - saveEncoder);
    float remaining = abs(encoderTotal - elapsedEncoder) / encoderTotal;
    float currentSpeed;
    if (remaining <= 0.3)
    {
      currentSpeed = 0.3;
    }
    else
    {
    }
    if (encoderTotal - elapsedEncoder <= MOTOR_PULSE_COUNTDOWN_TOLERENT)
    {
      motor(channel, ForceStop);
      return;
    }
    else if (encoderTotal - elapsedEncoder < 0)
    {

      if (direction == Clockwise)
      {
        motor(channel, CounterClockwise, remaining);
      }
      else
      {
        motor(channel, Clockwise, remaining);
      }
    }
    else if (encoderTotal - elapsedEncoder > 0)
    {
      motor(channel, direction, remaining);
    }
    delay(100);
  }
}

int64_t getCurrentEncoder(MotorChannel channel)
{
  switch (channel)
  {
  case Front:
    return encoder.getCount();
  case Back:
    return encoder2.getCount();
  case Left:
    return encoder3.getCount();
  case Right:
    return encoder4.getCount();
  default:
    return 0;
  }
}