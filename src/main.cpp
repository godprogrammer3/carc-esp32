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
#define MOTOR_PULSE_COUNTDOWN_TOLERENT 15

// Servo
#define SERVO_PWM_FREQ 50
#define SERVO_PWM_RESOLUTION 8
#define SERVO_PWM_PIN 16
#define SERVO2_PWM_PIN 4
#define SERVO3_PWM_PIN 0
#define SERVO_MIN 6
#define SERVO_MAX 35
#define SERVO2_MIN 6
#define SERVO2_MAX 41
#define SERVO3_MIN 20
#define SERVO3_MAX 25

// IR
#define IR_PIN 10
#define IR2_PIN 11

// Button
#define BUTTON_PIN 13

// Led
#define LED_PIN 12

// Rotate
#define ROTATE_TIMEOUT 60
enum MotorDirection
{
  Clockwise,
  CounterClockwise,
  Stop,
  ForceStop,
};
enum MotorChannel
{
  Back,
  Left,
  Front,
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
enum RotateDirection
{
  LeftRotate,
  RightRotate,
  UTurnRotate,
};
enum IRChannel
{
  IRChannel0,
  IRChannel1,
};
enum RotateMotor
{
  FrontBack,
  LeftRight,
};
typedef struct
{
  MotorChannel channel;
  MotorDirection direction;
  float speed;
  int encoderTotal;
} MotorEncoderCountdownParams;
enum IRFilter
{
  IR1,
  IR2,
  IR1orIR2,
  IR1andIR2,
};

const int8_t motorPwmPins[] = {MOTOR_PWM_PIN, MOTOR2_PWM_PIN, MOTOR3_PWM_PIN, MOTOR4_PWM_PIN, MOTOR5_PWM_PIN};
const int8_t motorAPins[] = {MOTOR_A_PIN, MOTOR2_A_PIN, MOTOR3_A_PIN, MOTOR4_A_PIN, MOTOR5_A_PIN};
const int8_t motorBPins[] = {MOTOR_B_PIN, MOTOR2_B_PIN, MOTOR3_B_PIN, MOTOR4_B_PIN, MOTOR5_B_PIN};
const int8_t servoPwmPins[] = {SERVO_PWM_PIN, SERVO2_PWM_PIN, SERVO3_PWM_PIN};

void motor(MotorChannel channel, MotorDirection direction, float speed);
void servo(ServoChannel channel, int pulseWidth);
int64_t getCurrentEncoder(MotorChannel channel);
void motorEncoderCountdown(MotorEncoderCountdownParams params, bool *completedStatus);
bool readIR(IRChannel irChannel);
bool readButton();
void writeLED(bool state);
bool readLED();
void move(float degree, int encoderTotal, float speed);
void move(float degree, float speed);
void rotate(RotateDirection rotateDirection, float speed, RotateMotor rotateMotor);
void rotate(float degree, float speed, RotateMotor rotateMotor);
void clearAllEncoder();
void stopAllMotor();
void start();
void moveUntil(float degree, float speed, IRFilter filter, bool ir, bool ir2);
void forceStopAllMotor();
void calibrate();
ESP32Encoder encoder;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;
ESP32Encoder encoder5;

Adafruit_MCP23X17 mcp;

int multitaskCompletedCount = 0;
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
    delay(2000);
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
    ledcAttachPin(servoPwmPins[i - 5], i);
  }

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);

  // LED
  pinMode(LED_PIN, OUTPUT);

  // IR
  mcp.pinMode(IR_PIN, INPUT);
  mcp.pinMode(IR2_PIN, INPUT);

  while (true)
  {
    if (readButton())
    {
      writeLED(false);
      delay(1000);
      break;
    }
    writeLED(!readLED());
    delay(300);
  }
}

void loop()
{
  if (Serial.available())
  {
    String str = Serial.readString();
    Serial.printf("Input: %s\n", str.c_str());
    int values[10];
    int startIndex = 0;
    int endIndex = str.indexOf(",");
    values[0] = str.substring(startIndex, endIndex).toInt();

    startIndex = endIndex + 1;
    endIndex = str.indexOf(",", startIndex);

    values[1] = str.substring(startIndex, endIndex).toInt();

    startIndex = endIndex + 1;
    endIndex = str.indexOf(",", startIndex);

    values[2] = str.substring(startIndex, endIndex).toInt();

    startIndex = endIndex + 1;
    endIndex = str.indexOf(",", startIndex);

    values[3] = str.substring(startIndex, endIndex).toInt();

    startIndex = endIndex + 1;
    endIndex = str.indexOf(",", startIndex);

    values[4] = str.substring(startIndex, endIndex).toInt();

    startIndex = endIndex + 1;
    endIndex = str.indexOf(",", startIndex);

    values[5] = str.substring(startIndex).toInt();

    if (values[0] < 5)
    {
      motor((MotorChannel)values[0], (MotorDirection)values[1], values[2] / 100.0);
    }
    else if (values[0] < 8)
    {
      servo((ServoChannel)(values[0] - 5), values[1]);
    }
    else if (values[0] < 9)
    {
      move((float)values[1], values[2], values[3] / 100.0);
    }
    else if (values[0] < 10)
    {
      rotate((float)values[1], values[2] / 100.0, (RotateMotor)values[3]);
    }
    else if (values[0] < 11)
    {
      moveUntil(values[1], values[2] / 100.0, (IRFilter)values[3], values[4], values[5]);
    }
    else
    {
      calibrate();
    }
  }

  // start();
  // Serial.printf("IR: %d, %d\n", readIR(IRChannel0), readIR(IRChannel1));
  // delay(300);
  start();
  while (true)
  {
    delay(100);
  }
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

void motorEncoderCountdown(MotorEncoderCountdownParams params, bool *completedStatus)
{
  int64_t elapsedEncoder = abs(getCurrentEncoder(params.channel));
  // Serial.printf("elapsedEncoder: %lld\n", elapsedEncoder);
  if (abs(params.encoderTotal - elapsedEncoder) <= MOTOR_PULSE_COUNTDOWN_TOLERENT)
  {
    motor(params.channel, ForceStop);
    multitaskCompletedCount++;
    *completedStatus = true;
    return;
  }
  float percentProgress = abs(elapsedEncoder / params.encoderTotal * 100.0);
  // if (percentProgress > 100.0)
  // {
  //   motor(params.channel, ForceStop);
  //   delay(300);
  //   if (params.direction == Clockwise)
  //   {
  //     params.direction = CounterClockwise;
  //   }
  //   else
  //   {
  //     params.direction = Clockwise;
  //   }
  // }

  // float currentSpeed;
  // if (percentProgress < 20.0)
  // {
  //   currentSpeed = 5 * params.speed / params.encoderTotal * elapsedEncoder;
  // }
  // else if (percentProgress > 60.0)
  // {
  //   currentSpeed = -2.5 * params.speed / params.encoderTotal * elapsedEncoder + 2.5;
  // }
  // else
  // {
  //   currentSpeed = params.speed;
  // }
  // if (currentSpeed < 0.3)
  // {
  //   currentSpeed = 0.3;
  // }

  // motor(params.channel, params.direction, currentSpeed);
  motor(params.channel, params.direction, params.speed);
}

int64_t getCurrentEncoder(MotorChannel channel)
{
  switch (channel)
  {
  case Back:
    return encoder.getCount();
  case Left:
    return encoder2.getCount();
  case Front:
    return encoder3.getCount();
  case Right:
    return encoder4.getCount();
  case Grabber:
    return encoder5.getCount();
  default:
    return 0;
  }
}

bool readIR(IRChannel irChannel)
{
  if (irChannel == IRChannel0)
  {
    return mcp.digitalRead(IR_PIN);
  }
  else
  {
    return mcp.digitalRead(IR2_PIN);
  }
}

bool readButton()
{
  return digitalRead(BUTTON_PIN);
}

void writeLED(bool state)
{
  digitalWrite(LED_PIN, state);
}

bool readLED()
{
  return digitalRead(LED_PIN);
}

void move(float degree, int encoderTotal, float speed)
{
  float speedMotorFrontBack = abs(cos(degree * PI / 180.0)) * speed;
  int encoderMotorFrontBack = abs(cos(degree * PI / 180.0)) * encoderTotal;
  float speedMotorLeftRight = abs(sin(degree * PI / 180.0)) * speed;
  int encoderMotorLeftRight = abs(sin(degree * PI / 180.0)) * encoderTotal;

  MotorDirection motorFrontDirection, motorBackDirection;
  if (degree >= 90 && degree <= 270)
  {
    motorFrontDirection = Clockwise;
    motorBackDirection = CounterClockwise;
  }
  else
  {
    motorFrontDirection = CounterClockwise;
    motorBackDirection = Clockwise;
  }

  MotorDirection motorLeftDirection, motorRightDirection;
  if (degree >= 0 && degree <= 180)
  {
    motorLeftDirection = CounterClockwise;
    motorRightDirection = Clockwise;
  }
  else
  {
    motorLeftDirection = Clockwise;
    motorRightDirection = CounterClockwise;
  }

  multitaskCompletedCount = 0;
  clearAllEncoder();
  MotorEncoderCountdownParams front;
  front.channel = Front;
  front.direction = motorFrontDirection;
  front.speed = speedMotorFrontBack;
  front.encoderTotal = encoderMotorFrontBack;
  MotorEncoderCountdownParams back;
  back.channel = Back;
  back.direction = motorBackDirection;
  back.speed = speedMotorFrontBack;
  back.encoderTotal = encoderMotorFrontBack;
  MotorEncoderCountdownParams left;
  left.channel = Left;
  left.direction = motorLeftDirection;
  left.speed = speedMotorLeftRight;
  left.encoderTotal = encoderMotorLeftRight;
  MotorEncoderCountdownParams right;
  right.channel = Right;
  right.direction = motorRightDirection;
  right.speed = speedMotorLeftRight;
  right.encoderTotal = encoderMotorLeftRight;
  bool frontCompleteStatus = false;
  bool backCompleteStatus = false;
  bool leftCompleteStatus = false;
  bool rightCompleteStatus = false;
  while (multitaskCompletedCount < 4)
  {
    if (!frontCompleteStatus)
    {
      motorEncoderCountdown(front, &frontCompleteStatus);
    }
    if (!backCompleteStatus)
    {
      motorEncoderCountdown(back, &backCompleteStatus);
    }
    if (!leftCompleteStatus)
    {
      motorEncoderCountdown(left, &leftCompleteStatus);
    }
    if (!rightCompleteStatus)
    {
      motorEncoderCountdown(right, &rightCompleteStatus);
    }
  }
}

void rotate(RotateDirection rotateDirection, float speed, RotateMotor rotateMotor = LeftRight)
{
  switch (rotateDirection)
  {
  case LeftRotate:
    rotate(600, speed, rotateMotor);
    break;
  case RightRotate:
    rotate(-600, speed, rotateMotor);
    break;
  case UTurnRotate:
    rotate(1200, speed, rotateMotor);
    break;
  default:
    return;
  }
}
void rotate(float degree, float speed, RotateMotor rotateMotor = LeftRight)
{
  MotorEncoderCountdownParams motor;
  MotorEncoderCountdownParams motor2;
  if (degree > 0)
  {
    motor.direction = Clockwise;
    motor2.direction = Clockwise;
  }
  else
  {
    motor.direction = CounterClockwise;
    motor2.direction = CounterClockwise;
  }

  if (rotateMotor == FrontBack)
  {
    motor.channel = Front;
    motor2.channel = Back;
  }
  else
  {
    motor.channel = Left;
    motor2.channel = Right;
  }
  motor.speed = speed;
  motor2.speed = speed;
  degree = abs(degree);
  motor.encoderTotal = degree;
  motor2.encoderTotal = degree;

  bool isMotorComplete = false;
  bool isMotor2Complete = false;
  multitaskCompletedCount = 0;
  stopAllMotor();
  clearAllEncoder();
  while (multitaskCompletedCount < 2)
  {
    if (!isMotorComplete)
    {
      motorEncoderCountdown(motor, &isMotorComplete);
    }
    if (!isMotor2Complete)
    {
      motorEncoderCountdown(motor2, &isMotor2Complete);
    }
  }
}

void clearAllEncoder()
{
  encoder.clearCount();
  encoder2.clearCount();
  encoder3.clearCount();
  encoder4.clearCount();
  encoder5.clearCount();
}

void stopAllMotor()
{
  motor(Front, Stop, 0);
  motor(Back, Stop, 0);
  motor(Left, Stop, 0);
  motor(Right, Stop, 0);
}

void moveUntil(float degree, float speed, IRFilter filter, bool ir, bool ir2)
{
  move(degree, speed);
  while (true)
  {
    switch (filter)
    {
    case IR1:
      if (readIR(IRChannel0) == ir)
      {
        forceStopAllMotor();
        return;
      }
      break;
    case IR2:
      if (readIR(IRChannel1) == ir2)
      {
        forceStopAllMotor();
        return;
      }
      break;
    case IR1orIR2:
      if (readIR(IRChannel0) == ir || readIR(IRChannel1) == ir2)
      {
        forceStopAllMotor();
        return;
      }
      break;
    case IR1andIR2:
      if (readIR(IRChannel0) == ir && readIR(IRChannel1) == ir2)
      {
        forceStopAllMotor();
        return;
      }
      break;
    }
  }
}

void move(float degree, float speed)
{
  float speedMotorFrontBack = abs(cos(degree * PI / 180.0)) * speed;
  float speedMotorLeftRight = abs(sin(degree * PI / 180.0)) * speed;

  MotorDirection motorFrontDirection, motorBackDirection;
  if (degree >= 90 && degree <= 270)
  {
    motorFrontDirection = Clockwise;
    motorBackDirection = CounterClockwise;
  }
  else
  {
    motorFrontDirection = CounterClockwise;
    motorBackDirection = Clockwise;
  }

  MotorDirection motorLeftDirection, motorRightDirection;
  if (degree >= 0 && degree <= 180)
  {
    motorLeftDirection = CounterClockwise;
    motorRightDirection = Clockwise;
  }
  else
  {
    motorLeftDirection = Clockwise;
    motorRightDirection = CounterClockwise;
  }
  motor(Front, motorFrontDirection, speedMotorFrontBack);
  motor(Back, motorBackDirection, speedMotorFrontBack);
  motor(Left, motorLeftDirection, speedMotorLeftRight);
  motor(Right, motorRightDirection, speedMotorLeftRight);
}

void forceStopAllMotor()
{
  motor(Front, ForceStop, 0);
  motor(Back, ForceStop, 0);
  motor(Left, ForceStop, 0);
  motor(Right, ForceStop, 0);
}

void calibrate()
{
  if (readIR(IRChannel0))
  {
    motor(Left, ForceStop, 0);
    motor(Right, Clockwise, 0.3);
    motor(Back, Clockwise, 0.3);
    while (!readIR(IRChannel1))
    {
      delay(1);
    }
  }
  else if (readIR(IRChannel1))
  {
    motor(Right, ForceStop, 0);
    motor(Left, CounterClockwise, 0.3);
    motor(Back, CounterClockwise, 0.3);
    while (!readIR(IRChannel0))
    {
      delay(1);
    }
  }
  motor(Left, Stop, 0);
  motor(Right, Stop, 0);
  motor(Back, Stop, 0.3);
}

void start()
{
  move(90, 2500, 0.8);
  delay(500);
  rotate(-562, 0.5);
  delay(500);
  moveUntil(90, 0.4, IR1orIR2, true, true);
  delay(200);
  calibrate();
  delay(300);
  move(90, 50, 0.3);
  delay(300);
  moveUntil(0, 0.3, IR2, true, true);
  delay(50);
  move(0, 45, 0.3);
}