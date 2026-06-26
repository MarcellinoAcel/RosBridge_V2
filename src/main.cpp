#include <Arduino.h>
#include <ESP32Encoder.h>
#include <config.h>
#include <kinematic.h>
#include <pid.h>

#define COMMAND_RATE 20 // hz
#define DEBUG_RATE 5
#define SERIAL_BAUD 115200
#define CMD_TIMEOUT_MS 400

void moveBase();
void stopBase();
void printDebug();
void parseSerialCommand();
void parseData();
void publishData(float vx, float vy, float wz);
void setMotor(int cw_pin, int ccw_pin, int pwm_pin, float pwm);

const int enca[3] = {MOTOR1_ENC_A, MOTOR2_ENC_A, MOTOR3_ENC_A};
const int encb[3] = {MOTOR1_ENC_B, MOTOR2_ENC_B, MOTOR3_ENC_B};

ESP32Encoder motor1;
ESP32Encoder motor2;
ESP32Encoder motor3;

unsigned long prevT = 0;
unsigned long g_prev_command_time = 0;

const int pwmFreq = 30000;
const int pwmChannel = 0;
const int pwmResolution = 8; // 8-bit resolution (0-255)

// cmd_vel stored as plain floats instead of ROS Twist
float cmd_linear_x = 0.0;
float cmd_linear_y = 0.0;
float cmd_angular_z = 0.0;

PID wheel1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematic kinematic(
    Kinematic::ROBOT_BASE,
    MOTOR_MAX_RPS,
    MAX_RPS_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    ROBOT_DIAMETER);

bool g_command_received = false; // add this flag
static unsigned long prev_control_time = 0;
static unsigned long prev_debug_time = 0;
// Variables to store the received velocity data
float vx = 0.0;
float vy = 0.0;
float wz = 0.0;

void setup()
{
  Serial.begin(SERIAL_BAUD);

  pinMode(Motor1_IN1, OUTPUT);
  pinMode(Motor1_IN2, OUTPUT);
  pinMode(Motor1_PWM, OUTPUT);
  pinMode(Motor2_IN1, OUTPUT);
  pinMode(Motor2_IN2, OUTPUT);
  pinMode(Motor2_PWM, OUTPUT);
  pinMode(Motor3_IN1, OUTPUT);
  pinMode(Motor3_IN2, OUTPUT);
  pinMode(Motor3_PWM, OUTPUT);

  motor1.attachHalfQuad(MOTOR1_ENC_A, MOTOR1_ENC_B);
  motor2.attachHalfQuad(MOTOR2_ENC_A, MOTOR2_ENC_B);
  motor3.attachHalfQuad(MOTOR3_ENC_A, MOTOR3_ENC_B);

  motor1.setCount(37);
  motor2.setCount(37);
  motor3.setCount(37);

  motor1.clearCount();
  motor2.clearCount();
  motor3.clearCount();

  wheel1.ppr_total(COUNTS_PER_REV1);
  wheel2.ppr_total(COUNTS_PER_REV2);
  wheel3.ppr_total(COUNTS_PER_REV3);

  // Configure LEDC PWM peripheral for ESP32
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(Motor3_PWM, pwmChannel);
  ledcAttachPin(Motor2_PWM, pwmChannel);
  ledcAttachPin(Motor1_PWM, pwmChannel);

  g_prev_command_time = millis(); // ← add this line
}

void loop()
{
  parseData();

  // if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  // {
  moveBase();
  //   prev_control_time = millis();
  // }

  // if ((millis() - g_prev_command_time) >= CMD_TIMEOUT_MS && g_command_received)
  // {
  //   stopBase();
  // }

  // if (DEBUG && (millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
  // {
  //   printDebug();
  //   prev_debug_time = millis();
  // }
  // ← g_prev_command_time and g_command_received removed from here
}

// Parses incoming serial string in format: "x,y,z\n"
// Example: "0.5,0.0,0.1\n"
void parseSerialCommand()
{
  static char buf[64];
  static uint8_t idx = 0;

  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n')
    {
      buf[idx] = '\0';
      idx = 0;

      float x = 0.0, y = 0.0, z = 0.0;
      if (sscanf(buf, "%f,%f,%f", &x, &y, &z) == 3)
      {
        cmd_linear_x = x;
        cmd_linear_y = y;
        cmd_angular_z = z;
        g_prev_command_time = millis();
        g_command_received = true; // ← add this
      }
      return;
    }

    if (idx < sizeof(buf) - 1)
    {
      buf[idx++] = c;
    }
  }
}

void parseData()
{
  if (Serial.available() > 0)
  {
    // Read the incoming string payload until a newline character
    String inputString = Serial.readStringUntil('\n');

    // Clean up any hidden whitespace carriage returns (\r) from the string
    inputString.trim();

    // Convert the String to a standard C-string character array for parsing
    char inputBuffer[64];
    inputString.toCharArray(inputBuffer, sizeof(inputBuffer));

    // Use sscanf to safely extract three floats separated by commas and spaces
    // The format "%f, %f, %f" handles optional spaces around the commas perfectly
    int parsedFields = sscanf(inputBuffer, "%f, %f, %f", &vx, &vy, &wz);

    // Verify that all 3 components were parsed successfully
    if (parsedFields == 3)
    {

        cmd_linear_x = vx;
        cmd_linear_y = vy;
        cmd_angular_z = wz;

      // -------------------------------------------------------------
      // TODO: Pass your parsed 'vx', 'vy', and 'wz' variables
      // directly into your kinematic.h library functions here!
      // -------------------------------------------------------------
    }
    else
    {
      // Error handling if data was corrupted or format mismatched
      Serial.print("[Error] Failed parsing input. Expected 3 floats, got values for: ");
      Serial.println(parsedFields);
    }
  }
}

Kinematic::rps req_rps;
void moveBase()
{
  req_rps = kinematic.getRPS(
      cmd_linear_x,
      cmd_linear_y,
      cmd_angular_z);

  unsigned long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;

  long p0, p1, p2;

  // Atomically snapshot all three counters
  p0 = motor1.getCount();
  p1 = motor2.getCount();
  p2 = motor3.getCount();

  float controlled_motor1 = wheel1.control_speed(req_rps.motor1, p0, deltaT);
  float controlled_motor2 = wheel2.control_speed(req_rps.motor2, p1, deltaT);
  float controlled_motor3 = wheel3.control_speed(req_rps.motor3, p2, deltaT);

  float current_rps1 = wheel1.get_filt_vel();
  float current_rps2 = wheel2.get_filt_vel();
  float current_rps3 = wheel3.get_filt_vel();
  float current_rps4 = 0.0;

  prevT = currT;

  if (fabs(req_rps.motor1) < 0.02)
    controlled_motor1 = 0.0;
  if (fabs(req_rps.motor2) < 0.02)
    controlled_motor2 = 0.0;
  if (fabs(req_rps.motor3) < 0.02)
    controlled_motor3 = 0.0;

  setMotor(cw[0], ccw[0], pwm[0], controlled_motor1);
  setMotor(cw[1], ccw[1], pwm[1], controlled_motor2);
  setMotor(cw[2], ccw[2], pwm[2], controlled_motor3);

  Kinematic::velocities vel = kinematic.getVelocities(
      current_rps1,
      current_rps2,
      current_rps3,
      current_rps4);

  // Publish current velocities back to host as CSV: "vx,vy,wz\n"
  publishData(
      vel.linear_x,
      vel.linear_y,
      vel.angular_z);
}

// Sends odometry velocities to host over Serial
void publishData(float vx, float vy, float wz)
{
  Serial.print(vx, 4);
  Serial.print(',');
  Serial.print(vy, 4);
  Serial.print(',');
  Serial.println(wz, 4);
}

void stopBase()
{
  cmd_linear_x = 0.0;
  cmd_linear_y = 0.0;
  cmd_angular_z = 0.0;

  setMotor(cw[0], ccw[0], pwm[0], 0);
  setMotor(cw[1], ccw[1], pwm[1], 0);
  setMotor(cw[2], ccw[2], pwm[2], 0);
}

void printDebug()
{
  char buffer[80];
  sprintf(buffer, "[DBG] RPS FL:%.3f FR:%.3f RL:%.3f",
          req_rps.motor1,
          req_rps.motor2,
          req_rps.motor3);
  Serial.println(buffer);
}

void setMotor(int cw_pin, int ccw_pin, int pwm_pin, float pwm)
{
  if (pwm > 0)
  {
    digitalWrite(cw_pin, HIGH);
    digitalWrite(ccw_pin, LOW);
    analogWrite(pwm_pin, fabs(pwm));
  }
  else if (pwm < 0)
  {
    digitalWrite(cw_pin, LOW);
    digitalWrite(ccw_pin, HIGH);
    analogWrite(pwm_pin, fabs(pwm));
  }
  else
  {
    digitalWrite(cw_pin, LOW);
    digitalWrite(ccw_pin, LOW);
    analogWrite(pwm_pin, 0);
  }
}
