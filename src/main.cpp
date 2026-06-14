#include <Arduino.h>
#include <config.h>
#include <kinematic.h>
#include <pid.h>

#define COMMAND_RATE 20 // hz
#define DEBUG_RATE 5
#define SERIAL_BAUD 115200
#define CMD_TIMEOUT_MS 400

template <int j>
void readEncoder();
void moveBase();
void stopBase();
void printDebug();
void parseSerialCommand();
void publishData(float vx, float vy, float wz);
void setMotor(int cw_pin, int ccw_pin, int pwm_pin, float pwm);

const int enca[3] = {MOTOR1_ENC_A, MOTOR2_ENC_A, MOTOR3_ENC_A};
const int encb[3] = {MOTOR1_ENC_B, MOTOR2_ENC_B, MOTOR3_ENC_B};

volatile long pos[5];
unsigned long prevT = 0;
unsigned long g_prev_command_time = 0;

// cmd_vel stored as plain floats instead of ROS Twist
float cmd_linear_x = 0.0;
float cmd_linear_y = 0.0;
float cmd_angular_z = 0.0;

PID wheel1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematic kinematic(
    Kinematic::OMNI,
    MOTOR_MAX_RPS,
    MAX_RPS_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    ROBOT_DIAMETER);

void setup()
{
  Serial.begin(SERIAL_BAUD);

  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
}

void loop()
{
  static unsigned long prev_control_time = 0;
  static unsigned long prev_debug_time = 0;

  // Parse any incoming serial command
  if (Serial.available())
  {
    parseSerialCommand();
  }

  // Run motor control loop at COMMAND_RATE hz
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  {
    moveBase();
    prev_control_time = millis();
  }

  // Stop motors if no command received within timeout
  if ((millis() - g_prev_command_time) >= CMD_TIMEOUT_MS)
  {
    stopBase();
  }

  // Debug output
  if (DEBUG)
  {
    if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
    {
      printDebug();
      prev_debug_time = millis();
    }
  }
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
      }
      return;
    }

    if (idx < sizeof(buf) - 1)
    {
      buf[idx++] = c;
    }
  }
}

void moveBase()
{
  Kinematic::rps req_rps;
  req_rps = kinematic.getRPS(cmd_linear_x, cmd_linear_y, cmd_angular_z, 0.0);

  unsigned long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;

  float controlled_motor1 = wheel1.control_speed(req_rps.motor1, pos[0], deltaT);
  float controlled_motor2 = wheel2.control_speed(req_rps.motor2, pos[1], deltaT);
  float controlled_motor3 = wheel3.control_speed(req_rps.motor3, pos[2], deltaT);

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
          wheel1.get_filt_vel(),
          wheel2.get_filt_vel(),
          wheel3.get_filt_vel());
  Serial.println(buffer);
}

void setMotor(int cw_pin, int ccw_pin, int pwm_pin, float pwm)
{
  if (pwm > 0)
  {
    digitalWrite(cw_pin, HIGH);
    digitalWrite(ccw_pin, LOW);
    analogWrite(pwm_pin, (int)pwm);
  }
  else if (pwm < 0)
  {
    digitalWrite(cw_pin, LOW);
    digitalWrite(ccw_pin, HIGH);
    analogWrite(pwm_pin, (int)-pwm);
  }
  else
  {
    digitalWrite(cw_pin, LOW);
    digitalWrite(ccw_pin, LOW);
    analogWrite(pwm_pin, 0);
  }
}

template <int j>
void readEncoder()
{
  int b = digitalRead(encb[j]);
  if (b > 0)
    pos[j]++;
  else
    pos[j]--;
}