#include <Servo.h>

// Front Right (FR) Motor
#define RightMotorDirPin1 22 // Front Right Motor direction pin 1 to Model-Y M_B IN1 (K1)
#define RightMotorDirPin2 24 // Front Right Motor direction pin 2 to Model-Y M_B IN2 (K1)
#define SpeedPinFR 9         // Front Wheel PWM pin connect Model-Y M_B ENA

// Front Left (FL) Motor
#define LeftMotorDirPin1 26 // Front Left Motor direction pin 1 to Model-Y M_B IN3 (K3)
#define LeftMotorDirPin2 28 // Front Left Motor direction pin 2 to Model-Y M_B IN4 (K3)
#define SpeedPinFL 10       // Front Wheel PWM pin connect Model-Y M_B ENB

// Rear Right (RR) Motor
#define RightMotorDirPin1B 5 // Rear Right Motor direction pin 1 to Model-Y M_A IN1 (K1)
#define RightMotorDirPin2B 6 // Rear Right Motor direction pin 2 to Model-Y M_A IN2 (K1)
#define SpeedPinRR 11        // Rear Wheel PWM pin connect Model-Y M_A ENA

// Rear Left (RL) Motor
#define LeftMotorDirPin1B 7 // Rear Left Motor direction pin 1 to Model-Y M_A IN3 (K3)
#define LeftMotorDirPin2B 8 // Rear Left Motor direction pin 2 to Model-Y M_A IN4 (K3)
#define SpeedPinRL 12       // Rear Wheel PWM pin connect Model-Y M_A ENB

#define LPT 2 // scan loop counter

#define SERVO_PIN 13 // servo connect to D5

#define ECHO_PIN 31 // Ultrasonic Echo pin connect to A5
#define TRIG_PIN 30 // Ultrasonic Trig pin connect to A4

#define SPEED 120

const int minimum_distance = 30; // Minimum distance from obstacle before evasive action

Servo radar_servo;

////// UTILS //////

int seconds(int seconds) {
  return seconds * 1000;
}

////// MOTORS //////

// Setup control pins for OUTPUT to motor driver (L298N)
void initialize_motor_driver()
{
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);

  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);

  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);

  pinMode(SpeedPinFR, OUTPUT);
  pinMode(SpeedPinFL, OUTPUT);

  pinMode(SpeedPinRL, OUTPUT);
  pinMode(SpeedPinRR, OUTPUT);
}

void set_motor_speed(int frontLeft, int frontRight, int rearLeft, int rearRight)
{
  analogWrite(SpeedPinFL, frontLeft);
  analogWrite(SpeedPinFR, frontRight);
  analogWrite(SpeedPinRR, rearRight);
  analogWrite(SpeedPinRL, rearLeft);
}

// Each motor has two pins. One sets the motor direction to forward while the
// other sets the motor direction to reverse. These are utility functions
// to set each motors direction.

void set_fr_forward()
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
}

void set_fr_reverse()
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
}

void stop_fr()
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, LOW);
}

void set_fl_forward()
{
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
}

void set_fl_reverse()
{
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
}

void stop_fl()
{
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, LOW);
}

void set_rr_forward()
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);
}

void set_rr_reverse()
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);
}

void stop_rr()
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, LOW);
}

void set_rl_forward()
{
  digitalWrite(LeftMotorDirPin1B, HIGH);
  digitalWrite(LeftMotorDirPin2B, LOW);
}

void set_rl_reverse()
{
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, HIGH);
}

void stop_rl()
{
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, LOW);
}

void stop_motors()
{
  stop_fr();
  stop_fl();
  stop_rr();
  stop_rl();

  set_motor_speed(0, 0, 0, 0);
}

void go_forward()
{
  set_fr_forward();
  set_fl_forward();

  set_rr_forward();
  set_rl_forward();

  set_motor_speed(SPEED, SPEED, SPEED, SPEED);
}

void go_reverse()
{
  set_fr_reverse();
  set_fl_reverse();

  set_rr_reverse();
  set_rl_reverse();

  set_motor_speed(SPEED, SPEED, SPEED, SPEED);
}

void rotate_left()
{
  set_fr_forward();
  set_fl_reverse();

  set_rr_forward();
  set_rl_reverse();

  set_motor_speed(SPEED, SPEED, SPEED, SPEED);
}

void rotate_right()
{
  set_fr_reverse();
  set_fl_forward();

  set_rr_reverse();
  set_rl_forward();

  set_motor_speed(SPEED, SPEED, SPEED, SPEED);
}

////// RADAR //////

// Setup ultrasonic pins make sure that ultrasonic sensor is not echoing
// before start
void initialize_ultrasonic_sensor()
{
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);
}

// Attach radar servo to bin and check that radar servo is centered correctly.
void initialize_radar_servo()
{
  radar_servo.attach(SERVO_PIN);

  radar_servo.write(0);
  delay(seconds(2));

  radar_servo.write(179);
  delay(seconds(2));

  radar_servo.write(90);
  delay(seconds(2));
}

// Take a reading from the ultrasonic sensor
int take_radar_reading()
{
  long obstacle_distance;

  digitalWrite(TRIG_PIN, LOW);
  delay(Microseconds(5));

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15);

  digitalWrite(TRIG_PIN, LOW);

  obstacle_distance = pulseIn(ECHO_PIN, HIGH);
  obstacle_distance = obstacle_distance * 0.01657;

  return round(obstacle_distance);
}

// Check if obstacle within minimum distance
int check_for_obstacle()
{
  int obstacle_distance = take_radar_reading();

  if (obstacle_distance >= minimum_distance) {
    return false;
  }

  return true;
}

////// LOOPS ///////

void setup()
{
  initialize_motor_driver();

  initialize_ultrasonic_sensor();

  initialize_radar_servo();

  stop_motors();


}

void loop()
{

}