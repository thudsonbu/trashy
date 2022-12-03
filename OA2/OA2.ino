#include <Servo.h>

// Motors

// Front Right (FR) Motor
#define FR_DIR_PIN_F 22 // Front right motor forward direction pin
#define FR_DIR_PIN_R 24 // Front right motor reverse direction pin
#define FR_SPEED_PIN 9  // Front right motor speed pin

// Front Left (FL) Motor
#define FL_DIR_PIN_F 26 // Front left motor forward direction pin
#define FL_DIR_PIN_R 28 // Front left motor reverse direction pin
#define FL_SPEED_PIN 10 // Front left motor speed pin

// Rear Right (RR) Motor
#define RR_DIR_PIN_F 5 // Rear right Motor forward direction pin
#define RR_DIR_PIN_R 6 // Rear right motor reverse direction pin
#define RR_SPEED_PIN 11 // Rear right motor speed pin

// Rear Left (RL) Motor
#define RL_DIR_PIN_F 7 // Rear left motor forward direction pin
#define RL_DIR_PIN_R 8 // Rear left motor reverse direction pin
#define RL_SPEED_PIN 12 // Rear left motor speed pin

#define SPEED 120

// Servo
#define SERVO_PIN 13 // servo connect to D5

// Ultrasonic Sensor
#define ECHO_PIN 31 // Ultrasonic Echo pin connect to A5
#define TRIG_PIN 30 // Ultrasonic Trig pin connect to A4

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
  pinMode(FR_DIR_PIN_F, OUTPUT);
  pinMode(FR_DIR_PIN_R, OUTPUT);

  pinMode(FL_DIR_PIN_F, OUTPUT);
  pinMode(FL_DIR_PIN_R, OUTPUT);

  pinMode(RR_DIR_PIN_F, OUTPUT);
  pinMode(RR_DIR_PIN_R, OUTPUT);

  pinMode(RL_DIR_PIN_F, OUTPUT);
  pinMode(RL_DIR_PIN_R, OUTPUT);

  pinMode(FR_SPEED_PIN, OUTPUT);
  pinMode(FL_SPEED_PIN, OUTPUT);

  pinMode(RL_SPEED_PIN, OUTPUT);
  pinMode(RR_SPEED_PIN, OUTPUT);
}

void set_each_motor_speed(int front_left, int front_right, int rear_left, int rear_right)
{
  analogWrite(FL_SPEED_PIN, front_left);
  analogWrite(FR_SPEED_PIN, front_right);
  analogWrite(RL_SPEED_PIN, rear_left);
  analogWrite(RR_SPEED_PIN, rear_right);
}

void set_all_motor_speed(int speed)
{
  set_each_motor_speed(speed, speed, speed, speed);
}

// Each motor has two pins. One sets the motor direction to forward while the
// other sets the motor direction to reverse. These are utility functions
// to set each motors direction.

void set_fr_forward()
{
  digitalWrite(FR_DIR_PIN_F, HIGH);
  digitalWrite(FR_DIR_PIN_R, LOW);
}

void set_fr_reverse()
{
  digitalWrite(FR_DIR_PIN_F, LOW);
  digitalWrite(FR_DIR_PIN_R, HIGH);
}

void stop_fr()
{
  digitalWrite(FR_DIR_PIN_F, LOW);
  digitalWrite(FR_DIR_PIN_R, LOW);
}

void set_fl_forward()
{
  digitalWrite(FL_DIR_PIN_F, HIGH);
  digitalWrite(FL_DIR_PIN_R, LOW);
}

void set_fl_reverse()
{
  digitalWrite(FL_DIR_PIN_F, LOW);
  digitalWrite(FL_DIR_PIN_R, HIGH);
}

void stop_fl()
{
  digitalWrite(FL_DIR_PIN_F, LOW);
  digitalWrite(FL_DIR_PIN_R, LOW);
}

void set_rr_forward()
{
  digitalWrite(RR_DIR_PIN_F, HIGH);
  digitalWrite(RR_DIR_PIN_R, LOW);
}

void set_rr_reverse()
{
  digitalWrite(RR_DIR_PIN_F, LOW);
  digitalWrite(RR_DIR_PIN_R, HIGH);
}

void stop_rr()
{
  digitalWrite(RR_DIR_PIN_F, LOW);
  digitalWrite(RR_DIR_PIN_R, LOW);
}

void set_rl_forward()
{
  digitalWrite(RL_DIR_PIN_F, HIGH);
  digitalWrite(RL_DIR_PIN_R, LOW);
}

void set_rl_reverse()
{
  digitalWrite(RL_DIR_PIN_F, LOW);
  digitalWrite(RL_DIR_PIN_R, HIGH);
}

void stop_rl()
{
  digitalWrite(RL_DIR_PIN_F, LOW);
  digitalWrite(RL_DIR_PIN_R, LOW);
}

void stop_motors()
{
  stop_fr();
  stop_fl();
  stop_rr();
  stop_rl();

  set_each_motor_speed(0, 0, 0, 0);
}

void go_forward()
{
  set_fr_forward();
  set_fl_forward();

  set_rr_forward();
  set_rl_forward();

  set_all_motor_speed(SPEED);
}

void go_reverse()
{
  set_fr_reverse();
  set_fl_reverse();

  set_rr_reverse();
  set_rl_reverse();

  set_all_motor_speed(SPEED);
}

void rotate_left()
{
  set_fr_forward();
  set_fl_reverse();

  set_rr_forward();
  set_rl_reverse();

  set_all_motor_speed(SPEED);
}

void rotate_right()
{
  set_fr_reverse();
  set_fl_forward();

  set_rr_reverse();
  set_rl_forward();

  set_all_motor_speed(SPEED);
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
  delay(seconds(5));
}

// Take a reading from the ultrasonic sensor
int take_radar_reading()
{
  long obstacle_distance;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15);

  digitalWrite(TRIG_PIN, LOW);

  obstacle_distance = pulseIn(ECHO_PIN, HIGH);
  obstacle_distance = obstacle_distance * 0.01657;

  return round(obstacle_distance);
}

// Check if obstacle within minimum distance
bool check_for_obstacle()
{
  // Take multiple reading to avoid noise
  int obstacle_distance_1 = take_radar_reading();
  int obstacle_distance_2 = take_radar_reading();

  if (obstacle_distance_1 >= minimum_distance &&
      obstacle_distance_2 >= minimum_distance) {
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
  if (check_for_obstacle()) {
    stop_motors();
    rotate_left();
    delay(seconds(1));
    stop_motors();
  } else {
    go_forward();
  }
}
