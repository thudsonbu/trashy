#include <Servo.h>

#define speedPinR 9          // Front Wheel PWM pin connect Model-Y M_B ENA
#define RightMotorDirPin1 22 // Front Right Motor direction pin 1 to Model-Y M_B IN1 (K1)
#define RightMotorDirPin2 24 // Front Right Motor direction pin 2 to Model-Y M_B IN2 (K1)
#define LeftMotorDirPin1 26  // Front Left Motor direction pin 1 to Model-Y M_B IN3 (K3)
#define LeftMotorDirPin2 28  // Front Left Motor direction pin 2 to Model-Y M_B IN4 (K3)
#define speedPinL 10         // Front Wheel PWM pin connect Model-Y M_B ENB

#define speedPinRB 11        // Rear Wheel PWM pin connect Left Model-Y M_A ENA
#define RightMotorDirPin1B 5 // Rear Right Motor direction pin 1 to Model-Y M_A IN1 (K1)
#define RightMotorDirPin2B 6 // Rear Right Motor direction pin 2 to Model-Y M_A IN2 (K1)
#define LeftMotorDirPin1B 7  // Rear Left Motor direction pin 1 to Model-Y M_A IN3 (K3)
#define LeftMotorDirPin2B 8  // Rear Left Motor direction pin 2 to Model-Y M_A IN4 (K3)
#define speedPinLB 12        // Rear Wheel PWM pin connect Model-Y M_A ENB

#define LPT 2 // scan loop counter

#define SERVO_PIN 13 // servo connect to D5

#define Echo_PIN 31 // Ultrasonic Echo pin connect to A5
#define Trig_PIN 30 // Ultrasonic Trig pin connect to A4

#define FAST_SPEED 160
#define SPEED 120
#define TURN_SPEED 120
#define BACK_SPEED 90
#define BACK_FAST_SPEED 160

int left_scan_val, center_scan_val, right_scan_val, left_diagonal_scan_val, right_diagonal_scan_val;

const int distance_limit = 30;     // distance limit for obstacles in front
const int side_distance_limit = 30; // minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)

int distance;
int numcycles = 0;

const int turntime = 250; // Time the robot spends turning (miliseconds)
const int backtime = 300; // Time the robot spends turning (miliseconds)

int thereis;
Servo head;

/*motor control*/
void go_forward() // Forward
{
  FR_fwd();
  FL_fwd();
  RR_fwd();
  RL_fwd();
}
void go_left() // Turn left
{
  FR_fwd();
  FL_bck();
  RR_fwd();
  RL_bck();
}
void go_right() // Turn right
{
  FR_bck();
  FL_fwd();
  RR_bck();
  RL_fwd();
}
void go_back() // Reverse
{
  FR_bck();
  FL_bck();
  RR_bck();
  RL_bck();
}

void stop_Stop() // Stop
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, LOW);
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, LOW);
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, LOW);
  set_Motorspeed(0, 0, 0, 0);
}

/*set motor speed */
void set_Motorspeed(int leftFront, int rightFront, int leftBack, int rightBack)
{
  analogWrite(speedPinL, leftFront);
  analogWrite(speedPinR, rightFront);
  analogWrite(speedPinLB, leftBack);
  analogWrite(speedPinRB, rightBack);
}

void FR_fwd() // front-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
}
void FR_bck() // front-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
}
void FL_fwd() // front-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
}
void FL_bck() // front-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
}

void RR_fwd() // rear-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);
}

void RR_bck() // rear-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);
}
void RL_fwd() // rear-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1B, HIGH);
  digitalWrite(LeftMotorDirPin2B, LOW);
}
void RL_bck() // rear-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, HIGH);
}

/*detection of ultrasonic distance*/
int watch()
{
  long echo_distance;

  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN, LOW);

  echo_distance = pulseIn(Echo_PIN, HIGH);
  echo_distance = echo_distance * 0.01657; // how far away is the object in cm

  return round(echo_distance);
}
// Measures distances to the right, left, front, left diagonal, right diagonal and assign them in cm to the variables right_scan_val,
// left_scan_val, centerscanval, left_diagonal_scan_val and right_diagonal_scan_val (there are 5 points for distance testing)

String checkSurroundings()
{
  // obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
  // for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha

  int obstacle_status = B100000;
  center_scan_val = watch();
  if (center_scan_val < distance_limit)
  {
    stop_Stop();

    obstacle_status = obstacle_status | B100;
  }

  head.write(120);
  delay(100);

  left_diagonal_scan_val = watch();
  if (left_diagonal_scan_val < distance_limit)
  {
    stop_Stop();

    obstacle_status = obstacle_status | B1000;
  }

  head.write(170); // Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);

  left_scan_val = watch();
  if (left_scan_val < side_distance_limit)
  {
    stop_Stop();

    obstacle_status = obstacle_status | B10000;
  }

  head.write(90); // use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);

  center_scan_val = watch();
  if (center_scan_val < distance_limit)
  {
    stop_Stop();

    obstacle_status = obstacle_status | B100;
  }

  head.write(40);
  delay(100);

  right_diagonal_scan_val = watch();
  if (right_diagonal_scan_val < distance_limit)
  {
    stop_Stop();

    obstacle_status = obstacle_status | B10;
  }

  head.write(0);
  delay(100);

  right_scan_val = watch();
  if (right_scan_val < side_distance_limit)
  {
    stop_Stop();

    obstacle_status = obstacle_status | 1;
  }

  head.write(90); // Finish looking around (look forward again)
  delay(300);

  String obstacle_str = String(obstacle_status, BIN);
  obstacle_str = obstacle_str.substring(1, 6);

  return obstacle_str; // return 5-character string standing for 5 direction obstacle status
}

void auto_avoidance()
{
  ++numcycles;

  if (numcycles >= LPT)
  { // Watch if something is around every LPT loops while moving forward
    stop_Stop();
    String obstacle_sign = checkSurroundings(); // 5 digits of obstacle_sign binary value means the 5 direction obstacle status

    if (obstacle_sign == "10000")
    {
      set_Motorspeed(FAST_SPEED, SPEED, FAST_SPEED, SPEED);
      go_forward();
      delay(turntime);
      stop_Stop();
    }

    else if (obstacle_sign == "00001")
    {
      set_Motorspeed(SPEED, FAST_SPEED, SPEED, FAST_SPEED);
      go_forward();
      delay(turntime);
      stop_Stop();
    }

    else if (obstacle_sign == "11100" || obstacle_sign == "01000" || obstacle_sign == "11000" || obstacle_sign == "10100" || obstacle_sign == "01100" || obstacle_sign == "00100" || obstacle_sign == "01000")
    {
      go_right();
      set_Motorspeed(TURN_SPEED, TURN_SPEED, TURN_SPEED, TURN_SPEED);
      delay(turntime);
      stop_Stop();
    }

    else if (obstacle_sign == "00010" || obstacle_sign == "00111" || obstacle_sign == "00011" || obstacle_sign == "00101" || obstacle_sign == "00110" || obstacle_sign == "01010")
    {
      go_left(); // Turn left
      set_Motorspeed(TURN_SPEED, TURN_SPEED, TURN_SPEED, TURN_SPEED);
      delay(turntime);
      stop_Stop();
    }

    else if (obstacle_sign == "01111" || obstacle_sign == "10111" || obstacle_sign == "11111")
    {
      go_back();
      set_Motorspeed(BACK_FAST_SPEED, BACK_SPEED, BACK_FAST_SPEED, BACK_SPEED);
      delay(backtime);
      stop_Stop();
    }

    else if (obstacle_sign == "11011" || obstacle_sign == "11101" || obstacle_sign == "11110" || obstacle_sign == "01110")
    {
      go_back();
      set_Motorspeed(BACK_SPEED, BACK_FAST_SPEED, BACK_SPEED, BACK_FAST_SPEED);
      delay(backtime);
      stop_Stop();
    }

    else
      Serial.println("no handle");

    numcycles = 0; // Restart count of cycles
  }

  else
  {
    set_Motorspeed(SPEED, SPEED, SPEED, SPEED);
    go_forward(); // if nothing is wrong go forward using go() function above.
    delay(backtime);
    stop_Stop();
  }

  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance < distance_limit)
  { // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
    Serial.println("final go back");
    go_back();
    set_Motorspeed(BACK_FAST_SPEED, BACK_SPEED, BACK_FAST_SPEED, BACK_SPEED);
    delay(backtime);
    ++thereis;
  }

  if (distance > distance_limit)
  {
    thereis = 0;
  } // Count is restarted
  if (thereis > 25)
  {
    Serial.println("final stop");
    stop_Stop(); // Since something is ahead, stop moving.
    thereis = 0;
  }
}

void setup()
{
  /*setup L298N pin mode*/
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);
  pinMode(speedPinLB, OUTPUT);

  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);
  pinMode(speedPinRB, OUTPUT);

  stop_Stop(); // stop move

  /*init HC-SR04*/
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);

  /*init buzzer*/
  digitalWrite(Trig_PIN, LOW);
  /*init servo*/
  head.attach(SERVO_PIN);
  head.write(0);
  delay(1000);
  head.write(179);
  delay(1000);
  head.write(90);
  delay(2000);

  Serial.begin(9600);

  stop_Stop();
}

void loop()
{
  auto_avoidance();
}
