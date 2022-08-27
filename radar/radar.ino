#include <SR04.h>
#include <Servo.h>

Servo servo;
SR04 hc( 3, 2 );

int pos    = 0;
int direct = 1;
int tick   = 0;

void setup() {
  servo.attach( 9 );

  Serial.begin( 9600 );
}

void forward() {
  pos = pos + 1;
  servo.write( pos );

  if ( pos == 181 ) {
    direct = 0;
  }
}

void backward() {
  pos = pos - 1;
  
  servo.write( pos );

  if ( pos == 0 ) {
    direct = 1;
  }
}

void rateLimitedRead() {
  tick++;

  if ( tick == 60 ) {
    Serial.println( hc.Distance() );
    tick = 0;
  }
}

void loop() {
  
  if ( direct == 1 ) {
    forward();
  } else {
    backward();
  }

  rateLimitedRead();

  delay( 15 );
}
