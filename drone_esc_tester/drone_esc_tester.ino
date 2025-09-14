#include <Servo.h>
Servo esc; // create servo object to control ESC
void setup() {
  // put your setup code here, to run once:
  esc.attach(9); // connect ESC signal wire to arduino pin 9
  esc.writeMicroseconds(1000); //send "stop" signal (1ms pulse)
  delay(2000); //wait 2 seconds 
}

void loop() {
  // put your main code here, to run repeatedly:
  //Example: slowly ramp motor speed up and down

  //Increase throttle
  for(int speed = 1000; speed<=2000;speed +=10){
    esc.writeMicroseconds(speed);
    delay(20);
  }

  delay(1000); //hold at max

  //Decrease throttle
  for(int speed = 2000; speed >= 1000; speed -=10){
    esc.writeMicroseconds(speed);
    delay(20);
  }

  delay(1000); //hold at min
}

