#include <MeMCore.h>
#include <Ultrasonic.h>

// Motor setup
MeDCMotor motor1(M1);
MeDCMotor motor2(M2);
uint8_t motorSpeed = 50;
double errorL = 0;
double errorR = 0;

// UltrasonicSensor setup
MeUltrasonicSensor frontUltra(PORT_3);
Ultrasonic leftUltra(10,9);
Ultrasonic rightUltra(1,0);
double frontDistance;
double leftDistance;
double rightDistance;
  
// Buzzer setup
MeBuzzer buzzer;

// Button setup
bool isOn = false;

void setup() {
  Serial.begin(9600);
  pinMode(7, INPUT); //Define button pin as input
}

void loop() {
  if (analogRead(7) < 100)
  {
    isOn = !isOn;
  }
  if (isOn)
  {
    run();
  }
}

void run() {
  if (rightDistance < 10.0)
  {
    buzzer.tone(394, 100);
  }
  frontDistance = frontUltra.distanceCm();
  leftDistance = leftUltra.Ranging(CM);
  rightDistance = rightUltra.Ranging(CM);
  double avrgDistance = (leftDistance + rightDistance) / 2;
  errorL = avrgDistance - leftDistance;
  errorR = avrgDistance - rightDistance;
  motor1.run(-(motorSpeed + errorL * 2));
  motor2.run(motorSpeed + errorR * 2);

  delay(100);
}

void stop() 
{
  motor1.stop();
  motor2.stop();
}


