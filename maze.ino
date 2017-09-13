#include <MeMCore.h>
#include <Ultrasonic.h>

// Motor setup
MeDCMotor motor1(M1);
MeDCMotor motor2(M2);
uint8_t motorSpeed = 100;

// UltrasonicSensor setup
MeUltrasonicSensor frontUltra(PORT_3);
Ultrasonic leftUltra(10,9);
Ultrasonic rightUltra(A1,A0);
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
  else
  {
    stop();
  }
  delay(50);

}

void run() {
  frontDistance = frontUltra.distanceCm();
  leftDistance = leftUltra.Ranging(CM);
  rightDistance = rightUltra.Ranging(CM);

  if (leftDistance > 10.0)
  {
    motor1.run(-motorSpeed);
    motor2.run(motorSpeed);
    buzzer.tone(394, 100);
    buzzer.tone(523, 100);  
  }
  else
  {
    motor1.stop();
    motor2.stop();

    for (int i = 0; i < 10; i++)
    {
      buzzer.tone(5000, 100);
      delay(100);
    }

    motor1.run(motorSpeed);
    motor2.run(-motorSpeed);
    delay(5000);
  }
  delay(100);
}

void stop() 
{
  motor1.stop();
  motor2.stop();
}

