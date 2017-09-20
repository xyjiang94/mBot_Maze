#include <MeMCore.h>

// line-following sensor setup
const int DataPin = A3;
uint8_t Sensor_Data[6];
uint8_t pattern;

// Motor setup
// M1 - left
// M2 - right
MeDCMotor motor1(M1);
MeDCMotor motor2(M2);
int motorSpeed = 100;
int deltaSpeed = 50;
int turnSpeed = 150;
int leftTurnCount = 0;

// Button setup
bool isOn = false;

void setup() 
{
    Serial.begin(9600);
    delay(100);
    Serial.println("Start.....");
    pinMode(7, INPUT); //Define button pin as input
}

void loop()
{
  if (analogRead(7) < 100)
  {
    isOn = !isOn;
  }
  if (isOn)
  {
    solve();
//    forward();
      
  }

//  oneInchForward();
//  Serial.println("stop!!!");
//  motor1.stop();
//  motor2.stop();
//  delay(5000);
}

void driveMotor(int lSpeed, int rSpeed, unsigned long t)
{
      motor1.run( lSpeed );
      motor2.run( rSpeed );
      delay(t);
}

boolean hasTurnLeft(uint8_t pattern)
{
  return (pattern == B111110 || pattern == B111101 || pattern == B111100 || pattern == B111001);
}

boolean hasTurnRight(uint8_t pattern)
{
  return (pattern == B011111 || pattern == B101111 || pattern == B100111 || pattern == B100111);
} 

void oneInchForward()
{
      Serial.println("****ONE Inch Forward");
      driveMotor(- motorSpeed, motorSpeed, 500);
}

void turnLeft()
{
      Serial.println("****TRUN LEFT");
      Serial.println(leftTurnCount);
//      driveMotor(2 * motorSpeed, 2 * motorSpeed, 825);
      if (leftTurnCount == 3)
      {
        leftTurnCount = 0;
        return;
      }
      leftTurnCount ++;
      while (!hasTurnLeft(getValue())) 
      {
        driveMotor(turnSpeed, turnSpeed, 50);
      }
}

void turnRight()
{
      Serial.println("****TRUN RIGHT");
//      driveMotor(- 2 * motorSpeed, - 2 * motorSpeed, 825);
      while (!hasTurnRight(getValue())) 
      {
        driveMotor(- turnSpeed, - turnSpeed, 50);
      }
}

void UTurn()
{
      Serial.println("****U TRUN");
//      driveMotor(- 2 * motorSpeed, - 2 * motorSpeed, 1500);
      while (!hasTurnRight(getValue())) 
      {
        driveMotor(- turnSpeed, - turnSpeed, 50);
      }
}

void forward()
{
      Serial.println("****Forward");
      driveMotor(- motorSpeed, motorSpeed, 50);
}

void slightlyLeft()
{
      Serial.println("****Slightly Left");
      driveMotor(- motorSpeed + deltaSpeed, motorSpeed + deltaSpeed, 50); 
}

void slightlyRight()
{
      Serial.println("****Slightly Right");
      driveMotor(- motorSpeed - deltaSpeed, motorSpeed - deltaSpeed, 50); 
}

void solve()
{
  //correct pattern = B110111 (line at S4)
  pattern = getValue();
  switch (pattern)
  {
    // normal
    case B110111:
      forward();
      break;
    
    // too right in straight line
    case B110011:
    case B111011:   
      slightlyLeft();
      break;

    // too left in straight line
    case B100111:
    case B101111:  
      slightlyRight();
      break;

    // Left turn / straight or left
    case B111000:
    case B110000:
    case B100000:
    {
      Serial.println("Going to turn left");
      oneInchForward();
      turnLeft();
    }
    break;

    // Right turn / straight or right
    case B000111:
    case B001111:
    case B000011:
    {
      leftTurnCount = 0;
      oneInchForward();
      pattern = getValue();
      if (pattern == B111111)
      {
        Serial.println("This is a right turn");
        turnRight();
      }
      else
      {
        Serial.println("straight or right -> straight");
      }
    }
    break;

    // T cross / x cross / end
    case B000000:
    {
        leftTurnCount = 0;
        oneInchForward();
        pattern = getValue();
        if (pattern = B000000)
        {
          Serial.println("Reach the end!!!YAY!!!");
          return; // reach the end
        }
        else
        {
          Serial.println("T / x cross road");
          turnLeft();
        }
    }
    break;

    //dead end
    case B111111:
    {
      UTurn();
      leftTurnCount = 0;
    }
    break;

    default:
      delay(100);
    break;
  }
}


uint8_t getValue()
{  
    long time_out_flag = 0;
    pinMode(DataPin, OUTPUT);
    digitalWrite(DataPin, LOW);
    delayMicroseconds(980);
    digitalWrite(DataPin, HIGH);
    delayMicroseconds(40);
    pinMode(DataPin, INPUT_PULLUP);
    delayMicroseconds(50); 
    time_out_flag = millis();
    while((digitalRead(DataPin) == 0)&&((millis() - time_out_flag) < 6)); 
    time_out_flag = millis();
    while((digitalRead(DataPin) == 1)&&((millis() - time_out_flag) < 6));
    for(uint8_t k=0; k<3; k++)
    {
        Sensor_Data[k] = 0x00;
        for(uint8_t i=0;i<8;i++)
        {
            time_out_flag = millis(); 
            while(digitalRead(DataPin) == 0&&((millis() - time_out_flag) < 6));
            uint32_t HIGH_level_read_time = micros();
            time_out_flag = millis(); 
            while(digitalRead(DataPin) == 1&&((millis() - time_out_flag) < 6));
            HIGH_level_read_time = micros() - HIGH_level_read_time;
           if(HIGH_level_read_time > 50 && HIGH_level_read_time < 100)  
            {
                Sensor_Data[k] |= (0x80 >> i);
            }
        }
    }
    if (Sensor_Data[1] == (uint8_t)(~(uint8_t)Sensor_Data[0]))
    {
       return Sensor_Data[0];
    }
}
