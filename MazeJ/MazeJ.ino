
#include <MeMCore.h>

// two motors 
MeDCMotor motorLeft(M2);
MeDCMotor motorRight(M1);
int speed;

// start button 
int run;
int buttonPin;

// line follower
const int DataPin = 12;
uint8_t Sensor_Data[6];

// status value
const byte biTurn = B000000;
const byte blank = B111111;

byte lineStatus;


void stop()
{
  motorLeft.run(0);
  motorRight.run(0);
}

void move(int time,int speedLeft, int speedRight)
{
  motorLeft.run(speedLeft);
  motorRight.run(speedRight);
  if ( time > 0) 
  {
    delay(time);
  }
 }

boolean isTurnLeft(byte lineStatus)
{
  byte leftTurnCase1 = B110000;
  byte leftTurnCase2 = B111000;
  if (lineStatus == leftTurnCase1 || lineStatus == leftTurnCase2)
  {
    return true;
  }
  return false; 
}

boolean isTurnRight(byte lineStatus)
{
  byte rightTurnCase1 = B000011;
  byte rightTurnCase2 = B000111;
  if (lineStatus == rightTurnCase1 || lineStatus == rightTurnCase2)
  {
    return true;
  }
  return false;
  
}

boolean isCenter(byte lineStatus)
{
  byte centerCase1 = B110011;
  byte centerCase2 = B110111;
  byte centerCase3 = B111011;
  if (lineStatus == centerCase1 || lineStatus == centerCase2 || lineStatus == centerCase3)
  {
    return true;
  }
  return false;
}

boolean isOffLeft(byte lineStatus)
{
  byte offLeftCase1 = B011111;
  byte offLeftCase2 = B101111;
  byte offLeftCase3 = B100111;
  byte offLeftCase4 = B001111;
  if (lineStatus == offLeftCase1 || lineStatus == offLeftCase2 || lineStatus == offLeftCase3 || lineStatus == offLeftCase4)
  {
    return true;
  }
  return false; 
}

boolean isOffRight(byte lineStatus)
{
  byte offRightCase1 = B111110;
  byte offRightCase2 = B111101;
  byte offRightCase3 = B111100;
  byte offRightCase4 = B111001;
    if (lineStatus == offRightCase1 || lineStatus == offRightCase2 || lineStatus == offRightCase3 || lineStatus == offRightCase4)
  {
    return true;
  }
  return false;
}

void turnLeft()
{
  while (!isOffRight(lineStatus))
  {
    move(200, speed + 20, speed + 20);
    lineStatus = byte(getValue());
  }
}

void turnRight()
{
  while (!isOffLeft(lineStatus))
  {
    move(200, -speed- 20, -speed - 20);
    lineStatus = byte(getValue());
  }
}

void goInch()
{
   move(600, -speed, speed);
   stop();
   delay(1000);
}

void setup() {
  Serial.begin(9600);
  
  run = 0; // starts stopped 
  buttonPin = 7; // button pin is 7 in my mBot board
  pinMode(buttonPin,INPUT);

  speed = 50;
}

void loop() {
  // code without control of button
//  Serial.println(run);
  if (analogRead(buttonPin) < 100 )
  {
    if ( run == 0 ) 
    {
      run = 255;
    }
    else 
    {
      run = 0;
    }
  }

  if ( run > 0)
  {
    lineStatus = byte(getValue());
    while (isCenter(lineStatus)) 
    {
      move(200, -speed, speed);
      lineStatus = byte(getValue());
    }

    while (isOffRight(lineStatus))
    {
      move(200, -speed + 20, speed + 20);
      lineStatus = byte(getValue());
    }

    while (isOffLeft(lineStatus))
    {
      move(200, -speed - 20, speed - 20);
      lineStatus = byte(getValue());
    }

    if (isTurnLeft(lineStatus))
    {
      goInch();
      lineStatus = byte(getValue());
      if (isCenter(lineStatus) || isOffRight(lineStatus) || isOffLeft(lineStatus)) {
        turnLeft();
      }

      if (lineStatus == blank) {
        turnLeft();
      }
    }
    

    if (isTurnRight(lineStatus))
    {
      goInch();
      lineStatus = byte(getValue());
      if (isCenter(lineStatus) || isOffRight(lineStatus) || isOffLeft(lineStatus)) {
        move(200, -speed, speed); // go straight
      }
      if (lineStatus == blank) 
      {
        turnRight();
      }
    }

    if (lineStatus == blank)
    {
      turnLeft();
    }

    if (lineStatus == biTurn)
    {
      goInch();
      lineStatus = byte(getValue());
      if (isCenter(lineStatus) || isOffRight(lineStatus) || isOffLeft(lineStatus)) {
        turnLeft();
      }

      if (lineStatus == blank) {
        turnLeft();
      }
    }
    
    
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
    for(uint8_t k=0; k<6; k++)
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
