// SCL A5
// Define the pin
#include <MPU6050_tockn.h>
#include <Wire.h>
#define SAMPLERATE_DELAY_MS (500)
int pingPin = 6;
int E1 = 11;
int E2 = 3;
int spin = 10;
int M1 = 12;
int M2 = 8;
int Mot_speedL = 230;
int Mot_speedR = 200;
float directionSwitchCount = 0;
MPU6050 mpu6050(Wire);
float AngleZ;
// A5 (SCL)
void setup()
{
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
  AngleZ = mpu6050.getAngleZ();
}

void loop()
{

  long duration, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  cm = microsecondsToCentimeters(duration);
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  delay(10);
  getDirection();
  go();
  if (cm < 20)
  {
    directionSwitchCount == 2 ? stop() : (directionSwitchCount == 0 ? turnL() : turnR());
	  directionSwitchCount++;
  }
}

long microsecondsToCentimeters(long microseconds)
{

  return microseconds / 29 / 2;
}

float getDirection()
{
  mpu6050.update();
  AngleZ = mpu6050.getAngleZ();
  Serial.print("\tangleZ : ");
  Serial.println(AngleZ);
  return AngleZ;
}

void go()
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  analogWrite(E1, 150);
  analogWrite(E2, 150);
  delay(10);
}


void turnL()
{
  while (AngleZ <155)
  { 
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    analogWrite(E1, Mot_speedL);
    analogWrite(E2, Mot_speedL);
    getDirection();
    //delay(10);
  }
}

void turnR()
{
  while (AngleZ > 15 )
  {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, Mot_speedR);
    analogWrite(E2, Mot_speedR);
    getDirection();
    //delay(10);
  }
}

void stop()
{
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, 0);
    analogWrite(E2, 0);
    while(1)
    {
    	delay(100);
    }	
}
