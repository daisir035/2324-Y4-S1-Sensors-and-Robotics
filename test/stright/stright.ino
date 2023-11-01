// Define pins connections
// Motor speed pins
int E1 = 11;
int E2 = 3;

// Motor direction pins
int M1 = 12;
int M2 = 8;

// Motor speed Max 255 Min 0
int Mot_speed;
int Mot_speed1;

void setup()
{
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  // Assign a value to Mot_speed for example 200
  Mot_speed = 200;
}

void loop()
{
  // Direction control HIGH and LOW opposite directions
  delay(1000);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

  // Speed control
  analogWrite(E1, Mot_speed);   
  analogWrite(E2, Mot_speed);
  delay(1000);
  
  analogWrite(E1, 0);   
  analogWrite(E2, 0);
  delay(1000);
  
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  analogWrite(E1, Mot_speed);   
  analogWrite(E2, Mot_speed);
  delay(1000);  
}
