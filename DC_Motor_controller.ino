#define TRIG 8
#define ECHO 9

#define VL 3
#define L1 4
#define L2 5

#define VR 10
#define R1 11
#define R2 12

void setup()
{
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  Serial.begin(9600);
  
}

void createTrigPulse()
{
  digitalWrite(TRIG,0);   
  delayMicroseconds(2);
  digitalWrite(TRIG,1); 
  delayMicroseconds(5); 
  digitalWrite(TRIG,0);
}

double distance()
{
  createTrigPulse();
  unsigned long duration = pulseIn(ECHO, HIGH);
  return duration/2/29.412;
}

void lMotor(int m)
{
  if (m == 0) 
  {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
  } else if (m == 1) 
  {
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
  } else
  {
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
  }
}

void lSpeed(int v)
{
  analogWrite(VL, v);
}

void rMotor(int m) 
{
  if (m == 0) 
  {
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
  } else if (m == 1) 
  {
    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
  } else
  {
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
  }
}

void rSpeed(int v)
{
  analogWrite(VR, v);
}

int speedCal(int d)
{
  return map(d, 50, 250,1,10);
}

void loop()
{
  double d = distance();
  if (d > 50 && d < 250) {
    int v = speedCal(d);
    rMotor(1);
  	rSpeed(v);
    lMotor(1);
    lSpeed(v);
  } else {
    rMotor(0);
  }
  Serial.println(distance());
  delay(500);
}
