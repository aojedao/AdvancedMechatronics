const int signalPin = 7;

volatile bool flag;
int dist = 0;

long readUltrasonicDistance(int signalPin)
{
 
  pinMode(signalPin, OUTPUT);  // Clear the trigger
  digitalWrite(signalPin, LOW);
  delayMicroseconds(2);
  // Sets the trigger pin to HIGH state for 10 microseconds
  digitalWrite(signalPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(signalPin, LOW);
  pinMode(signalPin, INPUT);
  // Reads the echo pin, and returns the sound wave travel time in microseconds
  return pulseIn(signalPin, HIGH);
}

void setup()
{

  pinMode(2,INPUT_PULLUP);
  pinMode(7, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2),isr,LOW);
  Serial.begin(9600);
}

void loop(){
  if(flag == 1 ){
    UltraRead();
    flag=0;
    delay(1000);
  }
}
void UltraRead()
{
  // measure the ping time in cm
  dist = 0.01723 * readUltrasonicDistance(signalPin);
  Serial.print(dist);
  Serial.println(" cm");
  delay(500); // Wait for 100 millisecond(s)
}

void isr(){
  //UltraRead();
  flag=1;
  //Serial.print("test");
}

