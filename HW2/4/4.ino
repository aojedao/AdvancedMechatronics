const int signalPin = 9;


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
  Serial.begin(9600);
  DDRB |= (1<<PB2);
}

void loop()
{
  // measure the ping time in cm
  dist = 0.01723 * readUltrasonicDistance(signalPin);
  Serial.print(dist);
  Serial.println(" cm");
  delay(100); // Wait for 100 millisecond(s)
  if(dist < 10) PORTB |= (1<<PB2); // if dist < 10 cm turn on
  else if(dist >=10 && dist <= 50){
    //if dist between 10 and 50 cm blink at 1Hz
    PORTB |=(1<<PB2);
    delay(500);
    PORTB &= ~(1<<PB2);
    delay(500);
  }
  else PORTB &= ~(1<<PB2);//if dist > 50cm turn off the light
}
