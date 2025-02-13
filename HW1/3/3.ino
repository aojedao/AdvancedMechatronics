const int led = 8;
const int push_button = 5;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(led,OUTPUT);
  pinMode(push_button, INPUT_PULLUP);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Starting Counter");
  unsigned long lastDebounceTime = 0;
  int lastButtonState = HIGH;
  long long count = 0;
  while(1){
    counter(lastDebounceTime,lastButtonState,count);
    delay(100);
  }
  

}

void counter(unsigned long& lastDebounceTime, int& lastButtonState, long long& count){
  int reading = digitalRead(push_button);
  //Serial.print(reading);
  if(reading != lastButtonState){
    lastDebounceTime = millis();
  }
  delay(49);
  //Serial.println(lastDebounceTime);
  if((millis() - lastDebounceTime)> 0){
    if((reading == LOW) && (lastButtonState == HIGH)){
      ++count;
      Serial.println((long)count);
    }
  }
  lastButtonState = reading;
  
}