#define WAIT_TIME 5000

const byte led = 3;
long long cur_time = 0;
void setup(){
  pinMode(led,OUTPUT);
  Serial.begin(9600);
}

void loop(){
  Serial.println("Enter a value between 0 - 4096");
  while(Serial.available() == 0) {}
  if(Serial.available()>0){
    //as the input is of range 2^12 we declare the input as uint16_t
    uint16_t val = Serial.parseInt();
    
    //Serial.println(val,BIN);
    
    //AND the val with 0x0FFF (the first 12 BITS) and then right shift by 4 bits
    //so that we get the 8 bits
    
    uint8_t converted_val = ((val & 0x0FFF)>>4);
    
    //Serial.println(converted_val,BIN);
    //Serial.println(converted_val);
    
    //Write the alanog value to the Analog pins
    analogWrite(led,255);
    
    // wait for 5 secs
    delay(WAIT_TIME);
    
    analogWrite(led,0);
    
  }
  
}
