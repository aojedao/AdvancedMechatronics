#define WAIT_TIME 100
#define BLINK_FREQ 1000

static bool blink_flag = false;
void setup(){
  Serial.begin(9600);//start serial comm
  DDRB |= (1<<PB1); // set pin9 as the output pin
  PORTB &= B000000; //flush PORTB to off
  
}

void loop(){
  Serial.println("Enter the Input [ON/OFF/BLINK/STOP]");
  
  while(Serial.available() == 0){//waits until user gives input
    if(blink_flag)//if the flag is set to blinking continue blinking
      blink();    // when asked to stop stop blinking
  }
  
  String  ip = Serial.readString();
  ip.replace("\n","");
  ip.replace("\r","");


  if(ip == "ON" || ip == "on") {
    blink_flag = false;
    on();
  }
  else if(ip == "off" || ip == "OFF") {
    blink_flag = false;
    off();
  }
  else if(ip == "blink" || ip == "BLINK"){
  	blink_flag = true;  
    blink();
  }
  else if(ip == "stop" || ip == "STOP"){
    blink_flag = false;
    off();
  }
  else
    Serial.println("Invalid Input");
  
  
}

void on(){
  PORTB |= (1<<PB1);
  delay(WAIT_TIME);
}

void off(){
  PORTB &= ~(1<<PB1);
  blink_flag = false;
  delay(WAIT_TIME);
}

void blink(){
    PORTB |= (1<<PB1);
    delay(BLINK_FREQ);
    PORTB &= ~(1<<PB1);
    delay(BLINK_FREQ);
}
