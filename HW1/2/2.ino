const byte AND_led = 8;
const byte OR_led = 9;
const byte XOR_led = 10;
const byte OP = 11;
const byte push_button1 = 5;
const byte push_button2 = 6;
const byte push_button3 = 7;

char buffer[64];


int lastButtonState = HIGH; // Set HIGH because of INPUT_PULLUP

unsigned long lastDebounceTime ;
const unsigned long debounceDelay = 50;
bool arr[2] = {0,0};
int counter;
char gates[3] = {'^','|','&'};

void setup() {
  Serial.begin(9600);
  Serial.println("Button Counter: ");

  pinMode(AND_led, OUTPUT);
  pinMode(OR_led, OUTPUT);
  pinMode(XOR_led, OUTPUT);
  pinMode(OP, OUTPUT);

  pinMode(push_button1, INPUT_PULLUP);
  pinMode(push_button2, INPUT_PULLUP);
  pinMode(push_button3, INPUT_PULLUP);
}

void loop() {
  char y_n;
  do{
  	Serial.println("Select the GATE");
  	int gate = select_gate();
  	binary_input();
  	show_output(gate);
  	delay(100);
    Serial.println("Do you want to continue? (Y/N)");
  	
    while (Serial.available() == 0) {
      // Do nothing, just wait for input
    }
    y_n = Serial.read();
  }while(y_n == 'Y' || y_n == 'y');
  
}

void show_output(int gate){
  
  int result = gate == 0 ? (arr[0]^arr[1]):gate==1?(arr[0]|arr[1]):(arr[0]&arr[1]);
  sprintf(buffer, "%d %c %d = %d",arr[0],gates[gate],arr[1],result);
  Serial.println(buffer);
  
  
}

void binary_input(){
  bool input1 = false, input2 = false;
  unsigned long lastButtonPress ;
  Serial.println("Select the first binary input: ");
  Serial.println(arr[0]);
  lastButtonState = HIGH;
  lastButtonPress = millis();
  while(millis() - lastButtonPress < 10000){
    int reading = digitalRead(push_button2);
    if(reading != lastButtonState){
      lastButtonPress = millis();
    }
    delay(3);
    if((millis() - lastButtonPress) > 0){
      if((reading == LOW) && (lastButtonState == HIGH)){
        
      	arr[0] = !arr[0];
      	Serial.println(arr[0]);
        
      }
    }
    
    
    lastButtonState = reading;
    delay(100);
  }
  Serial.println("Select the second binary input: ");
  Serial.println(arr[1]);
  lastButtonState = HIGH;
  lastButtonPress = millis();
  while(millis() - lastButtonPress < 10000){
    int reading = digitalRead(push_button3);
    
    if(reading != lastButtonState){
      lastButtonPress = millis();
    }
    delay(3);
    if((millis() - lastButtonPress) > 0){
      if((reading == LOW) && (lastButtonState == HIGH)){
        arr[1] = !arr[1];
        Serial.println(arr[1]);
      }
    }
    lastButtonState = reading;
    delay(100);
  }
  
}

int select_gate(){
  //set a 1000ms pausetime
  //create a loop, that when the button is not pressed for more than 1000ms we exit of the loop
  bool flag = 0;
  counter = 0;
  Serial.println("XOR");
  digitalWrite(XOR_led,HIGH);
  digitalWrite(AND_led,LOW);
  digitalWrite(OR_led,LOW);
  lastDebounceTime = millis();
  delay(100);
  while(!flag){
    debounce(flag,counter);
  }
  return counter;
  
}

void debounce(bool& flag, int& counter) {
  int reading = digitalRead(push_button1);
  
  if (reading != lastButtonState) {  
    lastDebounceTime = millis();  // Reset debounce timer only if the state changes
  }
    //Serial.print(reading);
    //Serial.print(" ");  // Print a space
	//Serial.print(lastButtonState);
  	delay(3);

  if ((millis() - lastDebounceTime) > 0) {  // Ensure stability
    if ((reading == LOW) && (lastButtonState == HIGH)) {  // Detect button press
      counter = ++counter%3; // Increment counter
      switch(counter){
      	case 0:
        	Serial.println("XOR");
        	digitalWrite(XOR_led,HIGH);
        	digitalWrite(AND_led,LOW);
        	digitalWrite(OR_led,LOW);
        	break;
        case 1:
			Serial.println("OR");
        	digitalWrite(XOR_led,LOW);
        	digitalWrite(AND_led,LOW);
        	digitalWrite(OR_led,HIGH);
        	break;
        case 2:
        	Serial.println("AND");
        	digitalWrite(XOR_led,LOW);
        	digitalWrite(AND_led,HIGH);
        	digitalWrite(OR_led,LOW);
        	break;
      }
    }
    
  }
  lastButtonState = reading;  // Update state after debounce
  if(millis() - lastDebounceTime > 10000) flag = 1; 
}
