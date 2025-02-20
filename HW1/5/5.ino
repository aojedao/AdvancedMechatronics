const int led = 9;
void setup()
{
  pinMode(led,OUTPUT);
  Serial.begin(9600);
}
/*
void loop()
{
  // for loop generate ramp from 0 to 255
  int i = 0;
  bool frwd_flag = 1;
  while(i>=0&&i<=255){
    frwd_flag = i==255?0:1;
    analogWrite(led,i);
    //frwd_flag?i++:i--;
    //frwd_flag?i+=5:i-=5;
    Serial.println(i<<1);
  }
}
*/

void loop(){
  
  int i = 1;
  bool frwd_flag = 1;
  while(true){
    frwd_flag = (i >= 255) ? 0 : (i <= 1) ? 1 : frwd_flag;
    i = frwd_flag?i+=5:i-=5;
    analogWrite(led,i);
    //delay(100);
    Serial.println(i);
  }
  
}
