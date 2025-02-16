//2 - control the 7 segment led using bitwise operation.

//P4: Segment B
//P5: Segment DP
//P6: Segment A
//P7: Segment C
//P8: Segment F
//P9: Segment D
//P10: Segment G
//P11: Segment E

#define A 6 (1<<PD6)
#define B 4 (1<<PD4)
#define C 7 (1<<PD7)


#define D 9 (1<<PB1)
#define E 11 (1<<PB3)
#define F 8 (1<<PB0)
#define G 10 (1<<PB2)
#define DP 5 1<<PD5


void setup(){
  DDRD |= (B11110000);
  DDRB |= B001111;
  PORTB = B00000000;
  
}
void all_off(){
	PORTB &= B000000;//set all to LED's LOW
  	PORTD &= B00000000;
}
void zero(bool frwd_flag) {
  //ABCDEF
  if(frwd_flag){
      PORTD |= ((1<<PD4) | (1<<PD6) | (1<<PD7) ); // SET BAC
      PORTB |= ((1<<PB1) | (1<<PB3) | (1<<PB0)); // SET DEF
  }
  else{
    PORTD |= ((1<<PD6));//ON A
    PORTB |= ( (1<<PB1)| (1<<PB3) | (1<<PB0));//ON DEF
  }
}

void one(bool frwd_flag) {
  //BC
  if(frwd_flag){
    PORTB &= ~( (1<<PB1)| (1<<PB3) | (1<<PB0)); //OFF DEF
    PORTD &= ~((1<<PD6)); //OFF A
  }
  else{
    PORTD &= ~((1<<PD6));//OFF A
    PORTB &= ~((1<<PB1)|(1<<PB2)|(1<<PB3));//OFF GED
    PORTD |= (1<<PD7);//ON C
  }
}

void two(bool frwd_flag) {
  //ABGED
  if(frwd_flag){
    PORTD &= ~(1<<PD7);  //OFF C
    PORTB |= ((1<<PB2)| (1<<PB3)|(1<<PB1)); //ON DEG
    PORTD |= ((1<<PD6)); // ON A
  }
  else{
    PORTD &= ~(1<<PD7);//OFF C
    PORTB |= (1<<PB3);//ON E
  }
}

void three(bool frwd_flag) {
  //ABGCD
  if(frwd_flag){
    PORTB &= ~(1<<PB3);//OFF E
    PORTD |= (1<<PD7);//ON C
  }
  else{
    PORTD |= (1<<PD6);//ON A
    PORTB &= ~((1<<PB0));//OFF F
    PORTB |= ((1<<PB1));//ON D
  }
  
}

void four(bool frwd_flag) {
  //BCFG
  if(frwd_flag){
    PORTD &= ~(1<<PD6);//OFF A
    PORTB &= ~(1<<PB1);//OFF D
    PORTB |=(1<<PB0);//ON F
    }
  else{
    PORTD &= ~(1<<PD6);//OFF A
    PORTD |= (1<<PD4);;//ON B
    PORTB &= ~(1<<PB1);//OFF D
  }
}

void five(bool frwd_flag) {
  //AFGCD
  if(frwd_flag){
    PORTD &= ~(1<<PD4);//OFF B
    PORTD |= (1<<PD6);//ON A
    PORTB |=(1<<PB1);//ON D
    }
  else{
    PORTB &= ~(1<<PB3);//OFF E
    
  }
}

void six(bool frwd_flag) {
  //ACDEFG
  if(frwd_flag){
    PORTB |= (1<<PB3);//ON E
  }
  else{
    PORTD &= ~(1<<PD4);//OFF B
    PORTD |= ((1<<PD6)|(1<<PD7));//AC
    PORTB |= ((1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3));//DEFG
  }
}

void seven(bool frwd_flag) {
  //ABC
  if(frwd_flag){
    PORTB &= ~((1<<PB1) |(1<<PB3) |(1<<PB0)| (1<<PB2));//OFF DEFG
    PORTD |= (1<<PD4);//ON B
  }
  else{
    PORTB &= ~((1<<PB1)|(1<<PB3)|(1<<PB0)|(1<<PB2));//OFF DEFG
  }
}

void eight(bool frwd_flag) {
  //ABCDEFG
  if(frwd_flag){
    PORTB |=((1<<PB1)|(1<<PB3)|(1<<PB0)|(1<<PB2));//ON DEFG
  }
  else{
    PORTB |= ((1<<PB3));//SET E
  }
}

void nine(bool frwd_flag) {
  //ABCDFG
  if(frwd_flag){
    PORTB &= ~(1<<PB3);//OFF E
  }
  else{
    PORTD |= ((1<<PD6)|(1<<PD4)|(1<<PD7));//ABC
    PORTB |= ((1<<PB0)|(1<<PB1)|(1<<PB2));//DFG
  }
}

// Start

// Start
void loop(void)
{
  int n = 0;
  bool frwd_flag = true;
  while(n >=0 && n<=9){
    switch(n){
      case 0:
      	zero(frwd_flag);
      	delay(1000);
      	break;
      case 1 :
      	one(frwd_flag);
		delay(1000);
      	break;
      case 2:
      	two(frwd_flag);
		delay(1000);
      	break;
      case 3:
      	three(frwd_flag);
		delay(1000);
      	break;
      

      case 4:
      	four(frwd_flag);
		delay(1000);
      	break;

      case 5:
      	five(frwd_flag);
		delay(1000);
      	break;
      case 6:
      	six(frwd_flag);
		delay(1000);
      	break;
      case 7:
      	seven(frwd_flag);
		delay(1000);
      	break;
      
      case 8:
	    eight(frwd_flag);
		delay(1000);
      	break;
      case 9:
      	nine(frwd_flag);
		delay(1000);
      	break;
      }
    //increment n if frwd_flg = 0;
    n = frwd_flag?++n:--n;
    if(n == -1) {
      frwd_flag = true;
      all_off();
      delay(2000);
      n = 0;
    }
    if(n == 10) {
      frwd_flag = false;
      all_off();
      delay(2000);
      n = 9;
    }
    
    }
}

