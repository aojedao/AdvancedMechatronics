//2 - control the 7 segment led using bitwise operation.

//P4: Segment B
//P5: Segment DP
//P6: Segment A
//P7: Segment C
//P8: Segment F
//P9: Segment D
//P10: Segment G
//P11: Segment E

#define B 4 1<<PD4
#define DP 5 1<<PD5
#define A 6 1<<PD6
#define C 7 1<<PD7
#define F 8 1<<PB0
#define D 9 1<<PB1
#define G 10 1<<PB2
#define E 11 1<<PB3

void setup(){
  DDRD |= (B11110000);
  DDRB |= B001111;
  
}

void zero(void) {
  //ABCDEF
  PORTD = ((1<<PD4) | (1<<PD6) | (1<<))
}

void one(void) {
  //BC
}

void two(void) {
  //ABGED
}

void three(void) {
  //ABGCD
}

void four(void) {
  //BCFG
}

void five(void) {
  //AFGCD
}

void six(void) {
  //ACDEFGG
}

void seven(void) {
  //ABC
}

void eight(void) {
  //ABCDEFG
}

void nine(void) {
  //ABCDFG
  
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
      	zero();
      	delay(1000);
      	break;
      case 1 :
      	one();
		delay(1000);
      	break;
      case 2:
      	two();
		delay(1000);
      	break;
      case 3:
      	three();
		delay(1000);
      	break;
      

      case 4:
      	four();
		delay(1000);
      	break;

      case 5:
      	five();
		delay(1000);
      	break;
      case 6:
      	six();
		delay(1000);
      	break;
      case 7:
      	seven();
		delay(1000);
      	break;
      
      case 8:
	    eight();
		delay(1000);
      	break;
      case 9:
      	nine();
		delay(1000);
      	break;
      }
    //increment n if frwd_flg = 0;
    if(n == 0) {
      frwd_flag = 1;
      delay(2000);
    }
    if(n == 9) {
      frwd_flag = 0;
      delay(2000);
    }
    n = frwd_flag?++n:--n;
    }
}

