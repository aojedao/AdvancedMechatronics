#include <SoftwareSerial.h>
#include <ParallaxLCD.h>
ParallaxLCD lcd(2,2,8); // desired pin, rows, cols
void setup () {
  lcd.setup();
  delay(1000);
  lcd.backLightOn();
  delay(1000);
  lcd.on();
  lcd.at(0,1,"Hello World...");
  lcd.at(1,1,"from Parallax!");
  delay(1000);
  lcd.off();
  delay(1000);
  lcd.on();
  lcd.cr();
  lcd.empty();
  lcd.pos(0,1);
  delay(1000);
  lcd.lf();
  delay(1000);
  lcd.at(1,1,"Hello World...");
  delay(1000);
  lcd.empty();
  lcd.off(); 
}
void loop(){ }
