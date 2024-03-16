
char fdArrs [8]  = {B10000000, B11000000, B01000000, B01100000, B00100000, B00110000, B00010000, B10010000};
 byte dir=0;
 int mSpeed=1200;

void setup() {
  // put your setup code here, to run once:
  DDRD = DDRD | B11110000;              // this sets pins 0-3 as inputs and 4 to 7 as outputs 
//  pinMode(9,OUTPUT);
//  digitalWrite(9,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  dir=!dir;
  for(int i=0;i<500;i++){
     motStep(dir);   
  }
}

void motStep(int dir){
  if(dir==0){
      for (int i=0;i<8;i++){
       PORTD = fdArrs[i]|B00001100; 
       delayMicroseconds(mSpeed);
     }   
  }
  if(dir==1)
  {
     for (int i=0;i<8;i++){
       int j=7-i;
       PORTD = fdArrs[j]|B00001100; 
       delayMicroseconds(mSpeed);
     } 
   }
   stopMot();
}

void stopMot(){
    PORTD = B00000000;  // this stops the motor 
}
