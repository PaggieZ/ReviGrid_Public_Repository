#include <Wire.h>
#include "PID_RT.h"
#define M1 0x50 // Device Address
PID_RT PID;;

char fdArrs [8]  = {B10000000, B11000000, B01000000, B01100000, B00100000, B00110000, B00010000, B10010000};

byte dir=1;
int sSpeed=1000;
int mSpeed=0;
byte firstIntr=1;
byte gFirstIntr=1;
byte motInterPin=3;
byte genInterPin=2;
long pulseInTimeBegin=0;
long gPulseInTimeBegin=0;
long pulseInTimeEnd=0;
long gPulseInTimeEnd=0;
byte newPulseDurationAvailable=false;
byte gNewPulseDurationAvailable=false;
byte onFlag=1;
float fSpeed=0;
float gSpeed=0;
unsigned long pTime=0;
long pidTime=0;
unsigned long gPTime=0;
long pulseDuration=0;
long gPulseDuration=0;
byte load=0;
int beltPos=0;
float ratio=0;
String serialReceived;
byte spaceIndex=0;
String nmbrString;
String command;
//int numbers[4];
String numbers[4];
int numCount = 0;
byte numEntered=0;
int timeoutDur=2000;
byte numAvail=0;
int numSteps=0;
byte commandAvail=0;
byte cA=0;
byte mTog=0;
byte gTog=0;
int gLoad=0;
byte motPWM=0;
byte genPat=0;
byte motPWM_PIN=9;
byte genVoltPin=6;
int genVoltLowPin=3;
int genV=0;
int genVL=0;
int genVoltMF=0;
int genVoltLowMF=0;
byte r1=100; //ohms
byte r2=56;  //ohms
byte r3=22;  //ohms
float mw=0;
float current=0;
int maxSteps=1200;
int cruiseSpeed=1000;
byte cruiseFlag=0;
float Kp=0.02;
float Ki=0.02;
float Kd=0.0;
byte printFlag=0;
byte op=0;
long pSkip=0;
int pDur=1000;
int preOp=0;
byte busyFlag=0;

char *commands[] = {"init", "*ID?", "*IDN?", "motOn", "motOff", "rapidOn", "setMot","getSpeeds",
                    "setLoad1>1","setLoad2>1","setLoad3>1","loadAll","clearLoads",
                    "setCruise>1","cruiseOn","cruiseOff","setK>1",
                    "getRatio<1", "getBPos<1", "getVH<1", "getVL<1","getVolts<1","getVDrop<1","getW<1","getBusy<1",
                    "printOn","printOff",
                    "goHigh","goMid","goLow","goHome",
                    "beltL>1","beltR>1","cycle","testEP",
                    "off", "eoc"};                

void motInterrupt()
{
  pTime=micros();
  if(digitalRead(motInterPin)==LOW){    
    if (!mTog) {
      // start measuring
      pulseInTimeBegin = micros();
      mTog=1;
    }
    else {
      // stop measuring
      pulseInTimeEnd = micros();
      pulseDuration=pulseInTimeEnd-pulseInTimeBegin;
      newPulseDurationAvailable = true;
      mTog=0;
    }
  }
}

void genInterrupt()
{
  gPTime=micros();
  if(digitalRead(genInterPin)==LOW){    
    if (!gTog) {
      // start measuring
      gPulseInTimeBegin = micros();
      gTog=1;
    }
    else {
      // stop measuring
      gPulseInTimeEnd = micros();
      gPulseDuration=gPulseInTimeEnd-gPulseInTimeBegin;
      gNewPulseDurationAvailable = true;
      gTog=0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  DDRD = DDRD | B11110000;              // this sets pins 0-3 as inputs and 4 to 7 as outputs 
  DDRB = DDRB | B00011100;              // D10 is R1 (100ohms), D11 is R2(56ohms) and D12 is R3(22 ohms)
  pinMode(motPWM_PIN,OUTPUT);
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);

  digitalWrite(10,LOW); 
  digitalWrite(11,LOW); 
  digitalWrite(12,LOW); 
  
  PID.setPoint(cruiseSpeed); // PID setpoint
  PID.setOutputRange(0, 255);  // PID output range is PWM range
  PID.setInterval(30); // time interval between 2 compute() calls
  PID.setK(Kp, Ki, Kd);
  PID.start(); // enable PID controller to compute() new output values

  digitalWrite(motPWM_PIN,LOW); 
  attachInterrupt(digitalPinToInterrupt(motInterPin),motInterrupt,CHANGE); 
  attachInterrupt(digitalPinToInterrupt(genInterPin),genInterrupt,CHANGE); 
}

void loop() {
  checkAction(1); // execute user cmds
  checkCruise();
  checkPrint();
}

void checkAction(bool checkAll){
  if(enterSerialInput()) {
      if(serialReceived=="motOn"){motOn();}
      else if(serialReceived=="*ID?"||serialReceived=="*IDN?"){Serial.println("cvt");}
      else if(serialReceived=="getCommands"){getCommands();}
      else if(serialReceived=="motOff"){motOff();}
      else if(serialReceived=="rapidOn"){rapidMode();}
      else if(serialReceived=="setMot"){setMot();}
      else if(serialReceived=="getSpeeds"){printSpeeds();}     
      else if(serialReceived=="setLoad1"){setLoad1();}
      else if(serialReceived=="setLoad2"){setLoad2();}
      else if(serialReceived=="setLoad3"){setLoad3();}
      else if(serialReceived=="loadAll"){setLoadsAll();}
      else if(serialReceived=="clearLoads"){clearLoads();}
      else if(serialReceived=="timeTest"){timeTest();}
      else if(serialReceived=="getRatio"){getRatio();}
      else if(serialReceived=="getBPos"){getBPos();}
      else if(serialReceived=="off"){off();}
      else if(serialReceived=="getVH"){getVH();}
      else if(serialReceived=="getVL"){getVL();}
      else if(serialReceived=="getVolts"){getVolts();}
      else if(serialReceived=="getVDrop"){getVDrop();}
      else if(serialReceived=="getW"){getW();}
      else if(serialReceived=="setCruise"){setCruise();}
      else if(serialReceived=="cruiseOn"){cruiseFlag=1;}
      else if(serialReceived=="cruiseOff"){cruiseFlag=0;}
      else if(serialReceived=="setPID"){setPID();}  
      else if(serialReceived=="printOn"){printFlag=1;} 
      else if(serialReceived=="printOff"){printFlag=0;} 
      else if(serialReceived=="getBusy"){Serial.println(busyFlag);}  
      if(checkAll==1){      
         if(serialReceived=="goHigh"|serialReceived=="goEnd"){goEnd();}
         else if(serialReceived=="goLow"|serialReceived=="goHome"){goHome();} 
         else if(serialReceived=="goCenter"|serialReceived=="goMid"){goCenter();}   
         else if(serialReceived=="beltL"){beltLeft();}
         else if(serialReceived=="beltR"){beltRight();}
         else if(serialReceived=="cycle"){cycle();}
         else if(serialReceived=="init"){initDev();}
         else if(serialReceived=="testEP"){testEEPROM();}
      }
  }
}

// Function to enter and process the received string
byte enterSerialInput() {
  commandAvail=0;
  serialReceived="";
  if (Serial.available()>0){
     serialReceived = Serial.readStringUntil('\n');
     Serial.read();
     command=serialReceived;
     numCount=0;
     numEntered=0;
     for (int i = 0; i < numCount; i++) {             // zero out the numbers array
       numbers[i]="0";
     }
     serialReceived.trim();
     int spacePos = serialReceived.indexOf(' ');         // Find the position of the first space
     if (spacePos != -1) {    // Extract the command
       command = serialReceived.substring(0, spacePos);
       int startPos = spacePos + 1;                   // Move the position to the character after the first space
       while (startPos < serialReceived.length()) {      // Process the rest of the string to extract numbers
         int nextSpacePos = serialReceived.indexOf(' ', startPos);        // Find the position of the next space
         if (nextSpacePos == -1) {                    // If no more spaces are found, use the end of the string
           nextSpacePos = serialReceived.length();
         }
         String numStr = serialReceived.substring(startPos, nextSpacePos);         // Extract the substring between spaces
         if (numStr.length() > 0) {                   // Convert the substring to an integer
           numEntered=1;
           numbers[numCount] = numStr.toFloat();
           numCount++;
         }
         startPos = nextSpacePos + 1;                 // Move to the character after the next space
         if (numCount >= 4) {                         // Break if we've reached the maximum number count
           break;
         }
       }
     } 
     serialReceived=command;
     commandAvail=1;
   } 
   return(commandAvail);
}  

byte readSerTO(int numNums){
  byte numCount = 0;
  long time=millis();
  while(numCount<numNums&&(millis()-time)<timeoutDur){
    time = millis();
    while(Serial.available() < 1 && millis() - time < timeoutDur){} // check serial timeout
    // read user input
    if(millis() - time < timeoutDur){
      serialReceived = Serial.readStringUntil('\n');
      int numVal=serialReceived.toInt();  
      numVal=(max(min(255,numVal),0));
      numbers[numCount]=numVal;
      numEntered=1;
    }
    numCount++;
    return(numAvail);
  }
}

/*************************** getCommands *****************************/
/*  The getCommands() function sends out all of the commands stored in the 
    commands[] array.
*/
void getCommands() {
  byte currCmdIdx = 0;  // current cmd index
  char *currCmd = commands[0];

  while (currCmd != "eoc") {               // while the current cmd is not "eoc"
    Serial.println(commands[currCmdIdx]);  // send out the current cmd
    currCmdIdx++;
    currCmd = commands[currCmdIdx];
  }
  Serial.println("eoc");
}

void initDev(){
  busyFlag=1;
  goCenter();
  goHome();
  motOff();
  busyFlag=0;
}

void testEEPROM(){
  unsigned int address = 32100;
  writeEEPROM(M1, address, 0x17);
  readEEPROM(M1, address); 
}

void off(){
  cruiseFlag=0;
  digitalWrite(motPWM_PIN,LOW);
}

void  motOn(){
  digitalWrite(motPWM_PIN,HIGH);
}

void  motOff(){
  digitalWrite(motPWM_PIN,LOW);
}

void getBPos(){
  Serial.println(beltPos);
}

void setMot(){
  if(!numEntered){readSerTO(1);}
  if(numEntered){
    mSpeed=numbers[0].toInt(); 
    mSpeed=(max(min(255,mSpeed),0));
    analogWrite(motPWM_PIN,mSpeed);
  }
}

void setCruise(){
  if(!numEntered){readSerTO(1);}
  if(numEntered){
    cruiseSpeed=numbers[0].toInt(); 
    cruiseSpeed=(max(min(7000,cruiseSpeed),0));
    PID.setPoint(cruiseSpeed); // PID setpoint
  }
}

void setPID(){
  if(!numEntered){readSerTO(3);}
  if(numEntered){
//    Serial.println(numbers[0]);
//    Serial.println(numbers[1]);
//    Serial.println(numbers[2]);
    Kp=numbers[0].toFloat();
    Kp = max(min(1000, Kp), 0);
    Ki=numbers[1].toFloat();
    Ki = max(min(1000, Ki), 0);
    Kd=numbers[2].toFloat();
    Kd = max(min(1000, Kd), 0);
  }
//  Serial.println(Kp);
//  Serial.println(Ki);
//  Serial.println(Kd);
  PID.setK(Kp, Ki, Kd);
}

void checkCruise(){
  if(cruiseFlag){
    // calculate PID interval
    float nuTime = millis()-pidTime; 
    preOp=op;
    if(nuTime > 3000) {
      nuTime = 30;        // !check this
    }   
    if(PID.compute(gReadSpeed())) { // send cruise speed setpoint and send to PID. This is the control variable
      PID.setInterval(nuTime);      // set new PID interval
      op=PID.getOutput();           // get computed output
//      if(cruiseSpeed<gSpeed&&beltPos>0){
//        for(int j=0;j<3;j++){ motStep(0);}
//      }
//      if(cruiseSpeed>gSpeed&&beltPos<1200){
//        for(int j=0;j<3;j++){ motStep(1);}        
//      }
    }    
    if(PID.compute(gReadSpeed())) { // send cruise speed setpoint and send to PID. This is the control variable
      PID.setInterval(nuTime);      // set new PID interval
      op=PID.getOutput();           // get computed output
      analogWrite(motPWM_PIN,op);   // adjust motor pin
    }
  }
}

void checkPrint(){
  if(printFlag&&millis()-pSkip>pDur){
    mReadSpeed();
//    Serial.print("Cruise Setpoint: ");Serial.print(cruiseSpeed);Serial.print(" Actual Speed: ");Serial.print(mReadSpeed());Serial.print(" Output: ");Serial.println(op);   
    Serial.print("SetPt: ");Serial.print(cruiseSpeed);
    Serial.print(" Whl: ");Serial.print(gReadSpeed());
    Serial.print(" Mot: ");Serial.print(mReadSpeed());
    Serial.print(" Out: ");Serial.print(op/255.0*100.0);Serial.print("%");
    Serial.print(" Rat: ");Serial.print(fSpeed/gSpeed); 
    Serial.print(" BPos: ");Serial.println(beltPos);
    pSkip=millis();
  } 
}

void setLoad1(){
  digitalWrite(10,HIGH);      // switch in the 22 ohm resistor
}

void setLoad2(){ 
  digitalWrite(11,HIGH);      // switch in the 56 ohm resistor
}

void setLoad3(){
  digitalWrite(12,HIGH);      // switch in the 100 ohm resistor
}

void getRatio(){
  float ratio=mReadSpeed()/gReadSpeed();
  Serial.println(ratio);
}

void getVH(){
  Serial.println(analogRead(genVoltPin));
}

void getVL(){
  Serial.println(analogRead(genVoltLowPin));
}

void getVolts(){
  Serial.println(analogRead(genVoltPin)/1023.0*5.0);
}

void getVDrop(){
  Serial.println(max(analogRead(genVoltPin)-analogRead(genVoltLowPin),0));
}

void getW(){
  genV=analogRead(genVoltPin);
  genVL=analogRead(genVoltLowPin);
  current=(float)max(((genV-genVL)/5.0),0)/22.0;
  mw=(float)(genV/1023.0)*5.0*current;
  Serial.print("VHigh: ");Serial.print(genV);Serial.print(" VDrop: ");Serial.print(genV-genVL);Serial.print(" Cur: ");Serial.println(current);
  Serial.print("millWatts: ");Serial.println(mw);
}

void setLoadsAll(){  
  digitalWrite(10,HIGH);  
  digitalWrite(11,HIGH);
  digitalWrite(12,HIGH);
}

void clearLoads(){
  digitalWrite(10,0); digitalWrite(11,0);digitalWrite(12,0); 
}

void checkMSpd(){
  mReadSpeed();
  if(mReadSpeed()<200){
    analogWrite(motPWM_PIN,255);
    delay(200);
  }
}

void beltLeft(){
  busyFlag=1;
  checkMSpd();
  numAvail=0;
  if(!numEntered){
     readSerTO(1);
  }
  else{
      numAvail=1; 
  }
  if(numAvail){
    numSteps=numbers[0].toInt(); 
    numSteps=(max(min(1200,numSteps),0));
    for(int i=0;i<numSteps;i++){
      motStep(1);
      beltPos=min(maxSteps,beltPos+1);
      checkAction(0); 
      checkCruise();  
      checkPrint();  
    }
  }
  busyFlag=0;
}

void beltRight(){
  busyFlag=1;
  checkMSpd();
  numAvail=0;
  if(!numEntered){
     readSerTO(1);
  }
  else{
      numAvail=1; 
  }
  if(numAvail){
    numSteps=numbers[0].toInt(); 
    numSteps=(max(min(1500,numSteps),0));
    for(int i=0;i<numSteps;i++){
      motStep(0);
      beltPos=max(0,beltPos-1); 
      checkAction(0);
      checkCruise();
      checkPrint();
    }
  }
  busyFlag=0;
}

 void runLoads(){
  digitalWrite(12,HIGH);
  digitalWrite(11,HIGH);
  digitalWrite(10,HIGH);
  for(int i=0;i<10;i++){
    printVals();
 }
  digitalWrite(12,LOW);
  digitalWrite(11,LOW);
  digitalWrite(10,LOW);
  for(int i=0;i<10;i++){
    int A6=analogRead(6);
    int A3=analogRead(3);
    printVals();
  }  
 }

// used for fast control applications
void rapidMode()
{
  byte hiByte=0;
  byte loByte=0;
  byte stayByte=1;
  motPWM=0;
  genPat=0;
  while(stayByte==1|stayByte==49)
  {
      while (Serial.available()<3){}
      stayByte=Serial.read();
      motPWM = Serial.read();
      genPat = Serial.read();
      analogWrite(motPWM_PIN,motPWM);
      PORTB=(genPat<<2)|PORTB;   // move the bits left to start at digital 10 and go to digital 12 and write       
      genV=analogRead(genVoltPin);
      genVL=analogRead(genVoltLowPin);
      loByte=genV&0xFF;
      hiByte=(genV>>8)&0xFF;
      while(Serial.availableForWrite() < 4){delayMicroseconds(10);}
      Serial.write(hiByte);
      Serial.write(loByte);
      loByte=genVL&0xFF;
      hiByte=(genVL>>8)&0xFF;
      Serial.write(hiByte);
      Serial.write(loByte);     
   }
}

void runTest(){
  digitalWrite(motPWM_PIN,HIGH);
  while(1){
    printVals();
    delay(100);
  }
}

void timeTest(){
  unsigned long sTime=micros();
  for(int i=0;i<100;i++){
    delay(50);
  }
  unsigned long eTime=micros();
  Serial.print("Duration = ");Serial.println(eTime-sTime);
}

void printVals(){
    int A6=analogRead(6);
    int A3=analogRead(3);
    Serial.print("Motor Speed: ");Serial.print(mReadSpeed());Serial.print(" Gen Speed: ");Serial.print(gReadSpeed());Serial.print(" ratio: ");Serial.println(mReadSpeed()/gReadSpeed());
    Serial.print(" A6: ");Serial.print(A6);Serial.print(" A3: ");Serial.print(A3);Serial.print(" Drop: ");Serial.println(A6-A3);delay(200); 
//    Serial.print(gPulseInTimeEnd);Serial.print(" begin: ");Serial.print(gPulseInTimeBegin);Serial.print(" dur: ");Serial.println(gPulseDuration);
}

void testMotSpeeds(){
  analogWrite(motPWM_PIN,255);
  delay(2000);
  analogWrite(motPWM_PIN,222);
  delay(2000);
  analogWrite(motPWM_PIN,200);
  delay(2000);
  analogWrite(motPWM_PIN,190);
  delay(2000);
  analogWrite(motPWM_PIN,170);
  delay(2000);
  digitalWrite(motPWM_PIN,LOW);
}

void printSpeeds(){
  Serial.print(mReadSpeed());
  Serial.print(" ");
  Serial.println(gReadSpeed());
}

void continCenter(){
  digitalWrite(motPWM_PIN,HIGH);
  delay(1000);
  fSpeed=mReadSpeed();
  gSpeed=gReadSpeed();
  float ratio=fSpeed/gSpeed;
  while(1){
  dir=1;
  ratio=mReadSpeed()/gReadSpeed();
  while(ratio>1){
    ratio=mReadSpeed()/gReadSpeed();
    motStep(dir);
   //   printVals();
  }
  dir=0;
  ratio=mReadSpeed()/gReadSpeed();
  while(ratio<1){
    ratio=mReadSpeed()/gReadSpeed();
    motStep(dir);
    //  printVals();
  }
  }
}

void moveLeft(){
  busyFlag=1;
  checkMSpd();
  dir=0;
  for (int i=0;i<maxSteps;i++){
     motStep(dir);
     checkAction(0);
     checkCruise();
     checkPrint();
  }
   busyFlag=0;
}

void moveRight(){
  busyFlag=1;
  checkMSpd();
    dir=1;
  for (int i=0;i<maxSteps;i++){
     motStep(dir);
     checkAction(0);
     checkCruise();
     checkPrint();
  }
  busyFlag=0;
}

void continEnd(){
  Serial.println("in continEnd");
  digitalWrite(motPWM_PIN,HIGH);
  delay(1000);
  fSpeed=mReadSpeed();
  gSpeed=gReadSpeed();
  ratio=fSpeed/gSpeed;
  while(1){
  dir=0;
  ratio=mReadSpeed()*2/gReadSpeed();
  while(ratio>1){
    ratio=mReadSpeed()/gReadSpeed();
    motStep(dir);
  }
  printVals();
  dir=1;
  ratio=mReadSpeed()*2/gReadSpeed();
  while(ratio<1){
    ratio=mReadSpeed()/gReadSpeed();
    motStep(dir);
  }
  printVals();
  }
}

void goCenter(){
  busyFlag=1;
  checkMSpd();
  fSpeed=mReadSpeed();
  gSpeed=gReadSpeed();
  float ratio=fSpeed/gSpeed;
  dir=1;
  while(ratio>1){
    ratio=mReadSpeed()/gReadSpeed();
    motStep(dir);
    checkAction(0);
    checkCruise();
    checkPrint();
  }
  dir=0;
  while(ratio<1){
    ratio=mReadSpeed()/gReadSpeed();
    motStep(dir);
    checkAction(0);
    checkCruise();
    checkPrint();
  }
  beltPos=600;
  busyFlag=0;
}

void goHome(){
  busyFlag=1;
  checkMSpd();
  int goSteps=beltPos;
  for(int i=0;i<goSteps;i++){
    motStep(0);
    checkAction(0);
    checkCruise();
    checkPrint();
  }
  beltPos=0;
  busyFlag=0;
}

void cycle(){
  for(int i=0;i<1;i++){
    goHome();
    goEnd();
  }
}

void goEnd(){
  busyFlag=1;
  checkMSpd();
  int goSteps=max(maxSteps-beltPos,0);
  for(int i=0;i<goSteps;i++){
    motStep(1);
    checkAction(0);
    checkCruise();
    checkPrint();
  }
  beltPos=1200;
  busyFlag=0;
}
  
void cycleWorm(){
  checkMSpd();
 dir=!dir;
  for (int i=0;i<1200;i++)
  {
    motStep(dir);
//    Serial.print("Motor Speed: ");Serial.print(mReadSpeed());Serial.print(" Gen Speed: ");Serial.print(gReadSpeed());Serial.print(" ratio: ");Serial.println(mReadSpeed()/gReadSpeed());
  } 
}
void simpleDelay(){
  for(int k=0;k<500;k++){
       Serial.print("Motor Speed: ");Serial.print(mReadSpeed());Serial.print(" Gen Speed: ");Serial.println(gReadSpeed());
  }
}

void readNPrint(){
  digitalWrite(motPWM_PIN,HIGH);
  while(1){
    printVals();
   // delay(100);
  }
}

float mReadSpeed(){
  if (newPulseDurationAvailable){
    newPulseDurationAvailable=false;
    fSpeed=60.0/(pulseDuration/1000000.0);
    if(micros()-pTime>=750000){fSpeed=0;}
  }
  long test=micros()-pTime;
  if(test>=1500000){
    fSpeed=0;
  }
  fSpeed=fSpeed*onFlag;
  return(fSpeed);
}

float gReadSpeed(){
  if (gNewPulseDurationAvailable){
    gNewPulseDurationAvailable=false;
    gSpeed=60.0/(gPulseDuration/1000000.0);
  }
  long test=micros()-gPTime;
  if(test>=1500000){
    gSpeed=0;
  }
  gSpeed=gSpeed*onFlag;
  return(gSpeed);
}

void motStep(int dir){
  if(dir==0)  // move right
  {
     for (int i=0;i<8;i++){
       PORTD = fdArrs[i]|B00001100; 
       delayMicroseconds(sSpeed);
     } 
     beltPos--;
   }
  if(dir==1) // move left
  {
     for (int i=0;i<8;i++){
       int j=7-i;
       PORTD = fdArrs[j]|B00001100; 
       delayMicroseconds(sSpeed);
     } 
     beltPos++;
   }
   stopMot();
}

void stopMot(){
    PORTD = B00000000|B00001100;  // this stops the motor 
}

void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) {
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Serial.println(Wire.endTransmission());
  delay(20);
}

void readEEPROM(int deviceaddress, unsigned int eeaddress ) 
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Serial.println(Wire.endTransmission());
  delay(5);
  Wire.requestFrom(deviceaddress, 1);
  if (Wire.available() > 0) 
  {
    Serial.println(Wire.read(), HEX);
  } 
  else{
    Serial.println("NO DATA!");
  }
}
