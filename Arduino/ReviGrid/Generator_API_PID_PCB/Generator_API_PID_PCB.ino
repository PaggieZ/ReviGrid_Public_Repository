// This sketch is for the ProtoGrid Generator Model
// Written by David Wilson
// Intern: Peiqi Zhu
// May, 2023

#include <MedianFilterLib.h>
#include "PID_RT.h"
#include "InterpolationLib.h"
#include <EEPROM.h>

PID_RT PID;

String serialReceived;
const byte argArrSize = 4;
String argArr[argArrSize];
byte numOfArgAvail = 0; // number of arguments available
const int timeoutDur = 2000;  // timeout duration for serial communication

const byte motPWM_PIN = 3;  // PWM output pin for steam turbine motor // Pin 5 for versions before GUB
const byte genPWM_PIN = 5;  // PWM output pint for generator motor // Pin 3 for versions before GUB
const byte redPIN =9; // Pin 11 for versions before GUB
const byte greenPIN = 10; 
const byte bluePIN = 11;  // Pin 9 for versions before GUB
// AIChan -> genVoltPin
const byte genVoltPin = A5; // This analog input channel reads the generator voltage
// AIChan6 -> genVoltDropPin
const byte genVoltLowPin = A6; // reads the lower voltage from the other side of 47 ohm resistor

// cubic spline values
double xValues[50] = {0}; // milliWatt
double yValues[50] = {0}; // PWM

byte maxLum=50; // max lumen for r, g, and b pins
bool redFlag=0; // red light on or off
bool mixFlag=0; // indicate not in mixOn mode
bool lTog=0;    // toggle flag for red light blink

int r=0; // red light lumen
int b=0; // blue light lumen
int g=0; // green light lumen

int nReads=10; // window size of median filter
float maxGen=0; // max voltage the generator can produce
bool genFlag=0; // generator on or off

float vLev=0; // voltage level scaled to 5 volts dc
float genI=0; // gen current in milliamp
float kwVal=0; // gen power calculated from vLev(voltage) and genI(current)
int kwAlloc = 0;

float vD=0; // voltage drop scaled to 5 volts dc
float svLev=0; // voltage level scaled to "vScale" volts dc

int maxPow = 0; // max milliWatt
long capKW=200; // full KW capacity of generator
float vScale=30; // voltage scale factor
float setV=120;
int kwV=100; // load allocated to generator
const float carbDiv=100.0; // divisor for calculating carbon

float Kp=1.5;
float Ki=10;
float Kd=0.001;

bool pMatch=0; // PID output is previousely matched with motPWM_PIN, generator kick start
bool matchFlag=0; // match mode is on

float loadRes=0;

long pTime = 0; // start time for calculating new PID interval

byte motPWM=0;
byte genPWM=0;
int genV=0;
int genVL=0;

long runtime = 0;
long delTime=0;

// "setVScale", "setVolts>1",
char *commands[] = {"init","runRange", "trackOn","trackOff",
                    "setLoad>1","setMot>1","setKp>1","setKi>1","setKd>1","setR>1","setG>1","setB>1", "setPWM>2",
                    "motOn","motOff",
                    "getVolts<1", "getBV<2",
                    "getVal<1","getKW<1","getCarbon<1","getAll<7","getRes<1","getDrop<1","getCurrent<1", 
                    "off","eoc"};


MedianFilter<float> genVoltMF(nReads); // generator voltage median filter
MedianFilter<float> genVoltLowMF(nReads); // generator voltage Low median filter

void setup() {
  Serial.begin(115200);
  pinMode(redPIN,OUTPUT);  // red LED
  pinMode(greenPIN,OUTPUT); // green LED
  pinMode(bluePIN,OUTPUT); // blue LED

  pinMode(genPWM_PIN,OUTPUT);   // generator load control
  pinMode(motPWM_PIN,OUTPUT);   //motor on/off

  PID.setPoint(setV); // PID setpoint
  PID.setOutputRange(0, 255);  // PID output range is PWM range
  // interval is in milli sec, used for PID intergral and derivative
  // interval is updated before every compute() in checkMatch()
  PID.setInterval(30); // time interval between 2 compute() calls
  PID.setK(Kp, Ki, Kd);
  PID.start(); // enable PID controller to compute() new output values
  digitalWrite(motPWM_PIN,LOW); // turn off steam turbine
  genFlag=0; // turn off generator

  readDefaultRange();
  
  // EEPROM.write(99,255);
  // setup cubic spline arrays
  byte val = EEPROM.read(99);
  if(val == 255) { // if eeprom[99] is not previousely set
    initDev();
    storeRangeVals();
    delay(1000);  
    loadLoop();
  }
  loadCubic();
}

void loop() {
  checkAction(); // check user cmd
  runtime = micros();
  checkGenStat(); // update RGB LED based on generator status
  checkMatch(); // check if target voltage == curr voltage 
  // delay(100);
 }

void checkAction() {
  if (enterSerialInput()) {
    if(serialReceived=="getAll"){getAll();}
    else if(serialReceived=="*ID?"){Serial.println("generator");}
    else if(serialReceived=="*IDN?"){Serial.println("generator");}
    else if(serialReceived=="init"){initDev();}   
    else if(serialReceived=="getRes"){getRes();}    
    else if(serialReceived=="setLoad"){setLoad();}
    else if(serialReceived=="setMot"){setMot();}
    else if(serialReceived=="getVolts"){getVolts();} // returns the actual voltage of the circuit
    else if(serialReceived=="getDrop"){getDrop();}
    else if(serialReceived=="getBV"){getBV();} // returns both genVolt and genVoltLow    
    else if(serialReceived=="getCurrent"){getCurrent();}
    else if(serialReceived=="getVal"){getVal();}
    // else if(serialReceived=="setVolts"){setVolts();}
    else if(serialReceived=="getKW"){getKW();}
    // else if(serialReceived=="getPower"){getPower();} // same as getKW()
    else if(serialReceived=="getCarbon"){getCarbon();}
    else if(serialReceived=="motOn"){motOn();}
    else if(serialReceived=="motOff"){motOff();}
    else if(serialReceived=="runRange"){runRange();}
    else if(serialReceived=="trackOn"){matchFlag=1;}    
    else if(serialReceived=="trackOff"){matchFlag=0;}  
    else if(serialReceived=="setKp"){setKp();} 
    else if(serialReceived=="setKi"){setKi();} 
    else if(serialReceived=="setKd"){setKd();}  
    else if(serialReceived=="setPWM"){setPWM();} 
    // else if(serialReceived=="setVScale"){setVScale();}
    else if(serialReceived=="off"){off();}    
    else if(serialReceived=="getCommands"){getCommands();}
    else if(serialReceived=="rapidOn"){rapidMode();}
    else if(serialReceived=="getRuntime"){getMicroRuntime();}
    else if(serialReceived=="runCal"){runCal();}
    else if(serialReceived=="getRaw"){getRaw();}   
    else if(serialReceived=="setRLoad"){setRLoad();} 
  }
}

void getAll(){
   readGenVolt();
   Serial.println(capKW);  // this is the full KW capacity of the generator
   Serial.println(kwVal);  // this is the current power output
   Serial.println(kwAlloc);  // this is the load allocated to the generator   
   Serial.println(max((kwVal-kwAlloc),0)); // this is the difference between output and the allocated load
   float carbVal=0;if(kwVal>5){carbVal=kwVal/carbDiv;}
   Serial.println(carbVal); 
   Serial.println(0);  // this is not a renewable energy source
   Serial.println(svLev); // this is the line voltage
}


// initialize the unit
void initDev(){
  runRange(); // calculate maxGen and capKW
  digitalWrite(motPWM_PIN,LOW); // turn motor off
  genFlag=0; // turn generator off
}

void runCal(){
  initDev();
  storeRangeVals();
  delay(1000);  
  loadLoop();
  loadCubic();
}

/*************************** clearArgArr *****************************/
/*  The clearArgArr() function sets all elements in argArr[] to NULL.
    Global Variable(s):
      argArr[]: all elements are set to NULL.
*/
void clearArgArr() {
  for (int i = 0; i < argArrSize; i++) {
    argArr[i] = "";
  }
} // end, clearArgArr()

/*************************** enterSerialInput *****************************/
/*  The enterSerialInput() function checks if a new command input is availeble and 
    stores the received string arguments into the argArr[] array.
    Global Variable(s):  
      serialReceived: stores cmd name.
      numOfArgAvail: number of argument received from the serial input.
      argArr[]: string arguments received.
    Return:
      0 - doesn't receive a new command.
      1 - receives a new command.
*/
byte enterSerialInput() {
  numOfArgAvail = 0;
  // return when no serial input is available
  if (Serial.available() <= 0) {
    return 0;
  }

  serialReceived = Serial.readStringUntil('\n'); // read serial input
  serialReceived.trim(); // remove leading and trailing whitespaces
  int spacePos = serialReceived.indexOf(' '); 
  // return when no whitespace is found in the string
  if (spacePos == -1) { 
    // serial input doesn't contain any argument
    return 1;
  }

  clearArgArr();
  // "remainingStr" stores arguments
  String remainingStr = serialReceived.substring(spacePos + 1);
  // "serialReceived" stores cmd name
  serialReceived = serialReceived.substring(0, spacePos);
  spacePos = remainingStr.indexOf(' ');

  while (spacePos != -1) { // while there are more than one arguments in the "remainingStr"
    argArr[numOfArgAvail] = remainingStr.substring(0, spacePos); // store an argument in the argArr[]
    remainingStr = remainingStr.substring(spacePos + 1); 
    numOfArgAvail++;
    spacePos = remainingStr.indexOf(' ');
  }
  argArr[numOfArgAvail] = remainingStr;
  numOfArgAvail++;
  
  // for (int i = 0; i < numOfArgAvail; i++) {
  //   Serial.println(argArr[i]);
  // }

  return 1;
} // end, enterSerialInput()


/*************************** readSerTO *****************************/
/*  The readSerTO() function reads command arguments and stores the 
    received string arguments into the argArr[] array. 
    Input(s):
      argCount: number of arguments the readSerTO() function needs to read.
    Global Variable(s):  
      argArr[]: string arguments received.
      numOfArgAvail: number of arguments received.
*/
void readSerTO(int argCount) {
  numOfArgAvail = 0;
  clearArgArr();
  long time = millis();
  for (int i = 0; i < argCount; i++) {
    while (Serial.available() < 1 && millis() - time < timeoutDur) {} // wait for user input
    if (millis() - time < timeoutDur) { // if no timeout
      argArr[numOfArgAvail] = Serial.readStringUntil('\n');
      numOfArgAvail++;
    } else { // timeout
      return; 
    }
  } // end, for
} // end, readSerTO()

void getRaw(){
  Serial.println(analogRead(5));
}

// returns the worst case runtime of the generator module
// for test purpose only
void getMicroRuntime() {
  long t = micros() - runtime;
  Serial.println(String(t));
}

// function for high speed send and receive of integers
// used for fast control applications
void rapidMode()
{
  byte hiByte=0;
  byte loByte=0;
  byte stayByte=1;
  motPWM=0;
  genPWM=0;
  analogWrite(redPIN,0);
  analogWrite(greenPIN,255);
  analogWrite(bluePIN,0);
  while(stayByte==1|stayByte==49)
  {
      while (Serial.available()<3){}
      stayByte=Serial.read();
      motPWM = Serial.read();
      genPWM = Serial.read();
      analogWrite(motPWM_PIN,motPWM);
      analogWrite(genPWM_PIN,genPWM);
      for(int i=0;i<10;i++){
        genV = genVoltMF.AddValue(analogRead(genVoltPin)); 
        genVL = genVoltLowMF.AddValue(analogRead(genVoltLowPin));   
      }
//      genV=analogRead(genVoltPin);
//      genVL=analogRead(genVoltLowPin);
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

void setPWM(){
  int numOfArg = 2;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  int mot_PWM=argArr[0].toInt();
  int gen_PWM=argArr[1].toInt();
  mot_PWM=max(min(255,mot_PWM),0);
  gen_PWM=max(min(255,gen_PWM),0);
  analogWrite(motPWM_PIN,mot_PWM);
  analogWrite(genPWM_PIN,gen_PWM);
}


// send out all available commands
void getCommands(){
  byte currCmdIdx = 0; // current cmd index
  char *currCmd = commands[0];

  while(currCmd != "eoc") { // while the current cmd is not "eoc"
    while(Serial.availableForWrite() < strlen(commands[currCmdIdx])){delayMicroseconds(100);}
    Serial.println(commands[currCmdIdx]); // send out the current cmd
    currCmdIdx++;
    currCmd = commands[currCmdIdx];
  }
  Serial.println("eoc");
}

// loads cubic spline arrays from eeprom
void loadCubic(){
  maxPow = 0;
  for(int u = 0; u < 50; u++) { 
    xValues[u] = EEPROM.read(u + 100);
    yValues[u] = u;
    if(xValues[u]>maxPow){maxPow=xValues[u];}
    // Serial.println(maxPow);
  }
}

// turn the genPWM_PIN from 0 to 49 to examine the milliwatt values
// use PID to maintain a stable voltage in this process
// store the milliwatt values to eeprom for cubic spline calculation
void loadLoop(){
  initDev(); // initialize the unit
  matchFlag = 1; // enable match mode for PID
  for(int w = 0; w < 40; w++) { // run checkMatch to reach setpoint voltage
    checkMatch();
    delay(100);
  }
  // calculate milliwatt values with 50 different PWM values
  for(int u = 0; u < 50; u++) {
    analogWrite(genPWM_PIN,u); // modify 
    checkAction();
    checkGenStat();
    checkMatch();
    delay(100);
    readGenVolt(); // calculate current
    // power = voltage * current
    byte eVal=vLev*genI; // milliwatt
    EEPROM.write(u+100,eVal); // write milliwatt values to eeprom
    // Serial.print(u); Serial.print(" "); Serial.println(eVal);
  }
  EEPROM.write(99, 0); // set eeprom[99]
  matchFlag = 0; // disable match mode
  analogWrite(genPWM_PIN,0); // set load PWM to 0
}

void storeRangeVals()
{
  int iCapKW=capKW;
  int iMaxGen=maxGen;
  byte loCapKW=(byte)(iCapKW & 0xFF);
  byte hiCapKW=(byte)(iCapKW>>8);
  byte loMaxGen=(byte)(iMaxGen & 0xFF);
  byte hiMaxGen=(byte)(iMaxGen>>8);
  EEPROM.write(50,loCapKW);
  EEPROM.write(51,hiCapKW);
  EEPROM.write(52,loMaxGen);
  EEPROM.write(53,hiMaxGen);
}

void readDefaultRange()
{
  byte loCapKW=(byte)EEPROM.read(50);
  byte hiCapKW=(byte)EEPROM.read(51);
  byte loMaxGen=(byte)EEPROM.read(52);
  byte hiMaxGen=(byte)EEPROM.read(53);
  capKW=(unsigned int)(hiCapKW<<8) | loCapKW;
  maxGen=(unsigned int) (hiMaxGen<<8) | loMaxGen;
}

// turn off the unit
void off(){
  matchFlag=0; // stop track mode
  analogWrite(motPWM_PIN,0);
  analogWrite(genPWM_PIN,0);
  genFlag=0;
 }

// target voltage is "setV", 120V
// adjusts motPWM_PIN using PID control to maintain a theoretical generator voltage of 120V
void checkMatch(){
  if(matchFlag==1){ // match mode is on
    // calculate PID interval
    float nuTime = millis()-pTime; 
    if(nuTime > 3000) {
      nuTime = 30;
    }
    float curVolts = readGenVolt(); // read current voltage     
    // kick-starts the steam turbine motor if it was not on before
    if(pMatch==0){ 
      digitalWrite(motPWM_PIN,HIGH);
      delay(500);
    }        
    if (PID.compute(curVolts)) { // send curr voltage to PID 
      PID.setInterval(nuTime); // set new PID interval
      int op = PID.getOutput(); // get computed output
      analogWrite(motPWM_PIN, op); // adjust motor pin
    }
    genFlag=1; 
    pMatch=1;

  } else if(pMatch==1 && matchFlag==0) { // match mode is off and motor is on
    pMatch=0; 
    analogWrite(motPWM_PIN,0);             // this shuts down the steam turbine motor
    genFlag=0; 
  }
  pTime = millis();
}

// returns the scaled generator voltage level
void getVal(){
  float genLevel = readGenVolt();
  if (genLevel <= 0) {
    Serial.print("000");
  } else if (genLevel <= 10) {
    Serial.print("00");
  } else if (genLevel <= 100) {
    Serial.print("0");
  }
  Serial.println(genLevel);
}

// returns the actual voltage level and voltage drop
void getBV(){
  readGenVolt();
  Serial.println(vLev);
  Serial.println(vD);
}

// returns the actual voltage level
void getVolts(){
  readGenVolt();
  Serial.println(vLev);
}

// returns the actual voltage drop
void getDrop(){
  readGenVolt();
  Serial.println(vD);
}

// returns the actual current in milliamp
void getCurrent(){
  readGenVolt();
  Serial.println(genI);
}

void setKp(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  Kp=argArr[0].toFloat();
  Kp = max(min(1000, Kp), 0);
  
  PID.setK(Kp, Ki, Kd);
}

void setKi(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  Ki=argArr[0].toFloat();
  Ki = max(min(1000, Ki), 0);
  
  PID.setK(Kp, Ki, Kd);
}

void setKd(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  Kd=argArr[0].toFloat();
  Kd = max(min(1000, Kd), 0);
  
  PID.setK(Kp, Ki, Kd);
}

void getKW(){
  readGenVolt();
  Serial.println(kwVal);
}

void getCarbon(){
  readGenVolt();
  float carbVal=0;if(kwVal>5){carbVal=kwVal/carbDiv;}
  Serial.println(carbVal);
}

// updates RGB LED based on generator voltage
void checkGenStat(){
  if(!genFlag){ // if generator is off
    // blink red light
//    if(redFlag==0){analogWrite(redPIN,maxLum);redFlag=1;}
//    else{analogWrite(redPIN,0);redFlag=0;}
//    delay(500);
    delTime=millis()%1000;
    // if(delTime<500){digitalWrite(redPIN,1);}else{digitalWrite(redPIN,0);}  
    bool pTog = lTog;    
    lTog=(delTime<500) ? true: false;          
    if(pTog!=lTog){lightsOff();digitalWrite(redPIN,lTog);} 
  } 
  else { // generator is on
      readNLite(); // update LED
  } 
}

void motOn(){
  analogWrite(motPWM_PIN,255);
  genFlag=1;
}

void motOff(){
  analogWrite(motPWM_PIN,0);
  genFlag=0;
}


void lightsOff(){
  digitalWrite(redPIN,LOW);
  digitalWrite(greenPIN,LOW);
  digitalWrite(bluePIN,LOW);
}

void readNLite(){
  if(genFlag==1){
    readGenVolt();
    mixColor(kwVal); // update r, g, b variable
    analogWrite(redPIN,r);
    analogWrite(greenPIN,g);
    analogWrite(bluePIN,b);
  }
}

// set motor speed
void setMot(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  int motLev=argArr[0].toInt();
  // set user input 
  motLev=min(255,max(motLev,0));  
  analogWrite(motPWM_PIN,motLev);
  genFlag=0;
  if(motLev>0){genFlag=1;}
  
}

// set the generator load
void setLoad(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  kwVal=argArr[0].toInt();
  kwAlloc = kwVal;
  kwVal=max(min(1000,kwVal),0); // kwVal range is 0 to 1GW
  float mwVal=kwVal/1000.0 * maxPow; 
  int loadLev=Interpolation::ConstrainedSpline(xValues, yValues, 50, mwVal); // convert milliWatt to PWM
  analogWrite(genPWM_PIN,loadLev);
}

void setRLoad(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  float mwVal=argArr[0].toInt();
  analogWrite(genPWM_PIN,mwVal);
}

// get load resistance
void getRes(){
  Serial.println(loadRes);
}

// same as getKW()
// void getPower(){
//   readGenVolt();
//   Serial.println(kwVal); 
// }


// calculate the current voltage, voltage drop, current and power
// by reading genVoltPin and genVoltDropPin
float readGenVolt(){
  float genVolt = 0; // median reading from genVoltPin
  float genVoltLow = 0; // median reading from genVoltLowPin

  for (int s=0;s<nReads;s++){ // get genVolt and genVoltLow
    genVolt = genVoltMF.AddValue(analogRead(genVoltPin)); 
    genVoltLow = genVoltLowMF.AddValue(analogRead(genVoltLowPin));     
    delay(2);
  }
  
  // voltage level scaled to 5 volts dc
  vLev=(genVolt / maxGen) * 5;          // scales generator counts to 5 volts dc
  // voltage drop scaled to 5 volts dc
  vD=((max(genVolt - genVoltLow, 0))/1023.0)*5.0;  // this reads the voltage drop across the 47 ohm resistor
  // voltage level scaled to "vScale" volts dc
  svLev=vLev*vScale;                  // this scales generator voltage to the representative 120 volts
  // ohm's law: current  = voltage / resistance
  // get current in milliamp
  genI=vD/47.0*1000;              // this calculates the current flowing through the 47 ohm resistor
  // power = voltage * current
  // get power in milliwatt
  float mwV=vLev*genI;                    // this calculates the milliwatt value
  // resistance = voltage / current
  // load resistance includes the 47 ohm resistor
  loadRes=(vLev/genI*1000.0); // load resistance
  kwVal=mwV/maxPow*1000.0;
  return(svLev);
}

// calculate the max generator voltage (maxGen)
// calculate the max power (capKW) can be generated from power plant
void runRange(){
  digitalWrite(motPWM_PIN,HIGH);  // max motor, spin motor in max speed
  delay(2000);
  maxGen=analogRead(genVoltPin); 
  digitalWrite(genPWM_PIN,HIGH);    // max load, shortout generator
  delay(1000);
  readGenVolt(); // read generator voltage with max motor and max load
                 // updates kwVal
  digitalWrite(motPWM_PIN,LOW);
  digitalWrite(genPWM_PIN,LOW);
  capKW = min(1000, kwVal); // most power the generator can output with max motor and max load
 }

// blend RGB led based on readVal
// update the r, g, and b variables
// rdVal -> readVal
void mixColor(float readVal){ 
  // map readVal on scale of 0 to 100
  // percent value
  int pctVal=max(min(readVal/capKW*100,100),0);
  if(pctVal >= 75){
    b=maxLum;
    g=maxLum*(pctVal-75)/25;
    r=maxLum;
  }
  else if(pctVal < 75 && pctVal >= 50){
    b=maxLum*(pctVal-50)/25;
    g=0;
    r=maxLum;
  }
  else if(pctVal < 50 && pctVal >= 25){
    b=0;
    g=maxLum*(50-pctVal)/25;
    r=maxLum*(pctVal-25)/25;
  }
  else if(pctVal < 25 && pctVal >= 0){
    b=maxLum*(25-pctVal)/25;
    g=maxLum*pctVal/25;
    r=0;   
  }
} 



// void setVolts(){
//   long prevTime=millis();
//   while(Serial.available()<1&millis()-prevTime<2000){}
//   if(millis()-prevTime<2000){
//     serialReceived = Serial.readStringUntil('\n');
//     setV=serialReceived.toInt();
//     setV=max(min(220,setV),0);
//   }
// }


// void setVScale(){
//   long prevTime=millis();
//   while(Serial.available()<1&millis()-prevTime<2000){}
//   if(millis()-prevTime<2000){
//     serialReceived = Serial.readStringUntil('\n');
//     vScale=serialReceived.toInt();
//     vScale=max(min(50,vScale),0);
//   }
// }