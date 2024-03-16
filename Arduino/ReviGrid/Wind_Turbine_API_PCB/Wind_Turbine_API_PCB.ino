// This sketch is for the ProtoGrid Wind Turbine Model
// Written by David Wilson
// Intern: Peiqi Zhu
// May, 2023

#include <MedianFilterLib.h>
#include <EEPROM.h> // [0] stores fdFlag, [1] stores MAX_PAN_POS flag, [2] + 200 = MAX_PAN_POS

// the following arrays set the patterns to energize the stepper motor. 
// there are 2 arrays because some motors are reversed
// once the direction is determined automatically, the fdFlag is stored in eeprom[0]

// the bit used for wind hall sensor must be 1
// windHallPin = 2
//char fdArrs[2][8] = {{B10010100, B00010100, B00110100, B00100100, B01100100, B01000100, B11000100, B10000100}, 
//                     {B10000100, B11000100, B01000100, B01100100, B00100100, B00110100, B00010100, B10010100}};

// the "1" in the 4 bit position is to hold the pullup resistor high on pin D3 when doing PORT writes for the stepper motor
// windHallPin = 3
// char fdArrs[2][8] = {{B10011000, B00011000, B00111000, B00101000, B01101000, B01001000, B11001000, B10001000}, 
//                     {B10001000, B11001000, B01001000, B01101000, B00101000, B00111000, B00011000, B10011000}};

char fdArrs[2][8] = {{B10000000, B11000000, B01000000, B01100000, B00100000, B00110000, B00010000, B10010000}, 
                     {B10010000, B00010000, B00110000, B00100000, B01100000, B01000000, B11000000, B10000000}};


byte fdFlag = 0; // indicate which of the two arrays is used for direction

String serialReceived;
const byte argArrSize = 4;
String argArr[argArrSize];
byte numOfArgAvail = 0; // number of arguments available
const int timeoutDur = 2000;  // timeout duration for serial communication

byte windHallPin = 2; // pin 2 for new version GUB v1.1, else use pin 3
byte homeHallPin = 12;
int nReads=20; // readCount, window size of median filter

long panPos=0; // pan position of stepper motor
long tPanPos=0;
int MAX_PAN_POS = 240;
int nSteps=25; // step size for lookCW and lookCCW

unsigned long mSpeed=1200; // motor speed, delay duration in-between motor steps 
int maxIndex=0; // pan position with the max wind level
unsigned long mRange = MAX_PAN_POS; // max pan position the stepper motor can go
float fSpeed=0; // wind speed

unsigned long delayTime=2000; // getDelay() duration
unsigned int mIncr=20; // step size for runScan()

bool atFlag=0; // auto track flag, track mode on or off

int loadVal=0;
bool firstIntr=1; // first wind interrupt or not
unsigned long pTime=0; // last time speed is calculated
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = pulseInTimeBegin;
volatile bool newPulseDurationAvailable = false;
bool onFlag=1; // wind turbine on or off
bool busyFlag=0;



char *commands[] = {"init","runScan","trackOn","trackOff",
                    "goHome","goL","goF","goR","moveCW>1","moveCCW>1","moveCWR<2","moveCCWR<2","goMax",
                    "lookCW","lookCCW",
                    "setLoad>1","setRange>1","setDelay>1","setSteps>1",
                    "getMax<1","getVal<1","getKW<1","getAll<7","getCarbon<1","getPos<1","getBusy", "getMaxPos<1",
                    "on","off","eoc"};

MedianFilter<float> medianFilter(nReads);

// interrupt service routine when windHallPin value changed
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
void windHallPinInterrupt()
{
  if (digitalRead(windHallPin) == LOW) { // sensor sensed magnet
    // start measuring
    pulseInTimeBegin = micros();
  }
  else { // magnet not sensed
    // stop measuring
    pulseInTimeEnd = micros();

    if(firstIntr) { // ignore first interrupt
      firstIntr=0;
      return;
    }

    newPulseDurationAvailable = true;
  }
 
}

void setup() {
  Serial.begin(115200); // Starting serial communication
  DDRD = DDRD | B11110000;              // this is safer as it sets pins 4 to 7 as outputs and 0 to 3 as inputs 
  // sets windHallPin as PULL_UP, the windHallPin bit must be written high whenever port D is written to keep pullup connected
  byte pullUpVal = pow(2, windHallPin);
  PORTD = PORTD | pullUpVal;    
  // PORTD = PORTD | B00001000;          

  pinMode(homeHallPin,INPUT_PULLUP);             // for home hall sensor
  attachInterrupt(digitalPinToInterrupt(windHallPin),windHallPinInterrupt,CHANGE);      // for propeller blade input
 
  // EEPROM.write(0, 255); // erase fdFlag routine result
  // EEPROM.write(1, 255); // erase findMaxPanPos routine result

  // sets fdArrs based on windHallPin number
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 8; j++) {
      fdArrs[i][j] = fdArrs[i][j] + pullUpVal;
    }
  }

  // set fdFlag
  byte val = EEPROM.read(0); // read eeprom[0]
  // Serial.println(val);
  if(val == 255) { // if eeprom[0] is not previousely set
    // 255 (0xFF) is default value
    setReverse(); // set direction
  } else { // eeprom[0] has been modified
    fdFlag = val; // set fdFlag to the previousely determined direction array index
  }

  // Serial.print("fdFlag: "); Serial.println(fdFlag);

  // set MAX_PAN_POS
  val = EEPROM.read(1);
  if (val == 255) { // the unit has not yet executed the find MAX_PAN_POS routine
    findMaxPanPos();
  }
  MAX_PAN_POS = EEPROM.read(2) + 200;
  mRange = MAX_PAN_POS;
  // Serial.print("fdFlag: "); Serial.println(fdFlag);
  // Serial.print("mRange"); Serial.println(mRange);
}

void loop() { 
  checkAction(1);
}

// returns maximum position that the solar panel can rotate to
void getMaxPos() {
  Serial.println(mRange);
}


void findMaxPanPos() {
  byte sensorVal = digitalRead(homeHallPin);
  if (sensorVal == 0) { // move if is at home position already
    moveAway();
  }
  goHome();

  // rotate to the max position
  for (int i = 0; i < MAX_PAN_POS; i++) {
    stepCW();
  }

  // rotate backward and increase step count until home
  sensorVal = digitalRead(homeHallPin);
  int stepCount = 0;
  while (sensorVal == 1) {
    stepCCW();
    stepCount = stepCount + 1;
    sensorVal = digitalRead(homeHallPin);
    // Serial.println(stepCount);
    // Serial.println(sensorVal);
  }
  EEPROM.write(2, stepCount - 200); // store MAX_PAN_POS in EEPROM
  EEPROM.write(1, 0); // set the find routine flag

}


// move the solar panel away from home position even with wrong direction array
// used in setReverse()
void moveAway() {
  // step clockwise 40 times
  for (byte i = 0; i < 60; i++) {
    stepCW(); 
  }
  // step counter-clockwise 20 times
  for (byte i = 0; i < 40; i++) {
    stepCCW();
  }
}

// set motor direction
// set fdFlag and eeprom[0] with the correct direction array index
void setReverse() {
  byte sensorVal = digitalRead(homeHallPin);
  if (sensorVal == 0) { // move if is at home position already
    moveAway();
  }
  fdFlag = 0;
  goHome(); // go home with fdArrs[0][]
  sensorVal = digitalRead(homeHallPin);
  if (sensorVal == 0) { // return if is at home position
    EEPROM.write(0, fdFlag); // update eeprom[0]
    return;
  }
  // use another array
  fdFlag = 1;
  goHome(); // go home with fdArrs[1][]
  sensorVal = digitalRead(homeHallPin);
  if (sensorVal == 0) { // return if is at home position
    EEPROM.write(0, fdFlag); // update eeprom[0]
    return;
  }
  // error message
}

// initialize unit
void initDev(){
  on();
  goHome();
  goF(); // go to front
}


void off(){
  atFlag=0;
  goHome();
  goF();
  stopMot();
  onFlag=0;
}

void on(){
  onFlag=1;
}

float readSpeed(){
  if (newPulseDurationAvailable){ // magnet finished entering and leaving sensor
    newPulseDurationAvailable = false;
    unsigned long pulseDuration = pulseInTimeEnd - pulseInTimeBegin;
    // calculate speed if start time and end time are valid
    if(pulseInTimeEnd > pulseInTimeBegin){ 
      fSpeed=7000000.0/pulseDuration;
    }
    pTime=micros();
  } else if(micros()-pTime > 750000){// fSpeed is 0 when no new pulse duration is available in the last 0.75 sec
    fSpeed=0;
  }
  // fSpeed is 0 if wind turbine is not on
  fSpeed=fSpeed*onFlag;
  return(fSpeed);
}
  

void checkAction(bool checkAll) {
  if (enterSerialInput() == 0) { // return when no user input
    return;
  }
  if(serialReceived=="getAll"){getAll();}
  else if(serialReceived=="*ID?"){Serial.println("windturbine");}
  else if(serialReceived=="trackOff"){atFlag=0;busyFlag=0;}
  else if(serialReceived=="getMax"){getMax();}
  else if(serialReceived=="getVal"){getVal();}
  else if(serialReceived=="getKW"){getKW();}
  else if(serialReceived=="getCarbon"){getCarbon();}
  else if(serialReceived=="getBusy"){getBusy();}  
  else if(serialReceived=="setLoad"){setLoad();}
  else if(serialReceived=="setRange"){setRange();}
  else if(serialReceived=="setDelay"){setDelay();}
  else if(serialReceived=="getPos"){getPos();}
  else if(serialReceived=="getMaxPos"){getMaxPos();}
  else if(serialReceived=="setSteps"){setSteps();}   
  else if(serialReceived=="off"){off();}  
  else if(serialReceived=="on"){on();}

  if(checkAll == 0) {
    return;
  }

  if(serialReceived=="init"){initDev();}
  else if(serialReceived=="moveCW"){moveCW();}
  else if(serialReceived=="moveCCW"){moveCCW();}
  else if(serialReceived=="moveCWR"){moveCWR();}
  else if(serialReceived=="moveCCWR"){moveCCWR();}
  else if(serialReceived=="goHome"){goHome();}
  else if(serialReceived=="goL"){goHome();}
  else if(serialReceived=="goF"){goF();}
  else if(serialReceived=="goR"){goR();}
  else if(serialReceived=="runScan"){runScan();}
  else if(serialReceived=="goMax"){goMax();}
  else if(serialReceived=="lookCW"){lookCW();}
  else if(serialReceived=="lookCCW"){lookCCW();}
  else if(serialReceived=="trackOn"){atFlag=1;busyFlag=1;runTrack();}
  else if(serialReceived=="getCommands"){getCommands();}
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

// send out all available commands
void getCommands(){
  byte currCmdIdx = 0; // current cmd index
  char *currCmd = commands[0];

  while(currCmd != "eoc") { // while the current cmd is not "eoc"
    Serial.println(commands[currCmdIdx]); // send out the current cmd
    currCmdIdx++;
    currCmd = commands[currCmdIdx];
  }
  Serial.println("eoc");
}

// execute track mode 
void runTrack(){
  while(atFlag==1){
    lookCW();
    getDelay(delayTime);
    lookCCW();
    getDelay(delayTime);
  }
}

// scan 180 degree and find panPos with strongest wind speed
void runScan(){ 
  busyFlag=1;
  goHome();
  unsigned long prevTime=millis();
  while(readSpeed() > 10 && millis() - prevTime < 5000) {checkAction(0); delay(100);}
  float maxPower = 0;
  maxIndex = 0; // postiiton with max wind speed
  float currPower = readSpeed();
  int numOfSteps = mRange / mIncr;

  for(int d = 0; d < numOfSteps; d++){ // for each step
    for(int k = 0; k < mIncr; k++){motStep(1);checkAction(0);} // move 1 step
    stopMot();
    currPower = readSpeed();
    if(currPower > maxPower){ // found new max
      maxPower = currPower; 
      maxIndex = d * mIncr;
    }
    getDelay(delayTime);    
  }
  // maxIndex=maxIndex*mIncr;
  panPos = mRange;
  goMax();
  busyFlag=atFlag;
}

void getAll(){
  float speed = readSpeed();
  Serial.println(290); // max KW of the wind turbine 
  Serial.println(speed); // this is the current wind power
  Serial.println(loadVal); // this is the load allocated to the wind turbine
  Serial.println(speed - loadVal); // this current power minus the load
  Serial.println(0); // this is the carbon value
  Serial.println(1); // this is that the wind turbine is a renewable energy source
  Serial.println(speed / 5.0); // this is the line voltage
}

void setLoad(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  loadVal=argArr[0].toInt();  
  loadVal = max(0, min(290, loadVal));
  
}

void setSteps(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  nSteps=argArr[0].toInt(); 
  nSteps = max(1, min(mRange, nSteps));
  
}


void setRange(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  mRange=argArr[0].toInt();  
  mRange = max(0, min(MAX_PAN_POS, mRange));
  
}

void setDelay(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  delayTime=argArr[0].toInt(); 
  delayTime = max(min(10000, delayTime), 0); 
  
}

void getVal(){
  fSpeed = readSpeed();
  Serial.println(fSpeed);
}

void getMax(){
  Serial.println(maxIndex);
}

void getKW(){
  fSpeed=readSpeed();
  Serial.println(max((fSpeed*5.0)-loadVal,0));
}

void getCarbon(){
  Serial.println("0");
}

void getPos(){
  Serial.println(panPos);
}

void getBusy(){
  Serial.println(busyFlag);  
}

void moveCW(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  long moveSteps=argArr[0].toInt(); 
  if(panPos + moveSteps <= mRange){
    for(int z=0; z < moveSteps; z++){
      stepCW();
      checkAction(0);
    }
  }
  
}

// move CW and Read according nSteps
void moveCWR(){
  busyFlag=1;
    if(panPos + nSteps <= mRange){       
      for(int z=0;z<nSteps;z++){
        stepCW();
        checkAction(0); // check partial cmds only
      }
    }
  stopMot();
  Serial.println(panPos);
  Serial.println(readSpeed());  
  busyFlag=atFlag;
}
 
void moveCCW(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  long moveSteps=argArr[0].toInt(); 
  if(panPos-moveSteps>=0){ 
    for(int z=0;z<moveSteps;z++){
      stepCCW();
      checkAction(0);
    }
  }
  
}

// move CCW and Read according nSteps
void moveCCWR(){
  busyFlag=1;
    if(panPos + nSteps <= mRange){       
      for(int z=0;z<nSteps;z++){
        stepCCW();
        checkAction(0); // check partial cmds only
      }
    }
  stopMot();
  Serial.println(panPos);
  Serial.println(readSpeed());  
  busyFlag=atFlag;
}

void getDelay(int mSec){
  unsigned long mTime=millis();
  while(millis()<mTime+mSec){
    checkAction(0);
  }
}

void goMax(){
  busyFlag=1;
  tPanPos=panPos;
  if(tPanPos > maxIndex){
   for(int r=0; r < (tPanPos-maxIndex); r++){
      motStep(0);
      checkAction(0);
    }   
  } else if(panPos <=maxIndex){
   for(int r=0;r<(maxIndex-tPanPos);r++){
      motStep(1);
      checkAction(0);
    }   
  }
  stopMot();
  panPos=maxIndex;
  busyFlag=atFlag;
}

void lookCCW(){
  float fPrevSpd=readSpeed(); // prev speed
  for(int i=0;i<nSteps;i++){stepCCW();checkAction(0);} // move CCW 1 step
  stopMot();
  getDelay(delayTime);
  float fSpd = readSpeed(); // curr speed 
  if (fPrevSpd > fSpd){ // move back if prev position has stronger wind
    for(int i=0;i < nSteps; i++){stepCW();checkAction(0);}
  }
  stopMot();
}

void lookCW(){
  float fPrevSpd=readSpeed(); // prev speed
  for(int i=0;i<nSteps;i++){stepCW();checkAction(0);} // move CW 1 step
  stopMot();
  getDelay(delayTime);
  float fSpd=readSpeed(); // curr speed
  if (fPrevSpd>fSpd){ // move back if prev position has stronger wind
    for(int i=0;i<nSteps;i++){stepCCW();checkAction(0);}
  }
  stopMot();
}

// go home, go left
void goHome(){
  busyFlag=1;
  int mLim = digitalRead(homeHallPin);
  unsigned long prevTime = millis();
  // keep moving CCW until home
  unsigned long watchdogDur = (mRange * 8 * mSpeed * 2 / 1000)*5;
  // Serial.print("watchDogDur: ");
  // Serial.println(watchDogDur);
  while(mLim == 1 && millis() - prevTime < watchdogDur){ 
    mLim=digitalRead(homeHallPin);
//    for (int i=0;i<8;i++){
//      PORTD = fdArrs[fdFlag][i]; 
//      delayMicroseconds(mSpeed);
//      stopMot();
//    } 
//    panPos=max(panPos-1,0);
    motStep(0);
    checkAction(0); 
  }
  stopMot();
  panPos=0;
  getDelay(500);
  busyFlag=atFlag;
}

// go front
void goF(){
  tPanPos=panPos;
  busyFlag=1;
  if(tPanPos>120){
   for(int r=0;r<(tPanPos-120);r++){
      motStep(0);
      checkAction(0);
    }   
  } else if(tPanPos<=120){
   for(int r=0;r<(120-tPanPos);r++){
      motStep(1);
      checkAction(0);
    }   
  }
  stopMot();
  panPos=120;
  busyFlag=atFlag;
}

// go right
void goR(){
  tPanPos=panPos;
  busyFlag=1;
  if(panPos>mRange){
   for(int r=0;r<(tPanPos-mRange);r++){
      motStep(0);
      checkAction(0);      
    }   
  } else if(panPos<=mRange){
   for(int r=0;r<(mRange-tPanPos);r++){
      motStep(1);
      checkAction(0);      
    }   
  }
  stopMot();
  panPos=mRange;
  busyFlag=atFlag;
}

void stepCW(){
  if(panPos<mRange){
   for (int i=0;i<8;i++){
     int j=7-i;
     PORTD = fdArrs[fdFlag][j]; 
     delayMicroseconds(mSpeed);
     stopMot();
   } 
   panPos=min(panPos+1,mRange);
  }
}

void stepCCW(){
  if(panPos>0){
    for (int i=0;i<8;i++){
     PORTD = fdArrs[fdFlag][i]; 
     delayMicroseconds(mSpeed);
     stopMot();
   } 
   panPos=max(panPos-1,0);
  }
}

// dir = 0 is CCW
// dir = 1 is CW
void motStep(int dir){
   for (int i=0;i<8;i++){
     int j=i;
     if (dir==1){
       j=7-i;
     }
     PORTD = fdArrs[fdFlag][j]; 
     delayMicroseconds(mSpeed);
//     stopMot();
   }
   if (dir==0){
     panPos=max(panPos-1,0);
   }
   else{
     panPos=min(panPos+1,mRange);
   }
}

void stopMot(){
    // PORTD = B00000100; // !!! change when windHallPin is D2
    // PORTD = B00001000;  
    PORTD = pow(2, windHallPin); // this stops the motor and keeps the pullup  resistor on pin 3 connected
}