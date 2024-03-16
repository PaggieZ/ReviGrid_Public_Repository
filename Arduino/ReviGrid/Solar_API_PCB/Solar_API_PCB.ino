// This sketch is for the ProtoGrid Solar Panel Model
// Written by David Wilson
// Intern: Peiqi Zhu
// May, 2023

#include <SPI.h>
#include <MedianFilterLib.h>
#include <EEPROM.h> // [0] stores fdFlag, [1] stores MAX_PAN_POS flag, [2] + 400 = MAX_PAN_POS

// the following arrays set the patterns to energize the stepper motor. 
// there are 2 arrays because some motors are reversed
// once the direction is determined automatically, the fdFlag is stored in eeprom[0]

//forward array 
// char fdarray[] = {B10000000, B11000000, B01000000, B01100000, B00100000, B00110000, B00010000, B10010000};
//reverse array v
// char fdarray[] = {B10010000, B00010000, B00110000, B00100000, B01100000, B01000000, B11000000, B10000000};

char fdArrs[2][8] = {{B10000000, B11000000, B01000000, B01100000, B00100000, B00110000, B00010000, B10010000}, 
                     {B10010000, B00010000, B00110000, B00100000, B01100000, B01000000, B11000000, B10000000}};

byte fdFlag = 0; // indicate which of the two arrays is used for direction
byte flag51=1;   // set to 1 for MCP4151 chip else to 0 for MCP4162 chip

String serialReceived;
const byte argArrSize = 4;
String argArr[argArrSize];
byte numOfArgAvail = 0; // number of arguments available
const int timeoutDur = 2000;  // timeout duration for serial communication
const int csPin = 8; // Chip Select for the MCP41010/MCP4151 on pin D8

byte solarPin = A5; // solar panel reading pin (analog), use A0 for older versions and A5 for newer versions
byte hallPin = 12; // hall effect sensor reading pin (digital)

// the median filter library stores the last "nReads" elements of the window and calculates their median
int nReads=20; // num of reads, window size of median filter

long panPos=0; // pan position of stepper motor
long tPanPos=0;
int MAX_PAN_POS = 480; // max pan position
int nSteps = 10; // num of steps, step size for each short distance movement

bool onFlag=1; // indicate solar panel is on or off
bool atFlag=0; // auto track flag, track mode on or off
bool trackTog = 1; // track toggle, 1 -> CW, 0 -> CCW
bool busyFlag=0;


// larger mSpeed -> slower but stronger rotation -> more power -> more heat
// vice versa
// min mSpeed is around 750, motor might stall if mSpeed < 750
unsigned long mSpeed=1200; // motor speed, wait time between each coil pattern in micro sec
// maxIndex -> maxLightPos
int maxLightPos=0; // position with the max light level

// long moveSteps=0; // user input, num of step in moveCW() and moveCCW()
int loadVal=0; // load value

// dwelTime -> delayTime
int delayTime=200; // delay duration before read in runScan()
unsigned long mRange=MAX_PAN_POS; // motor range, range of panPos


// !!! setDwel -> setDelay
char *commands[] = {"init","runScan","trackOn","trackOff","runIVScan",
                    "goHome","goMax","go1Q","go2Q","go3Q","go4Q",
                    "moveCW>1","moveCCW>1","moveCWR<2","moveCCWR<2","lookCCW","lookCW",
                    "setSteps>1","setLoad>1",
                    "setRange>1", "setDelay>1", "setSpeed>1",
                    "getVal<1","getKW<1","getCarbon<1","getMax<1","getAll<7","getIV>1<2",
                    "getPos<1","getBusy<1", "getMaxPos<1", 
                    "on","off","eoc"};


MedianFilter<float> medianFilter(nReads); // median filter constructor


void setup() {
  Serial.begin(115200); // Starting serial communication
  SPI.begin(); // Initialize SPI
  pinMode(csPin, OUTPUT);
  setResHigh();
  DDRD = DDRD | B11110000;              // this sets pins 0-3 as inputs and 4 to 7 as outputs 
  pinMode(hallPin,INPUT_PULLUP);              // this is for the hall effect home sensor

//   EEPROM.write(0, 255); // erase fdFlag routine result
//   EEPROM.write(1, 255); // erase findMaxPanPos routine result

// set fdFlag
  byte val = EEPROM.read(0); // read eeprom[0]
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
  MAX_PAN_POS = EEPROM.read(2) + 400;
  mRange = MAX_PAN_POS;
  // Serial.println(mRange);
} // end setup()

void loop() { 
  checkAction(1); // check all cmds
  checkTrack();
}

void setResHigh(){
  digitalWrite(csPin, HIGH); // Ensure MCP41010 is not selected by default
  digitalWrite(csPin, LOW); // Select MCP41010
  SPI.transfer(0x11); // Command byte to write to potentiometer 0
  if(flag51){
    SPI.transfer(0); // Data byte for hi-scale value     // use this for MCP 4151 chip
  }
  else{
    SPI.transfer(255); // Data byte for hi-scale value   // use this for mcp4162 chip
  }

  digitalWrite(csPin, HIGH); // Deselect MCP41010
}

void findMaxPanPos() {
  busyFlag=1;
  byte sensorVal = digitalRead(hallPin);
  if (sensorVal == 0) { // move if is at home position already
    moveAway();
  }
  goHome();

  // rotate to the max position
  for (int i = 0; i < MAX_PAN_POS; i++) {
    stepCW();
  }

  // rotate backward and increase step count until home
  sensorVal = digitalRead(hallPin);
  int stepCount = 0;
  while (sensorVal == 1) {
    stepCCW();
    stepCount = stepCount + 1;
    sensorVal = digitalRead(hallPin);
    // Serial.println(stepCount);
    // Serial.println(sensorVal);
  }
  EEPROM.write(2, stepCount - 400); // store MAX_PAN_POS in EEPROM
  EEPROM.write(1, 0); // set the find routine flag
  busyFlag=atFlag;
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
  byte sensorVal = digitalRead(hallPin);
  if (sensorVal == 0) { // move if is at home position already
    moveAway();
  }
  fdFlag = 0;
  goHome(); // go home with fdArrs[0][]
  sensorVal = digitalRead(hallPin);
  if (sensorVal == 0) { // return if is at home position
    EEPROM.write(0, fdFlag); // update eeprom[0]
    return;
  }
  // use another array
  fdFlag = 1;
  goHome(); // go home with fdArrs[1][]
  sensorVal = digitalRead(hallPin);
  if (sensorVal == 0) { // return if is at home position
    EEPROM.write(0, fdFlag); // update eeprom[0]
    return;
  }
  // error message
}

// initialize unit
void initDev(){
  busyFlag=1;
  on();
  goHome();
  go2Q(); // go to quadrant 2
  busyFlag=0;
  setRes(255);
}

// receive and execute user cmds
// when checkAll = 1, check all cmds
// when checkAll = 0, check partial cmds
void checkAction(bool checkAll) {
  if(enterSerialInput()) {
    if(serialReceived=="getAll"){ getAll();}
    else if(serialReceived=="trackOff"){atFlag=0;busyFlag=0;} 
    else if(serialReceived=="getKW"){getKW();}
    else if(serialReceived=="getCarbon"){getCarbon();} 
    else if(serialReceived=="setLoad"){setLoad();}
    else if(serialReceived=="setRange"){setRange();}
    else if(serialReceived=="getPos"){getPos();}
    else if(serialReceived=="getMax"){getMax();}
    else if(serialReceived=="getVal"){getVal();}
    else if(serialReceived=="getBusy"){getBusy();}
    else if(serialReceived=="getIV"){getIV();}
    else if(serialReceived=="getMaxPos"){getMaxPos();}
    else if(serialReceived=="setSteps"){setSteps();}  
    else if(serialReceived=="setDelay"){setDelay();}   
    else if(serialReceived=="setSpeed"){setSpeed();}
    else if(serialReceived=="on"){on();}  
    else if(serialReceived=="off"){off();}  
    else if(serialReceived=="*ID?"){Serial.println("solartracker");}

    if (checkAll == 1) { // check the rest of the cmds
      if(serialReceived=="init"){initDev();}  
      else if(serialReceived=="moveCW"){moveCW();}
      else if(serialReceived=="moveCCW"){moveCCW();}
      else if(serialReceived=="moveCWR"){moveCWR();}
      else if(serialReceived=="moveCCWR"){moveCCWR();}
      else if(serialReceived=="goHome"){goHome();}
      else if(serialReceived=="runScan"){runScan();}
      else if(serialReceived=="runIVScan"){runIVScan();}
      else if(serialReceived=="goMax"){goMaxLight();}
      else if(serialReceived=="go1Q"){go1Q();}
      else if(serialReceived=="go2Q"){go2Q();}
      else if(serialReceived=="go3Q"){go3Q();}
      else if(serialReceived=="go4Q"){go4Q();}
      else if(serialReceived=="lookCCW"){lookCCW();}
      else if(serialReceived=="lookCW"){lookCW();}
      else if(serialReceived=="trackOn"){atFlag=1;busyFlag=1;}
      else if(serialReceived=="getCommands"){getCommands();}
    }
  }
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

// turns the unit off
void off(){
  busyFlag=1;
  atFlag=0; // turn auto tracking routine off
  goHome();
  go2Q();
  stopMot();
  onFlag=0; // turn solar panel to storage position
  busyFlag=0;
}

// turn on solar panel
void on(){
  onFlag=1;
}

// return unit status
void getAll(){
  Serial.println(290); // max KW of the solar turbine
  float solarVal = readSolar();
  Serial.println(solarVal); //this is the current solar power
  Serial.println(loadVal); // this is the load allocated to the solar tracker
  Serial.println(solarVal - loadVal);  
  Serial.println(0); 
  Serial.println(1); // this is a renewable energy source
  Serial.println(solarVal); //this is the current solar voltage
}

// read solar panel
// readAna0() -> readSolar()
float readSolar(){
  stopMot();
  float solarVal = 0;
  for (int i = 0; i < nReads; i++){ // read solar panel for "nReads" times
    // get median 
    solarVal = medianFilter.AddValue(analogRead(solarPin));
  }
  solarVal = solarVal * onFlag;
  return solarVal;
}

// executes a tracking routine to seek and follow the brightest light source
void checkTrack(){
  if(atFlag == 0) { // return when track mode is off
    return;
  }

  if(trackTog){
    lookCW();
    getDelay(1000); 
  } else {
    lookCCW();
    getDelay(1000); 
  }
  trackTog = !trackTog; // look at the other direction next time
}


void getKW(){
  float kwVal=(readSolar()/5.0)-loadVal;
  Serial.println(kwVal);
}

void getCarbon(){
  Serial.println(0);
}

// sets the KW load assigned to the solar panel
void setLoad(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  loadVal=argArr[0].toInt();  
  loadVal = max(0, min(290, loadVal)); 
}

// returns maximum position that the solar panel can rotate to
void getMaxPos() {
  Serial.println(mRange);
}

void getIV(){
  int numOfArg=1;
  if(numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  long res = min(101000,max(1000,atol(argArr[0].c_str()))); 
  long aRes=min(100000,max(0,res-1000));
  int dRes=(aRes/100000.0)*255;
  setRes(dRes);
  delay(200);
  int a5=analogRead(5);
  float voltage5=float(a5/1023.0)*5.0;
  float mA=float(voltage5/res)*1000;
  if(mA>2.0){setRes(255);Serial.println("999");Serial.println("999");return;}
  Serial.println(voltage5,3);Serial.println(mA,4);
}

// sets the delayTime used by getDelay() function
void setDelay(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  delayTime=argArr[0].toInt(); 
  delayTime = max(min(1000, delayTime), 0); 
  
}


void setSpeed(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  mSpeed=argArr[0].toInt(); 
  // motor would stall from 500 to 750
  mSpeed = min(500, mSpeed); 
  
}

// sets the number of steps the runScan, lookCW and lookCCW use to rotate the panel
// set the step size for each short distance movement
// used in runScan, lookCCW, lookCW
void setSteps(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  nSteps=argArr[0].toInt();  
  nSteps = max(1, min(mRange, nSteps));
}

void getVal(){
  Serial.println(readSolar());
}

void getPos(){
  Serial.println(panPos);
}

void getMax(){
  Serial.println(maxLightPos);
}

void getBusy(){
  Serial.println(busyFlag);
}

// set motor range
void setRange(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  mRange=argArr[0].toInt();  
  mRange = max(1, min(MAX_PAN_POS, mRange));
  
}

// go to quadrant 1, 9 o'clock
void go1Q(){
  busyFlag=1;
  tPanPos=panPos;
  if(tPanPos>85){ // passed Q1 already
    for(int r=0;r<(tPanPos-85);r++){
      motStep(0);
      checkAction(0); // check partial cmds only
    }   
  } else if(tPanPos<=85){
    for(int r=0;r<(85-tPanPos);r++){
      motStep(1);
      checkAction(0); // check partial cmds only
    }   
  }
  stopMot();
  panPos=85;
  busyFlag=atFlag;
}

// go to quadrant2, 12 o'clock
void go2Q(){
  busyFlag=1;
  tPanPos=panPos;
  if(tPanPos>215){ // passed Q2 already
    for(int r=0;r<(tPanPos-215);r++){
      motStep(0);
      checkAction(0); // check partial cmds only
    }   
  } else if(tPanPos<=215){ 
    for(int r=0;r<(215-tPanPos);r++){
      motStep(1);
      checkAction(0); // check partial cmds only
    }   
  }
  stopMot();
  panPos=215;
  busyFlag=atFlag;
}

// go to quadrant3, 3 o'clock
void go3Q(){
  busyFlag=1;
  tPanPos=panPos;
  if(tPanPos>345){ // passed Q3 already
    for(int r=0;r<(tPanPos-345);r++){
      motStep(0);
      checkAction(0); // check partial cmds only
    }   
  } else if(tPanPos<=345){
   for(int r=0;r<(345-tPanPos);r++){
      motStep(1);
      checkAction(0); // check partial cmds only
    }   
  }
  stopMot();
  panPos=350;
  busyFlag=atFlag;
}

// go to quadrant4, 6 o'clock
void go4Q(){
  busyFlag=1;
  tPanPos=panPos;
  if(tPanPos > mRange){ // passed Q4 already
   for(int r=0;r<(tPanPos-mRange);r++){
      motStep(0);
      checkAction(0); // check partial cmds only
    }   
  } else if(tPanPos<=mRange){
   for(int r=0;r<(mRange-tPanPos);r++){
      motStep(1);
      checkAction(0); // check partial cmds only
    }   
  }
  stopMot();
  panPos=mRange;
  busyFlag=atFlag;
}

// move CW according to user input
void moveCW(){
  busyFlag=1;
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  long moveSteps=argArr[0].toInt(); 
  if(panPos + moveSteps <= mRange){       
    for(int z=0;z<moveSteps;z++){
      stepCW();
      checkAction(0); // check partial cmds only
    }
  }
  stopMot();
 
  busyFlag=atFlag;
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
  Serial.println(readSolar());  
  busyFlag=atFlag;
}
 
void moveCCW(){
  busyFlag=1;
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  long moveSteps=argArr[0].toInt();
  // Serial.println(panPos - moveSteps);
  if((panPos - moveSteps) < 0) { // return when final panPos is out of range
    return;
  }

  for(int z=0;z<moveSteps;z++){
    stepCCW();
    checkAction(0); // check partial cmds only
  }
  // panPos=panPos-moveSteps;
  
  stopMot();
  busyFlag=atFlag;
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
  Serial.println(readSolar());  
  busyFlag=atFlag;
}

// delay for "delayms" milliseconds
// check user action during delay
void getDelay(int delayms){
  unsigned long currTime=millis();
  while(millis()<currTime + delayms){
    checkAction(0); // check partial cmds only
  }
}

// make a full rotation (320 degree) and find the max light position
void runScan(){
  busyFlag=1;
  atFlag = 0; // turn off track mode
  goHome();
  // fMaxAna0=0; // -> maxSolarVal
  float maxSolarVal = 0;
  // fAna0=0; // -> currSolarVal
  float currSolarVal = 0;
  for(int s = 0; s < mRange/nSteps; s++){ // scan range
    for(int y = 0; y < nSteps; y++){motStep(1);} // make a step
    stopMot();
    getDelay(delayTime);
    currSolarVal = readSolar(); // get new reading
    if(currSolarVal > maxSolarVal){ // update max if curr > max
      maxLightPos = s * nSteps;
      maxSolarVal = currSolarVal;
    } 
  }
  stopMot();
  panPos = mRange;
  goMaxLight(); 
  busyFlag=atFlag;
}

void  runIVScan(){
  busyFlag=1;
  atFlag = 0; // turn off track mode  
  int j=0;
  digitalWrite(csPin, HIGH); // Ensure MCP41010 is not selected by default
  bool IVFlag=true;
  int s=0;
  setRes(255);
  Serial.print("Resistance(ohms) , Voltage(V) , Current(mA) , Power(mW)");Serial.println(" ");
  delay(1000);
  while(s<255&&IVFlag){
     j=max(min(255-s,255),0);
     float resistance=float((100000.0/256*j)+1000); 
     setRes(j);
     int a5=analogRead(5);
     float voltage5=float(a5/1023.0)*5.0;
     float mA=(voltage5/resistance)*1000;
     if(mA>2.0){setRes(255);Serial.println("eod");Serial.println(s);return;}
     Serial.print(resistance);Serial.print(" , ");
     Serial.print(voltage5,3);Serial.print(" , ");
     Serial.print(mA,5);Serial.print(" , ");
     Serial.println(mA*voltage5,3);
     delay(200);
 //    if(voltage5<1.5){delay(500);} else{delay(200);}
     s=min(s+1,255);
  }
  setRes(255);Serial.println("eod");Serial.println(s);
}

void setRes(int resVal){
  digitalWrite(csPin, LOW); // Select MCP41010
  SPI.transfer(0x00); // Command byte to write to potentiometer 0
  resVal=max(1,resVal);
  if(flag51){
    SPI.transfer(255-resVal);  // use this line for mcp4151 chip 
  }
  else{
    SPI.transfer(resVal); // Data byte for mid-scale value  // use this line for mcp4162 chip   
  }

  digitalWrite(csPin, HIGH); // Deselect MCP41010
}

// go to position with max light 
void goMaxLight(){
  busyFlag=1;
  tPanPos=panPos;
  if(tPanPos > maxLightPos){
   for(int r=0;r<(tPanPos-maxLightPos);r++){
      motStep(0);
      checkAction(0); // check partial cmds only
    }   
  } else if(tPanPos<=maxLightPos){
   for(int r=0;r<(maxLightPos-tPanPos);r++){   
      motStep(1);
      checkAction(0); // check partial cmds only
    }   
  }
  stopMot();
  panPos=maxLightPos;
  busyFlag=atFlag;
}

void lookCCW(){
  busyFlag=1;
  // fAna0=readSolar(); // -> currSolarVal
  float currSolarVal = readSolar();
  // fPrevAna0=0; // -> prevSolarVal
  float prevSolarVal = 0;
  long endTime = millis() + 1000;
  while(currSolarVal > prevSolarVal && millis() < endTime){
    prevSolarVal = currSolarVal;
    for(int i = 0; i < nSteps; i++){
      stepCCW();
      checkAction(0); // check partial cmds only
    }
    currSolarVal = readSolar();
  }
  busyFlag=atFlag;
}

void lookCW(){
  busyFlag=1;
  // fAna0=readSolar(); // -> currSolarVal
  float currSolarVal = readSolar();
  // fPrevAna0=0; // -> prevSolarVal
  float prevSolarVal = 0;
  long endTime = millis() + 1000;
  while(currSolarVal > prevSolarVal && millis() < endTime){
    prevSolarVal = currSolarVal;
    for(int i=0;i<nSteps;i++){
      stepCW();
      checkAction(0); // check partial cmds only
    }
    currSolarVal = readSolar();
  }
  busyFlag=atFlag;
}
  
// go back to home position
void goHome(){
  busyFlag=1;
  int lim = digitalRead(hallPin);
  long prevTime = millis();
  // num of steps: mRange * 8 * mSpeed
  unsigned long watchDogDur = (mRange * 8 * mSpeed * 1.6 / 1000)*5; // added times 5 because there may be getPos and getAll commands that slow  things down
  // Serial.print("watchDogDur: ");
  // Serial.println(watchDogDur);
  while(lim == 1 && millis() - prevTime < watchDogDur){ // watchdog
    lim = digitalRead(hallPin);
    for (int i=0;i<8;i++){ // step CCW
      PORTD = fdArrs[fdFlag][i]; 
      delayMicroseconds(mSpeed);
    } 
    panPos=max(panPos-1,0);
    checkAction(0); // check partial cmds only
  }
  panPos=0;
  stopMot();
  busyFlag=atFlag;
}

void stepCW(){
  if(panPos<mRange){
   for (int i=0;i<8;i++){
     int j=7-i;
     PORTD = fdArrs[fdFlag][j]; 
     delayMicroseconds(mSpeed);
   } 
   panPos=min(panPos+1,mRange);
  }
}

void stepCCW(){
  if(panPos>0){
   for (int i=0;i<8;i++){
     PORTD = fdArrs[fdFlag][i]; 
     delayMicroseconds(mSpeed);
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
  }
  if (dir==0){
    panPos=max(panPos-1,0);
  }
  else{
    panPos=min(panPos+1,mRange);
  }
}

void stopMot(){
    PORTD = B00000000;  // this stops the motor 
}
