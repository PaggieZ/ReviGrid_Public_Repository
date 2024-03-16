// This sketch is for the ProtoGrid House Load Model
// Written by David Wilson
// Intern: Peiqi Zhu
// May, 2023

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme; // Create an instance of the BME280 sensor

#define M1 0x50 // EEPROM Memory Device Address
unsigned int eeMaxAddress = 65535; // max address on EEPROM

const byte NUM_OF_HOUSES = 4;
const byte hPin[NUM_OF_HOUSES] = {6, 9, 10, 11}; // pin number for each house LED // newer PCB versions might use D6 pin for house0
byte h[NUM_OF_HOUSES] = {0, 0, 0, 0}; // curr state of each house, light levels
byte hLim[NUM_OF_HOUSES] = {1, 1, 1, 1}; // the highest brightness level of each house

const byte MAX_LIGHT_LEVEL = 4; // light level starts from 0
const byte hPWM[MAX_LIGHT_LEVEL + 1] = {0, 30, 90, 180, 255}; // PWMs for different light levels

String serialReceived;
const byte argArrSize = 4;
String argArr[argArrSize];
byte numOfArgAvail = 0; // number of arguments available
const int timeoutDur = 2000;  // timeout duration for serial communication

long autoPrevTime=0; // last time auto mode changes the state of unit
bool autoFlag=0; // in auto mode, each house lights up randomly
bool chaseFlag=0; // in chase mode, each house turns on and off sequentially
long chasePrevTime = 0; // last time chase mode changes the state of unit
byte chasePWMIdx = MAX_LIGHT_LEVEL;
byte currChaseCycle = 0;
byte currHouseIdx = 0; // the index of current active house in chase mode, corresponds with the index of hPin[] array'

byte bmeFlag=0;

int load = 200; // load for one house

void setup() {
  Serial.begin(115200); 
  Wire.begin(); 
  if(bme.begin(0x76)){bmeFlag=1;}
  // set all house pins as OUTPUT
  for (byte i = 0; i < NUM_OF_HOUSES; i++) {
    pinMode(hPin[i], OUTPUT);
  }
  randomSeed(analogRead(0)); // randomize output of random() function
}

void loop() {
  checkAuto(); // execute auto mode if autoFlag is on
  checkChase(); // execute chase mode if chaseFlag is on
  checkAction(1); // execute user cmds if received
}

// initialize the module
void initDev(){
  blinkHouse(1); 
  chaseOn();
  // prevTime=millis();
}
 
// all available commands for users to interact with
char *commands[] = {"init","autoOn","autoOff","lightAll","lightsOut",
                     "light0","light1","light2","light3",
                     "blinkHouses", 
                     "chaseOn","chaseOff",
                     "setLimits>4", "setLoad>1",
                     "setLight0>1", "setLight1>1", "setLight2>1", "setLight3>1", 
                     "getAll<7","getLoads<4","getLoadVal<1","getKW<1","getCarbon<1","getTemp<1","getHumidity<1","getPressure<1",
                     "EPw>2", "EPr>1<1",
                     "off","eoc"}; // eoc stands for "end of commands"

// receive and execute user commands
void checkAction(bool checkAll) {
  if (enterSerialInput()) {
    if(serialReceived=="getAll"){getAll();}
    else if(serialReceived=="getLoads"){getLoads();}
    else if(serialReceived=="getLoadVal"){getLoadVal();} 
    else if(serialReceived=="getKW"){getKW();}    
    else if(serialReceived=="getCarbon"){getCarbon();}   
    else if(serialReceived=="getTemp"){getTemp();}  
    else if(serialReceived=="getHumidity"){getHumidity();} 
    else if(serialReceived=="getPressure"){getPressure();}             
    else if(serialReceived=="*ID?"){Serial.println("houseload");}
    else if(serialReceived=="getCommands"){getCommands();}

    if (checkAll == 1) { // check the rest of the cmds
      if(serialReceived=="init"){initDev();}
      else if(serialReceived=="setLimits"){setLimits();}
      else if(serialReceived=="setLoad"){setLoad();}
      else if(serialReceived=="blinkHouses"){blinkHouse(3);}
      else if(serialReceived=="autoOn"){autoOn();}
      else if(serialReceived=="autoOff"){autoOff();}
      else if(serialReceived=="lightsOut"){lightsOut();}
      else if(serialReceived=="lightAll"){lightAll();}
      else if(serialReceived=="light0"){light0();}
      else if(serialReceived=="light1"){light1();}
      else if(serialReceived=="light2"){light2();}
      else if(serialReceived=="light3"){light3();} 
      else if(serialReceived=="setLight0"){setLight0();} 
      else if(serialReceived=="setLight1"){setLight1();} 
      else if(serialReceived=="setLight2"){setLight2();} 
      else if(serialReceived=="setLight3"){setLight3();} 
      else if(serialReceived=="chaseOn"){chaseFlag=1;}
      else if(serialReceived=="chaseOff"){chaseFlag=0;}
      else if(serialReceived=="off"){off();}
      else if(serialReceived=="EPw"){writeEEPROM();}
      else if(serialReceived=="EPr"){readEEPROM();}
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


// turn the module off
void off(){
  chaseFlag = 0;
  autoOff();
  lightsOut();
  for(byte i = 0; i < NUM_OF_HOUSES; i++) {
    hLim[i] = 1;
  }
}

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

// send out the current status of the unit
void getAll(){
  int maxKW = load * 4 * (-1);
  Serial.println(maxKW); // max KW of the houses
  // int allocLoad=(hLim[0]+hLim[1]+hLim[2]+hLim[3])*(-1) * load;
  int curLoad=(h[0]+h[1]+h[2]+h[3])*(load/MAX_LIGHT_LEVEL);
  Serial.println(-1*curLoad); //this is the current house load
  Serial.println(-1*curLoad); // this is the load allocated to the houses
  Serial.println(0); // this is the difference between allocated and used 
  Serial.println(0); // this is the carbon value of the houses
  Serial.println(-1); // this means the houses are neither renewable or non-renewable
  // houseV=0;if(curLoad>0){houseV=120;}
  Serial.println(120); //this is the current house power
}

void getTemp()
{
  float val=0;
  if(bmeFlag){val=bme.readTemperature();}
  Serial.println(val);
}

void getHumidity()
{
  float val=0;
  if(bmeFlag){val=bme.readHumidity();}
  Serial.println(val);
}

void getPressure()
{
  float val=0;
  if(bmeFlag){val=bme.readPressure();}
  Serial.println(val);
}

void getCarbon(){
   Serial.println(0); // this is not a renewable energy source
}

// turn auto mode on  
void autoOn(){
  autoFlag=1;
  chaseFlag=0;
}

// turn auto mode off
void autoOff(){
  autoFlag=0;
}

// change the state of module if chase mode is on
// the function turns the curr active house off, the next house on
void checkChase(){
  if(chaseFlag == 0) { // exit when not in chase mode
    return;
  }
  if(millis() - chasePrevTime < 100) {
    return;
  }
  // get index of the next house
  byte nextHouseIdx = (currHouseIdx + 1) % NUM_OF_HOUSES;
  // chase 2 cycles for each brightness
  if(nextHouseIdx == 1) {
    if(currChaseCycle < 2) { // less than 2 cycles
      currChaseCycle++;
    } else { // has finished 2 cycles
      chasePWMIdx--;
      currChaseCycle = 1;
    }
    // update brightness
    if(chasePWMIdx == 0){
      chasePWMIdx = MAX_LIGHT_LEVEL;
    }
  }
  // Serial.print("nextHouseIdx: "); Serial.println(nextHouseIdx);
  // Serial.print("currChaseCycle: "); Serial.println(currChaseCycle);
  // Serial.print("chasePWMIdx: "); Serial.println(chasePWMIdx);
  // Serial.println();

  // turn curr house off
  digitalWrite(hPin[currHouseIdx], LOW);
  h[currHouseIdx] = 0;
  // turn next house on
  analogWrite(hPin[nextHouseIdx], hPWM[chasePWMIdx]);
  h[nextHouseIdx] = MAX_LIGHT_LEVEL;
  currHouseIdx = nextHouseIdx;
  chasePrevTime = millis();
}

// turn chase mode off
void chaseOff(){
  chaseFlag=0;
}

// turn chase mode on
void chaseOn(){
  chaseFlag=1;
}

void light0(){
  chaseOff();
  digitalWrite(hPin[0],HIGH);
  h[0]=MAX_LIGHT_LEVEL;
}

void light1(){
  chaseOff();
  digitalWrite(hPin[1],HIGH);
  h[1]=MAX_LIGHT_LEVEL;
}

void light2(){
  digitalWrite(hPin[2],HIGH);
  h[2]=MAX_LIGHT_LEVEL;
}

void light3(){
  chaseOff();
  digitalWrite(hPin[3],HIGH);
  h[3]=MAX_LIGHT_LEVEL;
}


// turn all lights on
// overwrite hLim[] and h[] to 1
void lightAll(){
  chaseOff();
  for (byte i = 0; i < NUM_OF_HOUSES; i++) { // for each house
    // overwrite limit
    hLim[i] = 1;
    // overwrite load
    h[i] = MAX_LIGHT_LEVEL;
    digitalWrite(hPin[i], HIGH);
  }
}


// turn off all houses
// overwrite hLim[] and h[] to 0
void lightsOut(){
  chaseOff();
  for (byte i = 0; i < NUM_OF_HOUSES; i++) {
    // overwrite house limit
    hLim[i] = 0; 
    // overwrite house brightness
    h[i] = 0;
    digitalWrite(hPin[i], LOW);
  }
}


// turn all lights on without changing any status
// used in blinkHouse()
void turnOn() {
  for (byte i = 0; i < NUM_OF_HOUSES; i++) {
    digitalWrite(hPin[i],HIGH);
  }
}

// turn all lights off without changing any status
// used in blinkHouse()
void turnOff() {
  for (byte i = 0; i < NUM_OF_HOUSES; i++) {
    digitalWrite(hPin[i],LOW);
  }
}


void blinkHouse(int blinkCount){
  chaseOff();
  for (int i=0; i < blinkCount; i++){
    turnOn();
    getDelay(300);
    turnOff();
    getDelay(300);
  }
  turnOn();
  getDelay(300);
}

// delay for "delayms" milliseconds
// check user action during delay
void getDelay(int delayms){
  unsigned long currTime=millis();
  while(millis()<currTime + delayms){
    checkAction(0); // check partial cmds only
  }
}

void setLimits(){
  int numOfArg = 4;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  for(byte i = 0; i < NUM_OF_HOUSES; i++) { // for each house
    hLim[i]=min(1,max(argArr[i].toInt(),0));
    digitalWrite(hPin[i],hLim[i]);
  } 
}


void checkAuto() {
  // return when last change was in less then timeout duration
  if(millis() - autoPrevTime < timeoutDur) {
    return;
  }
  // return if auto mode is off
  if(autoFlag == 0) {
    return;
  }

  for(byte i = 0; i < NUM_OF_HOUSES; i++) { // for each house
    if(random(5) == 0) { // if random() returns 0
      if(hLim[i] == 1) { // if current house limit is not 0
        h[i] = random(MAX_LIGHT_LEVEL + 1); // generate a random house load
        // if(load == 1) {
        //   h[i] = MAX_LIGHT_LEVEL;
        // } else {
        //   h[i] = 0;
        // }
        analogWrite(hPin[i], hPWM[h[i]]);
      } else {
        h[i] = 0;
        digitalWrite(hPin[i], 0);
      }
    }
  }
  autoPrevTime=millis(); // update last change time
}


void getLoads() {
  for(byte i = 0; i < NUM_OF_HOUSES; i++) {
    Serial.println(h[i]);
  }
}


void getLoadVal(){
  byte lVal=(h[0]+h[1]+h[2]+h[3])*(load/MAX_LIGHT_LEVEL);
  Serial.println(lVal);
}

void getKW(){
  float lVal=(h[0]+h[1]+h[2]+h[3])*(-1) * (load/MAX_LIGHT_LEVEL);
  Serial.println(lVal);
}


// set house load
void setLoad(){
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  load=min(500,max(argArr[0].toInt(),0));
}

// set brightness of house 0
void setLight0() {
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  h[0]=min(MAX_LIGHT_LEVEL, max(argArr[0].toInt(),0));
  analogWrite(hPin[0], hPWM[h[0]]);
}

// set brightness of house 1
void setLight1() {
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}
  
  h[1]=min(MAX_LIGHT_LEVEL,max(argArr[0].toInt(),0));
  analogWrite(hPin[1], hPWM[h[1]]);
}

// set brightness of house 2
void setLight2() {
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  h[2]=min(MAX_LIGHT_LEVEL,max(argArr[0].toInt(),0));
  analogWrite(hPin[2], hPWM[h[2]]);
}

// set brightness of house 3
void setLight3() {
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  h[3]=min(MAX_LIGHT_LEVEL,max(argArr[0].toInt(),0));
  analogWrite(hPin[3], hPWM[h[3]]);
}

void writeEEPROM() {
  int numOfArg = 2;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}

  // read data address (16-bit)
  unsigned int eeaddress = min(eeMaxAddress,max(argArr[0].toInt(),0));
  // read data value (8-bit)
  byte data = min(255,max(argArr[1].toInt(),0));

  Wire.beginTransmission(M1);
  Wire.write((int)(eeaddress >> 8));   // MSByte
  Wire.write((int)(eeaddress & 0xFF)); // LSByte
  Wire.write(data);
  if(Wire.endTransmission() == 0) {
    // Serial.println("write done");
  } else {
    // Serial.println("write failed");
  }
  delay(2); // time for eeprom to write to its memory
}

void readEEPROM() {
  int numOfArg = 1;
  if (numOfArgAvail == 0) {readSerTO(numOfArg);}
  // return when the number of arguments is insufficient.
  if (numOfArgAvail < numOfArg) {return;}
  // read data address (16-bit)
  unsigned int eeaddress = min(65535,max(argArr[0].toInt(),0));

  Wire.beginTransmission(M1);
  Wire.write((int)(eeaddress >> 8));   // MSByte
  Wire.write((int)(eeaddress & 0xFF)); // LSByte
  Wire.endTransmission();
  delay(2); // time for eeprom to read
  Wire.requestFrom(M1, 1); // second parameter means the num of byte(s) we are reading
  if (Wire.available() > 0) 
  {
    Serial.println(Wire.read());
  } 
  else{
    // Serial.println("NO DATA!");
  }
}


// void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) {
//   Wire.beginTransmission(deviceaddress);
//   Wire.write((int)(eeaddress >> 8));   // MSB
//   Wire.write((int)(eeaddress & 0xFF)); // LSB
//   Wire.write(data);
//   Serial.println(Wire.endTransmission());
//   delay(20);
// }

// void readEEPROM(int deviceaddress, unsigned int eeaddress ) 
// {
//   Wire.beginTransmission(deviceaddress);
//   Wire.write((int)(eeaddress >> 8));   // MSB
//   Wire.write((int)(eeaddress & 0xFF)); // LSB
//   Serial.println(Wire.endTransmission());
//   delay(5);
//   Wire.requestFrom(deviceaddress, 1);
//   if (Wire.available() > 0) 
//   {
//     Serial.println(Wire.read(), HEX);
//     } 
//   else{
//     Serial.println("NO DATA!");
//     }
// }