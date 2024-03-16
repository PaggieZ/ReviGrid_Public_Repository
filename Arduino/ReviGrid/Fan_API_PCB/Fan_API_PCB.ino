// test
// This sketch is for the ProtoGrid Fan Model
// Written by David Wilson
// Intern: Peiqi Zhu
// May, 2023

String serialReceived;
const byte argArrSize = 4;
String argArr[argArrSize];
byte numOfArgAvail = 0; // number of arguments available
const int timeoutDur = 2000;  // timeout duration for serial communication

byte capPin = 12;  // Capacitive button input pin
byte motPin = 3;   // Fan motor output pin for transistor

bool fanFlag = 0;  // fan on or off by cmd

bool manualEntryFlag = 1;  // when manual mode is on, manualEntryFlag is on upon initial user interaction with sensor
bool autoFlag = 0;         // auto mode on or off
bool entryFlag = 1;        // first entry for current auto cycle
bool switchFlag = 1;       // used in auto mode for switching fan speed

// durations for auto mode
long dur1 = 0;
long dur2 = 0;

// commands and numerics
// String cmd;
// String nmbrString;
// float nmbr = 0;
// int spaceIndex = 0;
// int speed = 0;

byte fSpeed = 0;  // fan speed, duty cycle val ranged from 0 to 255 for analogWrite()

long cycleStartTime = 0;  // start time of the current auto cycle

char *commands[] = {"init", "fanOn", "fanOff", "autoOn", "autoOff", 
                    "setSpeed>1", 
                    "getKW<1", "getCarbon<1", "getAll<7", 
                    "off", "eoc"};

void setup() {
  Serial.begin(115200);
  TCCR2B = (TCCR2B & B11111000) | B00000101;  // set timer 2 divisor to  1024 for PWM frequency of 61.04 Hz
  pinMode(capPin, INPUT);
  pinMode(motPin, OUTPUT);
}

void loop() {
  checkAction();  // execute user cmds
  checkAuto();    // execute auto mode
  checkManual();  // execute manual mode
}

// initialize module
void initDev() {
  // gradual startup when all modules are initializing at the same time
  // full on can cause capacitive sensor reading mistake
  analogWrite(motPin, 200);
  delay(750);
  analogWrite(motPin, LOW);
}

void checkAction() {
  if (enterSerialInput()) {
    if (serialReceived == "getAll") {getAll();}
    else if (serialReceived == "fanOn") {fanOn();} 
    else if (serialReceived == "fanOff") {fanOff();} 
    else if (serialReceived == "autoOn") {autoOn();} 
    else if (serialReceived == "autoOff") {autoOff();} 
    else if (serialReceived == "getKW") {getKW();} 
    else if (serialReceived == "getCarbon") {getCarbon();} 
    else if (serialReceived == "setSpeed") {setFSpeed();} 
    else if (serialReceived == "getCommands") {getCommands();} 
    else if (serialReceived == "*ID?") {Serial.println("fan");} 
    else if (serialReceived == "init") {initDev();} 
    else if (serialReceived == "off") {off();}
  }
} // end, checkAction()

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


/*************************** getAll *****************************/
/*  The getAll() function sends out the current status of the fan module.
*/
void getAll() {
  Serial.println(0);  // max KW of the wind turbine
  int load = fSpeed * (-1);
  Serial.println(load);           // this is the current wind power
  Serial.println(0);              // this is the load allocated to the wind turbine
  Serial.println(0);              // this is the power available minus the load
  Serial.println(0);              // this is the carbon value
  Serial.println(-1);             // this means its neither renewable or non-renewable
  Serial.println(fSpeed / 50.0);  // this is the line voltage
}


// turn fan off
void off() {
  autoOff();
  fanOff();
}

// check input signal from capacitive sensor
void checkManual() {
  // exit when auto mode is on or when fan is already on by uer cmds
  if (autoFlag || fanFlag) {
    return;
  }

  // read capacitive sensor
  // with soldered pad B on capacitive sensor, signal stays on until next touch
  int capInput = digitalRead(capPin);  // capacitive sensor input
  if (capInput) {                      // if signal is on
    // if use fSpeed to check the following condition, fan turns off when auto mode is off
    if (manualEntryFlag != 0) {  // return when fan is already on
      return;
    }
    // turn fan on
    analogWrite(motPin, 200);
    delay(500);
    digitalWrite(motPin, HIGH);
    fSpeed = 255;
    manualEntryFlag = 1;
  } else {                       // if signal is off
    if (manualEntryFlag == 0) {  // return when user did not touch sensor
      return;
    }
    // turn fan off
    digitalWrite(motPin, LOW);
    fSpeed = 0;
    manualEntryFlag = 0;
  }
}  // end checkManual()


void autoOn() {
  autoFlag = 1;
  entryFlag = 1;
}

void autoOff() {
  autoFlag = 0;
  entryFlag = 1;
  switchFlag = 1;
}

void setFSpeed() {
  if (numOfArgAvail == 0) {
    readSerTO(1); 
  }
  if (numOfArgAvail < 1) {
    // return when the number of arguments is insufficient.
    return;
  }
  int speed = argArr[0].toInt();
  fSpeed = (max(min(255, speed), 0));
  analogWrite(motPin, fSpeed);
  fanFlag = 1;
  if (fSpeed == 0) { fanFlag = 0; }
}

// fan speed range is 0 to 255
// void setFSpeed() {
//   long time = millis();
//   if (spaceIndex == -1) {
//     while (Serial.available() < 1 && millis() - time < timeoutDur) {}  // check serial timeout
//     speed = 0;                                                         // user input speed
//     // read user input
//     if (millis() - time < timeoutDur) {
//       serialReceived = Serial.readStringUntil('\n');
//       speed = serialReceived.toInt();
//       // modify user input if out of range
//       speed = (max(min(255, speed), 0));
//       fSpeed = speed;
//       // change fan speed
//       analogWrite(motPin, fSpeed);
//       fanFlag = 1;
//       if (fSpeed == 0) { fanFlag = 0; }
//     }
//   } else {
//     speed = nmbrString.toInt();
//     // modify user input if out of range
//     speed = (max(min(255, speed), 0));
//     fSpeed = speed;
//     // change fan speed
//     analogWrite(motPin, fSpeed);
//     fanFlag = 1;
//     if (fSpeed == 0) { fanFlag = 0; }
//   }
// }

// turns fan on to full speed
void fanOn() {
  autoFlag = 0;
  analogWrite(motPin, 200);
  delay(500);
  digitalWrite(motPin, HIGH);
  fSpeed = 255;
  entryFlag = 1;
  fanFlag = 1;
}

void fanOff() {
  digitalWrite(motPin, LOW);
  fSpeed = 0;
  autoFlag = 0;
  entryFlag = 1;
  fanFlag = 0;
}

void getKW() {
  Serial.println("0");
}

void getCarbon() {
  float carbVal = 0;
  Serial.println(carbVal);
}

// simulates random breeze by automatically adjusting fan speed
void checkAuto() {
  if (autoFlag == 0) {  // exit when not in auto mode
    return;
  }
  // generate new random durations when first entering a cycle
  // cycle time = dur1 + dur2
  if (entryFlag == 1) {          // first entry to curr cycle
    cycleStartTime = millis();   // get start time of dur1
    dur1 = random(7000, 10000);  // fan stays full on
    dur2 = random(3000, 6000);   // random fan speed
    entryFlag = 0;
  }

  if (millis() - cycleStartTime < dur1) {  // if in dur 1
    if (switchFlag) {                      // first entry of dur 1
      // change speed
      fSpeed = random(190, 255);
      analogWrite(motPin, fSpeed);
      switchFlag = !switchFlag;  // prevent speed from changing again in dur 1
    }
  } else if (millis() - cycleStartTime < dur1 + dur2) {  // if in dur 2
    if (!switchFlag) {                                   // first entry of dur 2
      // change speed
      fSpeed = random(140, 180);
      analogWrite(motPin, fSpeed);
      switchFlag = !switchFlag;  // prevent speed from changing again in dur 2
    }
  } else {          // cycle completed
    entryFlag = 1;  // reset cycle entry flag
  }
}
