#include <FastLED.h>

#define RGB_PIN        9      // LED DATA PIN !!!!!!!!!!!!!!!!!!!!!!!!!!
#define RGB_LED_NUM    40      // Number of LEDs on Strip
int brightness = 255;     // Brightness range [0..255]
#define CHIP_SET       WS2812B // Type of RGB LEDs
#define COLOR_CODE     GRB     // Sequence of colors in the data stream

int brightIdx = 0;
int brightPWM[6] = {20, 50, 110, 180, 255, 0};
// Define the array of LEDs
CRGB LEDs[RGB_LED_NUM];
int R = 255; 
int G = 255;
int B = 255;
int maxIdx = 22;
int currIdx = 0;

long stepDelay = 2000;
long stepPrevTime = 0;
byte stepFlag = 0;

String serialReceived;
int timeoutDur = 2000; // timeout duration for serial input in ms

char *commands[] = {"init", "getAll<7", "lightAll", "lightsOut",
                    "goHome", "goMax", "setMax>1", "goNext", "goBack", "goIdx>1",
                    "setBrightness>1", "getCurrIdx<1", "stepOn", "stepOff", "setStepDelay>1",
                    "off","eoc"};


void setup() {
  Serial.begin(115200);
  FastLED.addLeds<CHIP_SET, RGB_PIN, COLOR_CODE>(LEDs, RGB_LED_NUM).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(brightness);
  lightsOut();
}

void loop() {
  checkAction(); // execute user cmds
  checkStep();
}

void checkAction() {
  if (Serial.available() > 0) {
    serialReceived = Serial.readStringUntil('\n');
    if(serialReceived=="getAll"){getAll();}
    else if(serialReceived=="goHome"){goHome();}
    else if(serialReceived=="goNext"){goNext();}
    else if(serialReceived=="goBack"){goBack();}
    else if(serialReceived=="goIdx"){goIdx();}
    else if(serialReceived=="goMax"){goMax();}
    else if(serialReceived=="setMax"){setMax();}
    else if(serialReceived=="setBrightness"){setBrightness();}
    else if(serialReceived=="lightAll"){lightAll();}
    else if(serialReceived=="lightsOut"){lightsOut();}
    else if(serialReceived=="getCurrIdx"){getCurrIdx();}
    else if(serialReceived=="stepOn"){stepPrevTime = millis(); stepFlag = 1;}
    else if(serialReceived=="stepOff"){stepFlag = 0;}
    else if(serialReceived=="setStepDelay"){setStepDelay();}
    else if(serialReceived=="getCommands"){getCommands();}
    else if(serialReceived=="*ID?"){Serial.println("LEDSun");}
    else if(serialReceived=="init"){initDev();}
    else if(serialReceived=="off"){off();}
  }
}

void setStepDelay() {
  long prevTime = millis(); 
  while(Serial.available() < 1 && millis() - prevTime < timeoutDur){} // check serial timeout
  if(millis() - prevTime >= timeoutDur) {return;}
  serialReceived = Serial.readStringUntil('\n');
  stepDelay = min(10000,max(serialReceived.toInt(),0));
}

void checkStep() {
  if (stepFlag == 0) {
    return;
  }
  else if (millis() - stepPrevTime < stepDelay) {
    return;
  }

  goNext();
  stepPrevTime = millis();
}

void getCurrIdx() {
  Serial.println(currIdx);
}

void goIdx() {
  long prevTime = millis(); 
  while(Serial.available() < 1 && millis() - prevTime < timeoutDur){} // check serial timeout
  if(millis() - prevTime >= timeoutDur) {return;}
  serialReceived = Serial.readStringUntil('\n');
  currIdx = min(maxIdx,max(serialReceived.toInt(),0));
  lightsOut();
  LEDs[currIdx] = CRGB(R, G, B);
  FastLED.show();
}

void setBrightness() {
  long prevTime = millis(); 
  while(Serial.available() < 1 && millis() - prevTime < timeoutDur){} // check serial timeout
  if(millis() - prevTime >= timeoutDur) {return;}
  serialReceived = Serial.readStringUntil('\n');
  brightness = min(255,max(serialReceived.toInt(),0));
  FastLED.setBrightness(brightness);
  FastLED.show();
}

void goBack() {
  currIdx = currIdx - 1;
  if (currIdx < 0) {
    currIdx = maxIdx;
  }
  lightsOut();
  LEDs[currIdx] = CRGB(R, G, B);
  FastLED.show();
}

void goNext() {
  currIdx = currIdx + 1;
  if (currIdx >= maxIdx) {
    currIdx = 0;
  }
  lightsOut();
  LEDs[currIdx] = CRGB(R, G, B);
  FastLED.show();
}

void setMax() {
  long prevTime = millis(); 
  while(Serial.available() < 1 && millis() - prevTime < timeoutDur){} // check serial timeout
  if(millis() - prevTime >= timeoutDur) {return;}
  serialReceived = Serial.readStringUntil('\n');
  maxIdx = min(RGB_LED_NUM,max(serialReceived.toInt(),1));
  goMax();
}

void goMax() {
  lightsOut();
  LEDs[maxIdx - 1] = CRGB(R, G, B);
  FastLED.show();
  currIdx = maxIdx - 1;
}


void goHome() {
  lightsOut();
  LEDs[0] = CRGB(R, G, B);
  FastLED.show();
  currIdx = 0;
}

void lightAll() {
  for (int i = 0; i < maxIdx; i++) {
    LEDs[i] = CRGB(R, G, B);
  }
  FastLED.show();
}

void lightsOut() {
  for (int i = 0; i < RGB_LED_NUM; i++) {
    LEDs[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
}

// initialize module
void initDev(){
  stepFlag = 0;
  for (int i = 0; i < maxIdx; i++) {
    LEDs[i] = CRGB(R, G, B);
    FastLED.show();
    delay(20);
  }
  delay(200);
  lightsOut();
  delay(200);
  lightAll();
  delay(500);
  lightsOut();
  delay(200);
}

void off() {
  lightsOut();
}



// return module status
void getAll(){
  Serial.println(0);  // max KW of the wind turbine
  Serial.println(0); //this is the current wind power
  Serial.println(0); // this is the load allocated to the wind turbine
  Serial.println(0); // this is the power available minus the load
  Serial.println(0); // this is the carbon value
  Serial.println(0); // this means its neither renewable or non-renewable
  Serial.println(0); // this is the line voltage
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



// void loop() {
//   // Turn on/off each LED sequentially

//   int R = 255; 
//   int G = 255;
//   int B = 255;
//   int LEDSpeed = 4000; // in ms
//   int LEDShift = 0;

//   for (int i = LEDShift; i < RGB_LED_NUM; i++) {
//     LEDs[i] = CRGB(R, G, B); // Set color for the current LED
//     FastLED.show(); // Display the changes
//     delay(LEDSpeed); // Delay to show the LED on for a short time
//     LEDs[i] = CRGB(0, 0, 0); // Fade out the current LED
//     //delay(20); // Delay to show the LED off for a short time
//     FastLED.show(); // Display the changes
//     // FastLED.setBrightness(255);
//   }
// }

// PWM sequential
  // for (int i = LEDShift; i < RGB_LED_NUM; i++) {
  //   LEDs[i] = CRGB(R, G, B); // Set color for the current LED
  //   FastLED.show(); // Display the changes
  //   delay(LEDSpeed); // Delay to show the LED on for a short time
  //   LEDs[i] = CRGB(0, 0, 0); // Fade out the current LED
  //   //delay(20); // Delay to show the LED off for a short time
  //   FastLED.show(); // Display the changes
  //   FastLED.setBrightness(brightPWM[brightIdx]);
  //   Serial.println(brightPWM[brightIdx]);
  //   if (brightIdx == 5) {
  //     brightIdx = 0;
  //   } else {
  //     brightIdx++;
  //   }
  // }
