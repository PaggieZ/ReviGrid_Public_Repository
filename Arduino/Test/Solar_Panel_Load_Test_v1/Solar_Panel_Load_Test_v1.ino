#include <SPI.h>

const int csPin = 8; // Chip Select for the MCP41010 on pin D8

int i=0;
int j=20;
float mA=0;
float voltage0=0;
float voltage5=0;
float vDrop=0;
float resistance=0;
float power=0;
float mW=0;
int a5=0;
int a0=0;
byte k=0;
int numPts=256;

void setup() {
  Serial.begin(115200);
  SPI.begin(); // Initialize SPI
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH); // Ensure MCP41010 is not selected by default
  digitalWrite(csPin, LOW); // Select MCP41010
  SPI.transfer(0x00); // Command byte to write to potentiometer 0
  SPI.transfer(255); // Data byte for hi-scale value
  digitalWrite(csPin, HIGH); // Deselect MCP41010
  for(k=1;k<50;k++){
    a5=analogRead(5);
    voltage5=float(a5/1023.0)*5.0;
    Serial.println(voltage5);
    delay(100);
  }

// while(1){}
//    k=(k+1)%256;
//    digitalWrite(csPin, LOW); // Select MCP41010
//    SPI.transfer(0x11); // Command byte to write to potentiometer 0
//    SPI.transfer(k); // Data byte for hi-scale value
//    digitalWrite(csPin, HIGH); // Deselect MCP41010
//    Serial.println(k);
//    delay(2000);
//  }
//  while(1){};
  Serial.println("Step Resistance Voltage Current");
}

void loop() {
  j=max(min(numPts-i,255),1); 
  resistance=float((100000.0/256*j)+1000);
  setRes(j);
  a5=analogRead(5);
  voltage5=float(a5/1023.0)*5.0;
  mA=(voltage5/resistance)*1000;
  if(mA>2.0){setRes(255);}
  mW=voltage5*mA;
  power = voltage5*mA;
// Serial.print("step: ");Serial.print(i);Serial.print(" ");
//  Serial.print(resistance,2);Serial.print(" ");
//  Serial.print(power,4);
//  Serial.print(a5); Serial.print(" ");
 Serial.print(voltage5); Serial.print(" ");
//   Serial.print(mA,4);
   Serial.println(" ");
  delay(200); // Wait for a second before repeating
  if(i>=numPts-1||mA>2.0)
  {
    setRes(255);
    Serial.println("Done...");
    Serial.println(i);
    while(1);
  }
  i=(i+1)%numPts;
}

void setRes(int resVal){
  digitalWrite(csPin, LOW); // Select MCP41010
  SPI.transfer(0x00); // Command byte to write to potentiometer 0
  resVal=max(1,resVal);
  SPI.transfer(resVal); // Data byte for mid-scale value
  digitalWrite(csPin, HIGH); // Deselect MCP41010
}
