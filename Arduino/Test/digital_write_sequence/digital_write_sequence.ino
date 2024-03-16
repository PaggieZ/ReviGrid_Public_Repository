byte dPin=2;
void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i=2;i<13;i++){
    digitalWrite(i,HIGH);
    Serial.print(i);Serial.println(" is on");
    delay(2000);
    digitalWrite(i,LOW);
    Serial.print(i);Serial.println(" is off");
  }
}
