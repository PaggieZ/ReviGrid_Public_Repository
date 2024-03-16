void setup() {
  // put your setup code here, to run once:
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  pinMode(9,OUTPUT);
  Serial.begin(115200); 
  digitalWrite(9,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("D2: ");Serial.print(digitalRead(2));Serial.print(" D3: ");Serial.print(digitalRead(3));Serial.print(" D12: ");Serial.println(digitalRead(12));
 // delay(100);
}
