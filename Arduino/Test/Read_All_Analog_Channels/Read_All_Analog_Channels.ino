void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(" A0: "); Serial.print(analogRead(0));
  Serial.print(" A1: "); Serial.print(analogRead(1));
  Serial.print(" A2: "); Serial.print(analogRead(2));
  Serial.print(" A3: "); Serial.print(analogRead(3));
  Serial.print(" A4: "); Serial.print(analogRead(4));
  Serial.print(" A5: "); Serial.print(analogRead(5));
  Serial.print(" A6: "); Serial.println(analogRead(6));
  delay(500);
}
