void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
  Serial.println(ESP.getChipId(), HEX);
  Serial.println(ESP.getFlashChipId(), HEX);

  Serial.println(ESP.getFlashChipSize());
  Serial.println(ESP.getFlashChipSpeed());
}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println("TICK");
  delay(2000);
}
