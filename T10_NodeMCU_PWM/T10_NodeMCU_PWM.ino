uint8_t LEDpin = D1;

/* By default PWM frequency is 1000Hz and we are using same 
   for this application hence no need to set */
uint16_t dutycycle = 0;
void setup(){
  Serial.begin(9600);
  Serial.print("PWM: "); 
  analogWrite(LEDpin, 0);  /* set initial 50% duty cycle */
  analogWriteFreq();
}

void loop(){
  dutycycle += 1;
  
  if(dutycycle > 1023) dutycycle = 0;/* limit dutycycle to 1023 if POT read cross it */
  
  analogWrite(LEDpin, dutycycle);
  delay(30);
}                                     
