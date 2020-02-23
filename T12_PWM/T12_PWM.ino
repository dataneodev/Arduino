
#include <SmoothADC.h>
SmoothADC    ADC_0;  
unsigned int  ADC0Value = 0;
unsigned int  MemTimeSerial;

void setup() {
  
  // put your setup code here, to run once:
  TCCR3B = TCCR3B & B11111000 | B00000010;
  
ADC_0.init(A0, TB_MS, 10);  // Init ADC0 attached to A0 with a 50ms acquisition period
  if (ADC_0.isDisabled()) { ADC_0.enable(); }

pinMode(5, OUTPUT);
analogWrite(5, 50);

Serial.begin(115200);
}

int wypelnienie = 0;
int zmiana = 1;

void loop() {
  unsigned int  tempTime = millis();
  
  ADC_0.serviceADCPin();
  
//  // put your main code here, to run repeatedly:
//analogWrite(5, wypelnienie); //Generujemy sygnał o zadanym wypełnieniu
// 
// if (wypelnienie < 255) { //Jeśli wypełnienie mniejsze od 100%
// wypelnienie = wypelnienie + zmiana; //Zwiększamy wypełnienie
// } else {
// wypelnienie = 0; //Jeśli wypełnienie większe od 100%, to wracamy na początek
// }

 
 if ((tempTime - MemTimeSerial) > 1000)
 {
    MemTimeSerial = tempTime;
    
    ADC0Value = ADC_0.getADCVal();
  Serial.print("Napiecie: ");
  Serial.println(ADC0Value);
  
  float  natezenie = (float)map(ADC0Value, 523, 938, 0, 1000) / 100.0;
  Serial.print("Natezenie: ");
  Serial.println(natezenie);
  }
 
}
