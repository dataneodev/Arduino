
int PIN = A0;
int lastState = 0 ;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Uruchamiamy transmisję
  Serial.println("Czujnik swiatła"); //Jednorazowe wysłanie tekstu 
  
  pinMode(PIN, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int state = digitalRead(PIN);
  if(state != lastState){
    lastState = state;
    if(state == LOW){
      Serial.println("Czujnik aktywny"); //Jednorazowe wysłanie tekstu 
    }
    else{
      Serial.println("Czujnik nie aktywny"); //Jednorazowe wysłanie tekstu 
    }
  }
    
    
  delay(200);
}
