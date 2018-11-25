
int PIN = 8;
int PIN_REALY = 23;
int lastState = 0 ;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Uruchamiamy transmisję
  Serial.println("Czujnik swiatła"); //Jednorazowe wysłanie tekstu 
  
  pinMode(PIN, INPUT);
  pinMode(PIN_REALY, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int state = digitalRead(PIN);
  if(state != lastState){
    lastState = state;
    if(state == LOW){
      Serial.println("Czujnik aktywny"); //Jednorazowe wysłanie tekstu 
      digitalWrite(PIN_REALY, LOW);
    }
    
    else{
      Serial.println("Czujnik nie aktywny"); //Jednorazowe wysłanie tekstu 
      digitalWrite(PIN_REALY, HIGH);
    }
  }
    
    
  delay(200);
}
