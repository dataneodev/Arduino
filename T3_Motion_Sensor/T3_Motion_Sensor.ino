
int PIN = A1;
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
    if(state == HIGH){
      Serial.println("Ruch WYKRYTY"); //Jednorazowe wysłanie tekstu 
    }
    else{
      Serial.println("Brak ruchu"); //Jednorazowe wysłanie tekstu 
    }
  }
    
    
  delay(200);
}
