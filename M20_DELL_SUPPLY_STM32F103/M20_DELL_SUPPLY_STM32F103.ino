#include <OneWire.h>

// Pin PA10 jest bezpieczniejszy (5V tolerant) i nie wymaga dodatkowej konfiguracji
#define ONE_WIRE_PIN PA10


// Dane dla zasilacza 130W (Moc: 130, Napięcie: 195, Natężenie: 067)
const char dellData[] = "DELL00AC130195067CN0JU0127161541C0185A03";
byte deviceID[] = {0x09, 0x44, 0x45, 0x4C, 0x4C, 0x31, 0x33, 0x21};

OneWire ow(ONE_WIRE_PIN);

void setup() {
  Serial.begin(115200);
  
  // Pin w trybie INPUT, linia jest sterowana zewnętrznym rezystorem Pull-up
  pinMode(ONE_WIRE_PIN, INPUT);
  
  Serial.println("Emulacja Dell 130W na pinie PA10...");
}

void loop() {
  // Krytyczne dla stabilności: wyłączamy przerwania podczas rozmowy z laptopem
  noInterrupts();
  
  if (ow.reset()) {
    byte command = ow.read();

    // Laptop pyta o numer seryjny (READ ROM)
    if (command == 0x33 || command == 0x0F) {
      ow.write_bytes(deviceID, 8);
    } 
    // Laptop pyta o moc i parametry (READ MEMORY)
    else if (command == 0xF0) {
      // Laptop wysyła 2 bajty adresu - musimy je odebrać przed wysłaniem danych
      ow.read(); 
      ow.read();
      
      // Wysyłamy parametry 130W
      for (int i = 0; i < sizeof(dellData) - 1; i++) {
        ow.write(dellData[i]);
      }
    }
  }
  
  interrupts();
  delayMicroseconds(100);
}
