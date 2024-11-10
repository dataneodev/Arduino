####Sensors
+ 1 - S_BINARY - "Status otwarcia drzwi"
	+ V_STATUS - Stan otwarcia drzwi
	+ V_VAR1 - dodanie nowego urządzenia BLE, treść wiadomosci to adres np.: CB:F7:92:0F:3B:2E

+ 2 - 11- S_BINARY - "Status otwarcia drzwi dla odpowiedniego lokalizatora, max 10 lokalizatorów"
	+ V_STATUS - Stan otwarcia drzwi
	+ V_VAR_1 - usuniecie urządzenia BLE, treść wiadomosci: 1

+ 20 - S_INFO - "Liczba cykli otwarcia"
	+ V_TEXT - (get)
	
+ 21 - S_BINARY - "Drzwi zawsze otwarte"
    + V_STATUS - On/Off

+ 22 - S_BINARY - "Drzwi zawsze zamknięte"
    + V_STATUS - On/Off

+ 23 - S_BINARY - "Autoryzacja BLE"
    + V_STATUS - "On/Off"

+ 24 - S_BINARY - "Swiatło"
    + V_STATUS - "On/Off"
	
+ 25 - S_TEMP - "Temperatura"
    + V_TEMP
	
+ 26 - S_INFO - "Minimalny poziom sygnału(RSSI) do otwarcia - bliżej 0 mocniejszy sygnał, bliżej 100 słabszy sygnał, optymalnie (50-70)"
    + V_TEXT - "Poziom get/set <5, 100>"
	
+ 27 - S_INFO - "Minimalny czas blokady otwarcia po ostatnim zamknięciu drzwi (w sek), 0 bez blokady"
    + V_TEXT - "Poziom get/set <0, 3600>"