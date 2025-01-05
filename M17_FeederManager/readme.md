####Sensors
+ 1 - S_BINARY - "24V Relay"
    + V_STATUS

+ 2 - S_BINARY - "5V 1 Relay"
    + V_STATUS

+ 3 - S_BINARY - "5V 2 Relay"
    + V_STATUS

+ 4 - S_BINARY - "Motion detection"
    + V_STATUS - On/Off
	
+ 5 - S_INFO - "Pomp working time"
	+ V_TEXT -  "Time in sec <20, 1200>"

+ 6 - S_BINARY - "Clock schedule"
    + V_STATUS - "On/Off"
	
+ 7 - S_INFO - "Working time on schedule"	
    + V_TEXT -  "Time in sec <20, 1200>"

+ 8 - S_INFO - "Clock schedule hours"	
	+ V_TEXT -  "Time from 00:00 in hours <1, 24>"

+ 9 - S_INFO - "Internal clock"
    + V_TEXT - "Get/set time in unixtime"