####Sensors
+ 1 - S_BINARY - "24V Relay"
    + V_STATUS

+ 2 - S_BINARY - "5V 1 Relay"
    + V_STATUS

+ 3 - S_BINARY - "5V 2 Relay"
    + V_STATUS

+ 4 - S_BINARY - "Motion detection"
    + V_STATUS - On/Off
	+ V_VAR1 -  "Enable time in sec <20, 1200>"

+ 5 - S_BINARY - "Clock schedule"
    + V_STATUS - "On/Off"
    + V_VAR1 -  "Enable time in sec <20, 1200>"
	+ V_VAR2 -  "Interval start from 00:00 in hours <1, 24>"

+ 6 - S_CUSTOM - "Internal clock"
    + V_VAR1 - "Get/set time in unixtime"