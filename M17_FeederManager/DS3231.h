// Modified by Andy Wickert 5/15/11: Spliced in stuff from RTClib
// Modified by Simon Gassner 11/28/2017: Changed Term "PM" to "PM_time" for compability with SAMD Processors
#ifndef DS3231_h
#define DS3231_h

#include <Arduino.h>
#include <time.h>
#include <Wire.h>

// Checks if a year is a leap year
bool isleapYear(const uint16_t);

// Eric's original code is everything below this line
class DS3231
{
public:
	// Constructor
	DS3231();
	DS3231(TwoWire &w);

	TwoWire &_Wire;

	// Time-retrieval functions

	time_t getNow(TwoWire &_Wire = Wire);

	// Time-setting functions
	// Note that none of these check for sensibility: You can set the
	// date to July 42nd and strange things will probably result.

	// set epoch function gives the epoch as parameter and feeds the RTC
	// epoch = UnixTime and starts at 01.01.1970 00:00:00
	void setEpoch(time_t epoch = 0, bool flag_localtime = false);

	// Temperature function

	float getTemperature();

	// Oscillator functions

	void enableOscillator(bool TF, bool battery, byte frequency);
	// turns oscillator on or off. True is on, false is off.
	// if battery is true, turns on even for battery-only operation,
	// otherwise turns off if Vcc is off.
	// frequency must be 0, 1, 2, or 3.
	// 0 = 1 Hz
	// 1 = 1.024 kHz
	// 2 = 4.096 kHz
	// 3 = 8.192 kHz (Default if frequency byte is out of range);
	void enable32kHz(bool TF);
	// Turns the 32kHz output pin on (true); or off (false).
	bool oscillatorCheck();
	;
	// Checks the status of the OSF (Oscillator Stop Flag);.
	// If this returns false, then the clock is probably not
	// giving you the correct time.
	// The OSF is cleared by function setSecond();.

	void setSecond(byte Second);
	// In addition to setting the seconds, this clears the
	// "Oscillator Stop Flag".
	void setMinute(byte Minute);
	// Sets the minute
	void setHour(byte Hour);
	// Sets the hour
	void setDoW(byte DoW);
	// Sets the Day of the Week (1-7);
	void setDate(byte Date);
	// Sets the Date of the Month
	void setMonth(byte Month);
	// Sets the Month of the year
	void setYear(byte Year);

private:
	byte decToBcd(byte val);
	// Convert normal decimal numbers to binary coded decimal
	byte bcdToDec(byte val);
	// Convert binary coded decimal to normal decimal numbers

protected:
	byte readControlByte(bool which);
	// Read selected control byte: (0); reads 0x0e, (1) reads 0x0f
	void writeControlByte(byte control, bool which);
	// Write the selected control byte.
	// which == false -> 0x0e, true->0x0f.
};

#endif
