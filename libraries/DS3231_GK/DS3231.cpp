#include "DS3231.h"

#if defined(__AVR__)
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#endif
// Changed the following to work on 1.0
// #include "WProgram.h"
#include <Arduino.h>

#define CLOCK_ADDRESS 0x68

#define SECONDS_FROM_1970_TO_2000 946684800

// Constructor
DS3231::DS3231() : _Wire(Wire)
{
	// nothing to do for this constructor.
}

DS3231::DS3231(TwoWire &w) : _Wire(w)
{
}

static const uint8_t daysInMonth[] PROGMEM = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d)
{
	if (y >= 2000)
		y -= 2000;
	uint16_t days = d;
	for (uint8_t i = 1; i < m; ++i)
		days += pgm_read_byte(daysInMonth + i - 1);
	if (m > 2 && isleapYear(y))
		++days;
	return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s)
{
	return ((days * 24L + h) * 60 + m) * 60 + s;
}

static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }

bool isleapYear(const uint16_t y)
{
	if (y & 3) // check if divisible by 4
		return false;
	// only check other, when first failed
	return (y % 100 || y % 400 == 0);
}

///// ERIC'S ORIGINAL CODE FOLLOWS /////

time_t DS3231::getNow(TwoWire &_Wire)
{
	_Wire.beginTransmission(CLOCK_ADDRESS);
	_Wire.write(0); // This is the first register address (Seconds)
					// We'll read from here on for 7 bytes: secs reg, minutes reg, hours, days, months and years.
	_Wire.endTransmission();

	_Wire.requestFrom(CLOCK_ADDRESS, 7);
	uint16_t ss = bcd2bin(_Wire.read() & 0x7F);
	uint16_t mm = bcd2bin(_Wire.read());
	uint16_t hh = bcd2bin(_Wire.read());
	_Wire.read();
	uint16_t d = bcd2bin(_Wire.read());
	uint16_t m = bcd2bin(_Wire.read());
	uint16_t y = bcd2bin(_Wire.read()) + 2000;

	uint8_t yOff = y >= 2000 ? y - 2000 : y;
	uint32_t t;
	uint16_t days = date2days(yOff, m, d);
	t = time2long(days, hh, mm, ss);

	t += SECONDS_FROM_1970_TO_2000; // seconds from 1970 to 2000

	return t;
}

// setEpoch function gives the epoch as parameter and feeds the RTC
// epoch = UnixTime and starts at 01.01.1970 00:00:00
// HINT: => the AVR time.h Lib is based on the year 2000
void DS3231::setEpoch(time_t epoch, bool flag_localtime)
{
	struct tm tmnow;
	gmtime_r(&epoch, &tmnow);
	setSecond(tmnow.tm_sec);
	setMinute(tmnow.tm_min);
	setHour(tmnow.tm_hour);
	setDoW(tmnow.tm_wday + 1U);
	setDate(tmnow.tm_mday);
	setMonth(tmnow.tm_mon + 1U);
	setYear(tmnow.tm_year - 100U);
}

float DS3231::getTemperature()
{
	// Checks the internal thermometer on the DS3231 and returns the
	// temperature as a floating-point value.

	// Updated / modified a tiny bit from "Coding Badly" and "Tri-Again"
	// http://forum.arduino.cc/index.php/topic,22301.0.html

	byte tMSB, tLSB;
	float temp3231;

	// temp registers (11h-12h) get updated automatically every 64s
	_Wire.beginTransmission(CLOCK_ADDRESS);
	_Wire.write(0x11);
	_Wire.endTransmission();
	_Wire.requestFrom(CLOCK_ADDRESS, 2);

	// Should I do more "if available" checks here?
	if (_Wire.available())
	{
		tMSB = _Wire.read(); // 2's complement int portion
		tLSB = _Wire.read(); // fraction portion

		int16_t itemp = (tMSB << 8 | (tLSB & 0xC0)); // Shift upper byte, add lower
		temp3231 = ((float)itemp / 256.0);			 // Scale and return
	}
	else
	{
		temp3231 = -9999; // Impossible temperature; error value
	}

	return temp3231;
}

void DS3231::enableOscillator(bool TF, bool battery, byte frequency)
{
	// turns oscillator on or off. True is on, false is off.
	// if battery is true, turns on even for battery-only operation,
	// otherwise turns off if Vcc is off.
	// frequency must be 0, 1, 2, or 3.
	// 0 = 1 Hz
	// 1 = 1.024 kHz
	// 2 = 4.096 kHz
	// 3 = 8.192 kHz (Default if frequency byte is out of range)
	if (frequency > 3)
		frequency = 3;
	// read control byte in, but zero out current state of RS2 and RS1.
	byte temp_buffer = readControlByte(0) & 0b11100111;
	if (battery)
	{
		// turn on BBSQW flag
		temp_buffer = temp_buffer | 0b01000000;
	}
	else
	{
		// turn off BBSQW flag
		temp_buffer = temp_buffer & 0b10111111;
	}
	if (TF)
	{
		// set ~EOSC to 0 and INTCN to zero.
		temp_buffer = temp_buffer & 0b01111011;
	}
	else
	{
		// set ~EOSC to 1, leave INTCN as is.
		temp_buffer = temp_buffer | 0b10000000;
	}
	// shift frequency into bits 3 and 4 and set.
	frequency = frequency << 3;
	temp_buffer = temp_buffer | frequency;
	// And write the control bits
	writeControlByte(temp_buffer, 0);
}

void DS3231::enable32kHz(bool TF)
{
	// turn 32kHz pin on or off
	byte temp_buffer = readControlByte(1);
	if (TF)
	{
		// turn on 32kHz pin
		temp_buffer = temp_buffer | 0b00001000;
	}
	else
	{
		// turn off 32kHz pin
		temp_buffer = temp_buffer & 0b11110111;
	}
	writeControlByte(temp_buffer, 1);
}

bool DS3231::oscillatorCheck()
{
	// Returns false if the oscillator has been off for some reason.
	// If this is the case, the time is probably not correct.
	byte temp_buffer = readControlByte(1);
	bool result = true;
	if (temp_buffer & 0b10000000)
	{
		// Oscillator Stop Flag (OSF) is set, so return false.
		result = false;
	}
	return result;
}

/*****************************************
	Private Functions
 *****************************************/

byte DS3231::decToBcd(byte val)
{
	// Convert normal decimal numbers to binary coded decimal
	return ((val / 10 * 16) + (val % 10));
}

byte DS3231::bcdToDec(byte val)
{
	// Convert binary coded decimal to normal decimal numbers
	return ((val / 16 * 10) + (val % 16));
}

byte DS3231::readControlByte(bool which)
{
	// Read selected control byte
	// first byte (0) is 0x0e, second (1) is 0x0f
	_Wire.beginTransmission(CLOCK_ADDRESS);
	if (which)
	{
		// second control byte
		_Wire.write(0x0f);
	}
	else
	{
		// first control byte
		_Wire.write(0x0e);
	}
	_Wire.endTransmission();
	_Wire.requestFrom(CLOCK_ADDRESS, 1);
	return _Wire.read();
}

void DS3231::setSecond(byte Second)
{
	// Sets the seconds
	// This function also resets the Oscillator Stop Flag, which is set
	// whenever power is interrupted.
	_Wire.beginTransmission(CLOCK_ADDRESS);
	_Wire.write(0x00);
	_Wire.write(decToBcd(Second));
	_Wire.endTransmission();
	// Clear OSF flag
	byte temp_buffer = readControlByte(1);
	writeControlByte((temp_buffer & 0b01111111), 1);
}

void DS3231::setMinute(byte Minute)
{
	// Sets the minutes
	_Wire.beginTransmission(CLOCK_ADDRESS);
	_Wire.write(0x01);
	_Wire.write(decToBcd(Minute));
	_Wire.endTransmission();
}

// Following setHour revision by David Merrifield 4/14/2020 correcting handling of 12-hour clock

void DS3231::setHour(byte Hour)
{
	// Sets the hour, without changing 12/24h mode.
	// The hour must be in 24h format.

	bool h12;
	byte temp_hour;

	// Start by figuring out what the 12/24 mode is
	_Wire.beginTransmission(CLOCK_ADDRESS);
	_Wire.write(0x02);
	_Wire.endTransmission();
	_Wire.requestFrom(CLOCK_ADDRESS, 1);
	h12 = (_Wire.read() & 0b01000000);
	// if h12 is true, it's 12h mode; false is 24h.

	if (h12)
	{
		// 12 hour
		bool am_pm = (Hour > 11);
		temp_hour = Hour;
		if (temp_hour > 11)
		{
			temp_hour = temp_hour - 12;
		}
		if (temp_hour == 0)
		{
			temp_hour = 12;
		}
		temp_hour = decToBcd(temp_hour) | (am_pm << 5) | 0b01000000;
	}
	else
	{
		// 24 hour
		temp_hour = decToBcd(Hour) & 0b10111111;
	}

	_Wire.beginTransmission(CLOCK_ADDRESS);
	_Wire.write(0x02);
	_Wire.write(temp_hour);
	_Wire.endTransmission();
}

void DS3231::setDoW(byte DoW)
{
	// Sets the Day of Week
	_Wire.beginTransmission(CLOCK_ADDRESS);
	_Wire.write(0x03);
	_Wire.write(decToBcd(DoW));
	_Wire.endTransmission();
}

void DS3231::setDate(byte Date)
{
	// Sets the Date
	_Wire.beginTransmission(CLOCK_ADDRESS);
	_Wire.write(0x04);
	_Wire.write(decToBcd(Date));
	_Wire.endTransmission();
}

void DS3231::setMonth(byte Month)
{
	// Sets the month
	_Wire.beginTransmission(CLOCK_ADDRESS);
	_Wire.write(0x05);
	_Wire.write(decToBcd(Month));
	_Wire.endTransmission();
}

void DS3231::setYear(byte Year)
{
	// Sets the year
	_Wire.beginTransmission(CLOCK_ADDRESS);
	_Wire.write(0x06);
	_Wire.write(decToBcd(Year));
	_Wire.endTransmission();
}

void DS3231::writeControlByte(byte control, bool which)
{
	// Write the selected control byte.
	// which=false -> 0x0e, true->0x0f.
	_Wire.beginTransmission(CLOCK_ADDRESS);
	if (which)
	{
		_Wire.write(0x0f);
	}
	else
	{
		_Wire.write(0x0e);
	}
	_Wire.write(control);
	_Wire.endTransmission();
}
