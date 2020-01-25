/**
@file
Arduino library for communicating with Modbus slaves over RS232/485 (via RTU protocol).

@defgroup setup ModbusMasterDMod Object Instantiation/Initialization
@defgroup buffer ModbusMasterDMod Buffer Management
@defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
@defgroup register Modbus Function Codes for Holding/Input Registers
@defgroup constant Modbus Function Codes, Exception Codes
*/
/*

  ModbusMasterDMod.h - Arduino library for communicating with Modbus slaves
  over RS232/485 (via RTU protocol).

  Library:: ModbusMasterDMod

  Copyright:: 2009-2016 Doc Walker

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

  
#ifndef ModbusMasterDMod_h
#define ModbusMasterDMod_h


/**
@def __ModbusMasterDMod_DEBUG__ (0)
Set to 1 to enable debugging features within class:
  - PIN A cycles for each byte read in the Modbus response
  - PIN B cycles for each millisecond timeout during the Modbus response
*/
#define __ModbusMasterDMod_DEBUG__ (0)
#define __ModbusMasterDMod_DEBUG_PIN_A__ 4
#define __ModbusMasterDMod_DEBUG_PIN_B__ 5

/* _____STANDARD INCLUDES____________________________________________________ */
// include types & constants of Wiring core API
//#include "Arduino.h"

/* _____UTILITY MACROS_______________________________________________________ */


/* _____PROJECT INCLUDES_____________________________________________________ */
// functions to calculate Modbus Application Data Unit CRC
//#include "util/crc16.h"

static uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;

  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }

  return crc;
}

// functions to manipulate words
//#include "util/word.h"
static inline uint16_t lowWord(uint32_t ww)
{
  return (uint16_t) ((ww) & 0xFFFF);
}


/** @ingroup util_word
    Return high word of a 32-bit integer.

    @param uint32_t ww (0x00000000..0xFFFFFFFF)
    @return high word of input (0x0000..0xFFFF)
*/
static inline uint16_t highWord(uint32_t ww)
{
  return (uint16_t) ((ww) >> 16);
}

/* _____CLASS DEFINITIONS____________________________________________________ */
/**
Arduino class library for communicating with Modbus slaves over 
RS232/485 (via RTU protocol).
*/
class ModbusMasterDMod
{
  public:
    ModbusMasterDMod();
   
    void begin(uint8_t, Stream &serial);
    void idle(void (*)());
    void preTransmission(void (*)());
    void postTransmission(void (*)());
	  void SetSlaveID(uint8_t slave);

    // Modbus exception codes
    /**
    Modbus protocol illegal function exception.
    
    The function code received in the query is not an allowable action for
    the server (or slave). This may be because the function code is only
    applicable to newer devices, and was not implemented in the unit
    selected. It could also indicate that the server (or slave) is in the
    wrong state to process a request of this type, for example because it is
    unconfigured and is being asked to return register values.
    
    @ingroup constant
    */
    static const uint8_t ku8MBIllegalFunction            = 0x01;

    /**
    Modbus protocol illegal data address exception.
    
    The data address received in the query is not an allowable address for 
    the server (or slave). More specifically, the combination of reference 
    number and transfer length is invalid. For a controller with 100 
    registers, the ADU addresses the first register as 0, and the last one 
    as 99. If a request is submitted with a starting register address of 96 
    and a quantity of registers of 4, then this request will successfully 
    operate (address-wise at least) on registers 96, 97, 98, 99. If a 
    request is submitted with a starting register address of 96 and a 
    quantity of registers of 5, then this request will fail with Exception 
    Code 0x02 "Illegal Data Address" since it attempts to operate on 
    registers 96, 97, 98, 99 and 100, and there is no register with address 
    100. 
    
    @ingroup constant
    */
    static const uint8_t ku8MBIllegalDataAddress         = 0x02;
    
    /**
    Modbus protocol illegal data value exception.
    
    A value contained in the query data field is not an allowable value for 
    server (or slave). This indicates a fault in the structure of the 
    remainder of a complex request, such as that the implied length is 
    incorrect. It specifically does NOT mean that a data item submitted for 
    storage in a register has a value outside the expectation of the 
    application program, since the MODBUS protocol is unaware of the 
    significance of any particular value of any particular register.
    
    @ingroup constant
    */
    static const uint8_t ku8MBIllegalDataValue           = 0x03;
    
    /**
    Modbus protocol slave device failure exception.
    
    An unrecoverable error occurred while the server (or slave) was
    attempting to perform the requested action.
    
    @ingroup constant
    */
    static const uint8_t ku8MBSlaveDeviceFailure         = 0x04;

    // Class-defined success/exception codes
    /**
    ModbusMasterDMod success.
    
    Modbus transaction was successful; the following checks were valid:
      - slave ID
      - function code
      - response code
      - data
      - CRC
      
    @ingroup constant
    */
    static const uint8_t ku8MBSuccess                    = 0x00;
    
    /**
    ModbusMasterDMod invalid response slave ID exception.
    
    The slave ID in the response does not match that of the request.
    
    @ingroup constant
    */
    static const uint8_t ku8MBInvalidSlaveID             = 0xE0;
    
    /**
    ModbusMasterDMod invalid response function exception.
    
    The function code in the response does not match that of the request.
    
    @ingroup constant
    */
    static const uint8_t ku8MBInvalidFunction            = 0xE1;
    
    /**
    ModbusMasterDMod response timed out exception.
    
    The entire response was not received within the timeout period, 
    ModbusMasterDMod::ku8MBResponseTimeout. 
    
    @ingroup constant
    */
    static const uint8_t ku8MBResponseTimedOut           = 0xE2;
    
    /**
    ModbusMasterDMod invalid response CRC exception.
    
    The CRC in the response does not match the one calculated.
    
    @ingroup constant
    */
    static const uint8_t ku8MBInvalidCRC                 = 0xE3;
    
    uint16_t getResponseBuffer(uint8_t);
    void     clearResponseBuffer();
    uint8_t  setTransmitBuffer(uint8_t, uint16_t);
    void     clearTransmitBuffer();
    
    void beginTransmission(uint16_t);
    uint8_t requestFrom(uint16_t, uint16_t);
    void sendBit(bool);
    void send(uint8_t);
    void send(uint16_t);
    void send(uint32_t);
    uint8_t available(void);
    uint16_t receive(void);
    
    
    uint8_t  readCoils(uint16_t, uint16_t);
    uint8_t  readDiscreteInputs(uint16_t, uint16_t);
    uint8_t  readHoldingRegisters(uint16_t, uint16_t);
    uint8_t  readInputRegisters(uint16_t, uint8_t);
    uint8_t  writeSingleCoil(uint16_t, uint8_t);
    uint8_t  writeSingleRegister(uint16_t, uint16_t);
    uint8_t  writeMultipleCoils(uint16_t, uint16_t);
    uint8_t  writeMultipleCoils();
    uint8_t  writeMultipleRegisters(uint16_t, uint16_t);
    uint8_t  writeMultipleRegisters();
    uint8_t  maskWriteRegister(uint16_t, uint16_t, uint16_t);
    uint8_t  readWriteMultipleRegisters(uint16_t, uint16_t, uint16_t, uint16_t);
    uint8_t  readWriteMultipleRegisters(uint16_t, uint16_t);
    
  private:
    Stream* _serial;                                             ///< reference to serial port object
    uint8_t  _u8MBSlave;                                         ///< Modbus slave (1..255) initialized in begin()
    static const uint8_t ku8MaxBufferSize                = 64;   ///< size of response/transmit buffers    
    uint16_t _u16ReadAddress;                                    ///< slave register from which to read
    uint16_t _u16ReadQty;                                        ///< quantity of words to read
    uint16_t _u16ResponseBuffer[ku8MaxBufferSize];               ///< buffer to store Modbus slave response; read via GetResponseBuffer()
    uint16_t _u16WriteAddress;                                   ///< slave register to which to write
    uint16_t _u16WriteQty;                                       ///< quantity of words to write
    uint16_t _u16TransmitBuffer[ku8MaxBufferSize];               ///< buffer containing data to transmit to Modbus slave; set via SetTransmitBuffer()
    uint16_t* txBuffer; // from Wire.h -- need to clean this up Rx
    uint8_t _u8TransmitBufferIndex;
    uint16_t u16TransmitBufferLength;
    uint16_t* rxBuffer; // from Wire.h -- need to clean this up Rx
    uint8_t _u8ResponseBufferIndex;
    uint8_t _u8ResponseBufferLength;
    
    // Modbus function codes for bit access
    static const uint8_t ku8MBReadCoils                  = 0x01; ///< Modbus function 0x01 Read Coils
    static const uint8_t ku8MBReadDiscreteInputs         = 0x02; ///< Modbus function 0x02 Read Discrete Inputs
    static const uint8_t ku8MBWriteSingleCoil            = 0x05; ///< Modbus function 0x05 Write Single Coil
    static const uint8_t ku8MBWriteMultipleCoils         = 0x0F; ///< Modbus function 0x0F Write Multiple Coils

    // Modbus function codes for 16 bit access
    static const uint8_t ku8MBReadHoldingRegisters       = 0x03; ///< Modbus function 0x03 Read Holding Registers
    static const uint8_t ku8MBReadInputRegisters         = 0x04; ///< Modbus function 0x04 Read Input Registers
    static const uint8_t ku8MBWriteSingleRegister        = 0x06; ///< Modbus function 0x06 Write Single Register
    static const uint8_t ku8MBWriteMultipleRegisters     = 0x10; ///< Modbus function 0x10 Write Multiple Registers
    static const uint8_t ku8MBMaskWriteRegister          = 0x16; ///< Modbus function 0x16 Mask Write Register
    static const uint8_t ku8MBReadWriteMultipleRegisters = 0x17; ///< Modbus function 0x17 Read Write Multiple Registers
    
    // Modbus timeout [milliseconds]
    static const uint16_t ku16MBResponseTimeout          = 2000; ///< Modbus timeout [milliseconds]
    
    // master function that conducts Modbus transactions
    uint8_t ModbusMasterDModTransaction(uint8_t u8MBFunction);
    
    // idle callback function; gets called during idle time between TX and RX
    void (*_idle)();
    // preTransmission callback function; gets called before writing a Modbus message
    void (*_preTransmission)();
    // postTransmission callback function; gets called after a Modbus message has been sent
    void (*_postTransmission)();
};


/* _____GLOBAL VARIABLES_____________________________________________________ */


/* _____PUBLIC FUNCTIONS_____________________________________________________ */
/**
  Constructor.

  Creates class object; initialize it using ModbusMasterDMod::begin().

  @ingroup setup
*/
ModbusMasterDMod::ModbusMasterDMod(void)
{
  _idle = 0;
  _preTransmission = 0;
  _postTransmission = 0;
}

/**
  Initialize class object.

  Assigns the Modbus slave ID and serial port.
  Call once class has been instantiated, typically within setup().

  @param slave Modbus slave ID (1..255)
  @param &serial reference to serial port object (Serial, Serial1, ... Serial3)
  @ingroup setup
*/
void ModbusMasterDMod::begin(uint8_t slave, Stream &serial)
{
  //  txBuffer = (uint16_t*) calloc(ku8MaxBufferSize, sizeof(uint16_t));
  _u8MBSlave = slave;
  _serial = &serial;
  _u8TransmitBufferIndex = 0;
  u16TransmitBufferLength = 0;

#if __ModbusMasterDMod_DEBUG__
  pinMode(__ModbusMasterDMod_DEBUG_PIN_A__, OUTPUT);
  pinMode(__ModbusMasterDMod_DEBUG_PIN_B__, OUTPUT);
#endif
}

void ModbusMasterDMod::SetSlaveID(uint8_t slave)
{
  _u8MBSlave = slave;
}


void ModbusMasterDMod::beginTransmission(uint16_t u16Address)
{
  _u16WriteAddress = u16Address;
  _u8TransmitBufferIndex = 0;
  u16TransmitBufferLength = 0;
}

// eliminate this function in favor of using existing MB request functions
uint8_t ModbusMasterDMod::requestFrom(uint16_t address, uint16_t quantity)
{
  uint8_t read;
  // clamp to buffer length
  if (quantity > ku8MaxBufferSize)
  {
    quantity = ku8MaxBufferSize;
  }
  // set rx buffer iterator vars
  _u8ResponseBufferIndex = 0;
  _u8ResponseBufferLength = read;

  return read;
}


void ModbusMasterDMod::sendBit(bool data)
{
  uint8_t txBitIndex = u16TransmitBufferLength % 16;
  if ((u16TransmitBufferLength >> 4) < ku8MaxBufferSize)
  {
    if (0 == txBitIndex)
    {
      _u16TransmitBuffer[_u8TransmitBufferIndex] = 0;
    }
    bitWrite(_u16TransmitBuffer[_u8TransmitBufferIndex], txBitIndex, data);
    u16TransmitBufferLength++;
    _u8TransmitBufferIndex = u16TransmitBufferLength >> 4;
  }
}


void ModbusMasterDMod::send(uint16_t data)
{
  if (_u8TransmitBufferIndex < ku8MaxBufferSize)
  {
    _u16TransmitBuffer[_u8TransmitBufferIndex++] = data;
    u16TransmitBufferLength = _u8TransmitBufferIndex << 4;
  }
}


void ModbusMasterDMod::send(uint32_t data)
{
  send(lowWord(data));
  send(highWord(data));
}


void ModbusMasterDMod::send(uint8_t data)
{
  send(word(data));
}









uint8_t ModbusMasterDMod::available(void)
{
  return _u8ResponseBufferLength - _u8ResponseBufferIndex;
}


uint16_t ModbusMasterDMod::receive(void)
{
  if (_u8ResponseBufferIndex < _u8ResponseBufferLength)
  {
    return _u16ResponseBuffer[_u8ResponseBufferIndex++];
  }
  else
  {
    return 0xFFFF;
  }
}








/**
  Set idle time callback function (cooperative multitasking).

  This function gets called in the idle time between transmission of data
  and response from slave. Do not call functions that read from the serial
  buffer that is used by ModbusMasterDMod. Use of i2c/TWI, 1-Wire, other
  serial ports, etc. is permitted within callback function.

  @see ModbusMasterDMod::ModbusMasterDModTransaction()
*/
void ModbusMasterDMod::idle(void (*idle)())
{
  _idle = idle;
}

/**
  Set pre-transmission callback function.

  This function gets called just before a Modbus message is sent over serial.
  Typical usage of this callback is to enable an RS485 transceiver's
  Driver Enable pin, and optionally disable its Receiver Enable pin.

  @see ModbusMasterDMod::ModbusMasterDModTransaction()
  @see ModbusMasterDMod::postTransmission()
*/
void ModbusMasterDMod::preTransmission(void (*preTransmission)())
{
  _preTransmission = preTransmission;
}

/**
  Set post-transmission callback function.

  This function gets called after a Modbus message has finished sending
  (i.e. after all data has been physically transmitted onto the serial
  bus).

  Typical usage of this callback is to enable an RS485 transceiver's
  Receiver Enable pin, and disable its Driver Enable pin.

  @see ModbusMasterDMod::ModbusMasterDModTransaction()
  @see ModbusMasterDMod::preTransmission()
*/
void ModbusMasterDMod::postTransmission(void (*postTransmission)())
{
  _postTransmission = postTransmission;
}


/**
  Retrieve data from response buffer.

  @see ModbusMasterDMod::clearResponseBuffer()
  @param u8Index index of response buffer array (0x00..0x3F)
  @return value in position u8Index of response buffer (0x0000..0xFFFF)
  @ingroup buffer
*/
uint16_t ModbusMasterDMod::getResponseBuffer(uint8_t u8Index)
{
  if (u8Index < ku8MaxBufferSize)
  {
    return _u16ResponseBuffer[u8Index];
  }
  else
  {
    return 0xFFFF;
  }
}


/**
  Clear Modbus response buffer.

  @see ModbusMasterDMod::getResponseBuffer(uint8_t u8Index)
  @ingroup buffer
*/
void ModbusMasterDMod::clearResponseBuffer()
{
  uint8_t i;

  for (i = 0; i < ku8MaxBufferSize; i++)
  {
    _u16ResponseBuffer[i] = 0;
  }
}


/**
  Place data in transmit buffer.

  @see ModbusMasterDMod::clearTransmitBuffer()
  @param u8Index index of transmit buffer array (0x00..0x3F)
  @param u16Value value to place in position u8Index of transmit buffer (0x0000..0xFFFF)
  @return 0 on success; exception number on failure
  @ingroup buffer
*/
uint8_t ModbusMasterDMod::setTransmitBuffer(uint8_t u8Index, uint16_t u16Value)
{
  if (u8Index < ku8MaxBufferSize)
  {
    _u16TransmitBuffer[u8Index] = u16Value;
    return ku8MBSuccess;
  }
  else
  {
    return ku8MBIllegalDataAddress;
  }
}


/**
  Clear Modbus transmit buffer.

  @see ModbusMasterDMod::setTransmitBuffer(uint8_t u8Index, uint16_t u16Value)
  @ingroup buffer
*/
void ModbusMasterDMod::clearTransmitBuffer()
{
  uint8_t i;

  for (i = 0; i < ku8MaxBufferSize; i++)
  {
    _u16TransmitBuffer[i] = 0;
  }
}


/**
  Modbus function 0x01 Read Coils.

  This function code is used to read from 1 to 2000 contiguous status of
  coils in a remote device. The request specifies the starting address,
  i.e. the address of the first coil specified, and the number of coils.
  Coils are addressed starting at zero.

  The coils in the response buffer are packed as one coil per bit of the
  data field. Status is indicated as 1=ON and 0=OFF. The LSB of the first
  data word contains the output addressed in the query. The other coils
  follow toward the high order end of this word and from low order to high
  order in subsequent words.

  If the returned quantity is not a multiple of sixteen, the remaining
  bits in the final data word will be padded with zeros (toward the high
  order end of the word).

  @param u16ReadAddress address of first coil (0x0000..0xFFFF)
  @param u16BitQty quantity of coils to read (1..2000, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup discrete
*/
uint8_t ModbusMasterDMod::readCoils(uint16_t u16ReadAddress, uint16_t u16BitQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16BitQty;
  return ModbusMasterDModTransaction(ku8MBReadCoils);
}


/**
  Modbus function 0x02 Read Discrete Inputs.

  This function code is used to read from 1 to 2000 contiguous status of
  discrete inputs in a remote device. The request specifies the starting
  address, i.e. the address of the first input specified, and the number
  of inputs. Discrete inputs are addressed starting at zero.

  The discrete inputs in the response buffer are packed as one input per
  bit of the data field. Status is indicated as 1=ON; 0=OFF. The LSB of
  the first data word contains the input addressed in the query. The other
  inputs follow toward the high order end of this word, and from low order
  to high order in subsequent words.

  If the returned quantity is not a multiple of sixteen, the remaining
  bits in the final data word will be padded with zeros (toward the high
  order end of the word).

  @param u16ReadAddress address of first discrete input (0x0000..0xFFFF)
  @param u16BitQty quantity of discrete inputs to read (1..2000, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup discrete
*/
uint8_t ModbusMasterDMod::readDiscreteInputs(uint16_t u16ReadAddress,
    uint16_t u16BitQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16BitQty;
  return ModbusMasterDModTransaction(ku8MBReadDiscreteInputs);
}


/**
  Modbus function 0x03 Read Holding Registers.

  This function code is used to read the contents of a contiguous block of
  holding registers in a remote device. The request specifies the starting
  register address and the number of registers. Registers are addressed
  starting at zero.

  The register data in the response buffer is packed as one word per
  register.

  @param u16ReadAddress address of the first holding register (0x0000..0xFFFF)
  @param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup register
*/
uint8_t ModbusMasterDMod::readHoldingRegisters(uint16_t u16ReadAddress,
    uint16_t u16ReadQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  return ModbusMasterDModTransaction(ku8MBReadHoldingRegisters);
}


/**
  Modbus function 0x04 Read Input Registers.

  This function code is used to read from 1 to 125 contiguous input
  registers in a remote device. The request specifies the starting
  register address and the number of registers. Registers are addressed
  starting at zero.

  The register data in the response buffer is packed as one word per
  register.

  @param u16ReadAddress address of the first input register (0x0000..0xFFFF)
  @param u16ReadQty quantity of input registers to read (1..125, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup register
*/
uint8_t ModbusMasterDMod::readInputRegisters(uint16_t u16ReadAddress,
    uint8_t u16ReadQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  return ModbusMasterDModTransaction(ku8MBReadInputRegisters);
}


/**
  Modbus function 0x05 Write Single Coil.

  This function code is used to write a single output to either ON or OFF
  in a remote device. The requested ON/OFF state is specified by a
  constant in the state field. A non-zero value requests the output to be
  ON and a value of 0 requests it to be OFF. The request specifies the
  address of the coil to be forced. Coils are addressed starting at zero.

  @param u16WriteAddress address of the coil (0x0000..0xFFFF)
  @param u8State 0=OFF, non-zero=ON (0x00..0xFF)
  @return 0 on success; exception number on failure
  @ingroup discrete
*/
uint8_t ModbusMasterDMod::writeSingleCoil(uint16_t u16WriteAddress, uint8_t u8State)
{
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = (u8State ? 0xFF00 : 0x0000);
  return ModbusMasterDModTransaction(ku8MBWriteSingleCoil);
}


/**
  Modbus function 0x06 Write Single Register.

  This function code is used to write a single holding register in a
  remote device. The request specifies the address of the register to be
  written. Registers are addressed starting at zero.

  @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
  @param u16WriteValue value to be written to holding register (0x0000..0xFFFF)
  @return 0 on success; exception number on failure
  @ingroup register
*/
uint8_t ModbusMasterDMod::writeSingleRegister(uint16_t u16WriteAddress,
    uint16_t u16WriteValue)
{
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = 0;
  _u16TransmitBuffer[0] = u16WriteValue;
  return ModbusMasterDModTransaction(ku8MBWriteSingleRegister);
}


/**
  Modbus function 0x0F Write Multiple Coils.

  This function code is used to force each coil in a sequence of coils to
  either ON or OFF in a remote device. The request specifies the coil
  references to be forced. Coils are addressed starting at zero.

  The requested ON/OFF states are specified by contents of the transmit
  buffer. A logical '1' in a bit position of the buffer requests the
  corresponding output to be ON. A logical '0' requests it to be OFF.

  @param u16WriteAddress address of the first coil (0x0000..0xFFFF)
  @param u16BitQty quantity of coils to write (1..2000, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup discrete
*/
uint8_t ModbusMasterDMod::writeMultipleCoils(uint16_t u16WriteAddress,
    uint16_t u16BitQty)
{
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = u16BitQty;
  return ModbusMasterDModTransaction(ku8MBWriteMultipleCoils);
}
uint8_t ModbusMasterDMod::writeMultipleCoils()
{
  _u16WriteQty = u16TransmitBufferLength;
  return ModbusMasterDModTransaction(ku8MBWriteMultipleCoils);
}


/**
  Modbus function 0x10 Write Multiple Registers.

  This function code is used to write a block of contiguous registers (1
  to 123 registers) in a remote device.

  The requested written values are specified in the transmit buffer. Data
  is packed as one word per register.

  @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
  @param u16WriteQty quantity of holding registers to write (1..123, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup register
*/
uint8_t ModbusMasterDMod::writeMultipleRegisters(uint16_t u16WriteAddress,
    uint16_t u16WriteQty)
{
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = u16WriteQty;
  return ModbusMasterDModTransaction(ku8MBWriteMultipleRegisters);
}

// new version based on Wire.h
uint8_t ModbusMasterDMod::writeMultipleRegisters()
{
  _u16WriteQty = _u8TransmitBufferIndex;
  return ModbusMasterDModTransaction(ku8MBWriteMultipleRegisters);
}


/**
  Modbus function 0x16 Mask Write Register.

  This function code is used to modify the contents of a specified holding
  register using a combination of an AND mask, an OR mask, and the
  register's current contents. The function can be used to set or clear
  individual bits in the register.

  The request specifies the holding register to be written, the data to be
  used as the AND mask, and the data to be used as the OR mask. Registers
  are addressed starting at zero.

  The function's algorithm is:

  Result = (Current Contents && And_Mask) || (Or_Mask && (~And_Mask))

  @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
  @param u16AndMask AND mask (0x0000..0xFFFF)
  @param u16OrMask OR mask (0x0000..0xFFFF)
  @return 0 on success; exception number on failure
  @ingroup register
*/
uint8_t ModbusMasterDMod::maskWriteRegister(uint16_t u16WriteAddress,
    uint16_t u16AndMask, uint16_t u16OrMask)
{
  _u16WriteAddress = u16WriteAddress;
  _u16TransmitBuffer[0] = u16AndMask;
  _u16TransmitBuffer[1] = u16OrMask;
  return ModbusMasterDModTransaction(ku8MBMaskWriteRegister);
}


/**
  Modbus function 0x17 Read Write Multiple Registers.

  This function code performs a combination of one read operation and one
  write operation in a single MODBUS transaction. The write operation is
  performed before the read. Holding registers are addressed starting at
  zero.

  The request specifies the starting address and number of holding
  registers to be read as well as the starting address, and the number of
  holding registers. The data to be written is specified in the transmit
  buffer.

  @param u16ReadAddress address of the first holding register (0x0000..0xFFFF)
  @param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
  @param u16WriteAddress address of the first holding register (0x0000..0xFFFF)
  @param u16WriteQty quantity of holding registers to write (1..121, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup register
*/
uint8_t ModbusMasterDMod::readWriteMultipleRegisters(uint16_t u16ReadAddress,
    uint16_t u16ReadQty, uint16_t u16WriteAddress, uint16_t u16WriteQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = u16WriteQty;
  return ModbusMasterDModTransaction(ku8MBReadWriteMultipleRegisters);
}
uint8_t ModbusMasterDMod::readWriteMultipleRegisters(uint16_t u16ReadAddress,
    uint16_t u16ReadQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  _u16WriteQty = _u8TransmitBufferIndex;
  return ModbusMasterDModTransaction(ku8MBReadWriteMultipleRegisters);
}


/* _____PRIVATE FUNCTIONS____________________________________________________ */
/**
  Modbus transaction engine.
  Sequence:
  - assemble Modbus Request Application Data Unit (ADU),
    based on particular function called
  - transmit request over selected serial port
  - wait for/retrieve response
  - evaluate/disassemble response
  - return status (success/exception)

  @param u8MBFunction Modbus function (0x01..0xFF)
  @return 0 on success; exception number on failure
*/
uint8_t ModbusMasterDMod::ModbusMasterDModTransaction(uint8_t u8MBFunction)
{
  uint8_t u8ModbusADU[256];
  uint8_t u8ModbusADUSize = 0;
  uint8_t i, u8Qty;
  uint16_t u16CRC;
  uint32_t u32StartTime;
  uint8_t u8BytesLeft = 8;
  uint8_t u8MBStatus = ku8MBSuccess;

  // assemble Modbus Request Application Data Unit
  u8ModbusADU[u8ModbusADUSize++] = _u8MBSlave;
  u8ModbusADU[u8ModbusADUSize++] = u8MBFunction;

  switch (u8MBFunction)
  {
    case ku8MBReadCoils:
    case ku8MBReadDiscreteInputs:
    case ku8MBReadInputRegisters:
    case ku8MBReadHoldingRegisters:
    case ku8MBReadWriteMultipleRegisters:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16ReadAddress);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16ReadAddress);
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16ReadQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16ReadQty);
      break;
  }

  switch (u8MBFunction)
  {
    case ku8MBWriteSingleCoil:
    case ku8MBMaskWriteRegister:
    case ku8MBWriteMultipleCoils:
    case ku8MBWriteSingleRegister:
    case ku8MBWriteMultipleRegisters:
    case ku8MBReadWriteMultipleRegisters:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteAddress);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteAddress);
      break;
  }

  switch (u8MBFunction)
  {
    case ku8MBWriteSingleCoil:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
      break;

    case ku8MBWriteSingleRegister:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[0]);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[0]);
      break;

    case ku8MBWriteMultipleCoils:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
      u8Qty = (_u16WriteQty % 8) ? ((_u16WriteQty >> 3) + 1) : (_u16WriteQty >> 3);
      u8ModbusADU[u8ModbusADUSize++] = u8Qty;
      for (i = 0; i < u8Qty; i++)
      {
        switch (i % 2)
        {
          case 0: // i is even
            u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[i >> 1]);
            break;

          case 1: // i is odd
            u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[i >> 1]);
            break;
        }
      }
      break;

    case ku8MBWriteMultipleRegisters:
    case ku8MBReadWriteMultipleRegisters:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty << 1);

      for (i = 0; i < lowByte(_u16WriteQty); i++)
      {
        u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[i]);
        u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[i]);
      }
      break;

    case ku8MBMaskWriteRegister:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[0]);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[0]);
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[1]);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[1]);
      break;
  }

  // append CRC
  u16CRC = 0xFFFF;
  for (i = 0; i < u8ModbusADUSize; i++)
  {
    u16CRC = crc16_update(u16CRC, u8ModbusADU[i]);
  }
  u8ModbusADU[u8ModbusADUSize++] = lowByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize++] = highByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize] = 0;

  // flush receive buffer before transmitting request
  while (_serial->read() != -1);

  // transmit request
  if (_preTransmission)
  {
    _preTransmission();
  }
  for (i = 0; i < u8ModbusADUSize; i++)
  {
    _serial->write(u8ModbusADU[i]);
  }

  u8ModbusADUSize = 0;
  _serial->flush();    // flush transmit buffer
  if (_postTransmission)
  {
    _postTransmission();
  }

  // loop until we run out of time or bytes, or an error occurs
  u32StartTime = millis();
  while (u8BytesLeft && !u8MBStatus)
  {
    if (_serial->available())
    {
#if __ModbusMasterDMod_DEBUG__
      digitalWrite(__ModbusMasterDMod_DEBUG_PIN_A__, true);
#endif
      u8ModbusADU[u8ModbusADUSize++] = _serial->read();
      u8BytesLeft--;
#if __ModbusMasterDMod_DEBUG__
      digitalWrite(__ModbusMasterDMod_DEBUG_PIN_A__, false);
#endif
    }
    else
    {
#if __ModbusMasterDMod_DEBUG__
      digitalWrite(__ModbusMasterDMod_DEBUG_PIN_B__, true);
#endif
      if (_idle)
      {
        _idle();
      }
#if __ModbusMasterDMod_DEBUG__
      digitalWrite(__ModbusMasterDMod_DEBUG_PIN_B__, false);
#endif
    }

    // evaluate slave ID, function code once enough bytes have been read
    if (u8ModbusADUSize == 5)
    {
      // verify response is for correct Modbus slave
      if (u8ModbusADU[0] != _u8MBSlave)
      {
        u8MBStatus = ku8MBInvalidSlaveID;
        break;
      }

      // verify response is for correct Modbus function code (mask exception bit 7)
      if ((u8ModbusADU[1] & 0x7F) != u8MBFunction)
      {
        u8MBStatus = ku8MBInvalidFunction;
        break;
      }

      // check whether Modbus exception occurred; return Modbus Exception Code
      if (bitRead(u8ModbusADU[1], 7))
      {
        u8MBStatus = u8ModbusADU[2];
        break;
      }

      // evaluate returned Modbus function code
      switch (u8ModbusADU[1])
      {
        case ku8MBReadCoils:
        case ku8MBReadDiscreteInputs:
        case ku8MBReadInputRegisters:
        case ku8MBReadHoldingRegisters:
        case ku8MBReadWriteMultipleRegisters:
          u8BytesLeft = u8ModbusADU[2];
          break;

        case ku8MBWriteSingleCoil:
        case ku8MBWriteMultipleCoils:
        case ku8MBWriteSingleRegister:
        case ku8MBWriteMultipleRegisters:
          u8BytesLeft = 3;
          break;

        case ku8MBMaskWriteRegister:
          u8BytesLeft = 5;
          break;
      }
    }
    if ((millis() - u32StartTime) > ku16MBResponseTimeout)
    {
      u8MBStatus = ku8MBResponseTimedOut;
    }
  }

  // verify response is large enough to inspect further
  if (!u8MBStatus && u8ModbusADUSize >= 5)
  {
    // calculate CRC
    u16CRC = 0xFFFF;
    for (i = 0; i < (u8ModbusADUSize - 2); i++)
    {
      u16CRC = crc16_update(u16CRC, u8ModbusADU[i]);
    }

    // verify CRC
    if (!u8MBStatus && (lowByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 2] ||
                        highByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 1]))
    {
      u8MBStatus = ku8MBInvalidCRC;
    }
  }

  // disassemble ADU into words
  if (!u8MBStatus)
  {
    // evaluate returned Modbus function code
    switch (u8ModbusADU[1])
    {
      case ku8MBReadCoils:
      case ku8MBReadDiscreteInputs:
        // load bytes into word; response bytes are ordered L, H, L, H, ...
        for (i = 0; i < (u8ModbusADU[2] >> 1); i++)
        {
          if (i < ku8MaxBufferSize)
          {
            _u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 4], u8ModbusADU[2 * i + 3]);
          }

          _u8ResponseBufferLength = i;
        }

        // in the event of an odd number of bytes, load last byte into zero-padded word
        if (u8ModbusADU[2] % 2)
        {
          if (i < ku8MaxBufferSize)
          {
            _u16ResponseBuffer[i] = word(0, u8ModbusADU[2 * i + 3]);
          }

          _u8ResponseBufferLength = i + 1;
        }
        break;

      case ku8MBReadInputRegisters:
      case ku8MBReadHoldingRegisters:
      case ku8MBReadWriteMultipleRegisters:
        // load bytes into word; response bytes are ordered H, L, H, L, ...
        for (i = 0; i < (u8ModbusADU[2] >> 1); i++)
        {
          if (i < ku8MaxBufferSize)
          {
            _u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 3], u8ModbusADU[2 * i + 4]);
          }

          _u8ResponseBufferLength = i;
        }
        break;
    }
  }

  _u8TransmitBufferIndex = 0;
  u16TransmitBufferLength = 0;
  _u8ResponseBufferIndex = 0;
  return u8MBStatus;
}

#endif

/**
@example examples/Basic/Basic.pde
@example examples/PhoenixContact_nanoLC/PhoenixContact_nanoLC.pde
@example examples/RS485_HalfDuplex/RS485_HalfDuplex.ino
*/
