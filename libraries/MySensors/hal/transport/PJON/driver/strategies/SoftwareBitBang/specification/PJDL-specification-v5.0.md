
### Specifications index

#### Network layer
- [PJON (Padded Jittering Operative Network) v4.0](/specification/PJON-protocol-specification-v4.0.md)
- [Network services list](/specification/PJON-network-services-list.md)
#### Data link layer
- **[PJDL (Padded Jittering Data Link) v5.0](/src/strategies/SoftwareBitBang/specification/PJDL-specification-v5.0.md)**
- [PJDLR (Padded Jittering Data Link over Radio) v3.0](/src/strategies/OverSampling/specification/PJDLR-specification-v3.0.md)
- [PJDLS (Padded Jittering Data Link byte Stuffed) v2.0](/src/strategies/AnalogSampling/specification/PJDLS-specification-v2.0.md)
- [TSDL (Tardy Serial Data Link) v3.0](/src/strategies/ThroughSerial/specification/TSDL-specification-v3.0.md)
- [SFSP (Secure Frame Separation Protocol) v1.0](/specification/SFSP-frame-separation-specification-v1.0.md)

---

## PJDL v5.0
```
Invented by Giovanni Blu Mitolo with the support of
Fred Larsen, Julio Aguirre, Gerhard Sittig and Jack Anderson
Publication date: 10/04/2010 Latest revision: 24/07/2020
Related implementation: /src/strategies/SoftwareBitBang/
Compatible versions: PJON v13.0 and following
Released into the public domain

10/04/2010 0.1 - First experimental release
12/02/2017 1.0 - Frame initializer, response made safe
31/03/2017 1.1 - Physical layer info
24/09/2017 2.0 - Modes 1, 2, 3
29/12/2018 3.0 - Medium access control info, mode 4
03/07/2019 4.0 - Response initializer
10/03/2020 4.1 - Maximum range experimentally determined
17/07/2020 5.0 - Timeout, tolerance and preamble added
```
PJDL (Padded Jittering Data Link) is an asynchronous serial data link for low-data-rate applications that supports both master-slave and multi-master communication over a common conductive medium. PJDL can be easily implemented on limited microcontrollers with low clock accuracy and can operate directly using a single input-output pin.

### Physical layer
The medium's maximum length is limited by the wiring resistance, by the voltage level used and by externally induced interference. The maximum length of the bus can reach between 800 and 2000 meters depending on the mode used.
```cpp
PJDL SINGLE WIRE BUS                           ______
 ______    ______    ______    ______         |      |
|      |  |      |  |      |  |      |        |DEVICE|
|DEVICE|  |DEVICE|  |DEVICE|  |DEVICE|        |______|
|______|  |______|  |______|  |______|            |
___|__________|________|___________|______/\/\/\__| IO PIN
 ___|__    __|___    ___|__    ___|__  |  110-180 Ω
|      |  |      |  |      |  |      | |  
|DEVICE|  |DEVICE|  |DEVICE|  |DEVICE| |__/\/\/\__  GND
|______|  |______|  |______|  |______|    8 kΩ - 5 MΩ    
```
It is suggested to add 8kΩ-5MΩ pull-down resistor as shown in the graph above to reduce externally induced interference. The longer is the length of the cable and the higher is the amount of induced interference, the lower should be the resistance of the pull-down resistor. Pins can be optionally protected against overload adding a current limiting resistor to each connected pin. The resistor value can be obtained solving the following equation `R = (operating voltage / pin max current drain)`, for example to obtain the current limiting resistor value for an Arduino Uno simply substitute its characteristics: `R = (5v / 0.030A) = 166.66Ω`.

### Communication modes
The proposed communication modes are the result of years of testing and optimization and have been selected to be easily supported by limited microcontrollers:

| Mode | Bandwidth          | Range | Preamble bit | Pad bit | Data bit | Keep busy bit | Latency | Timeout |
| ---- | ------------------ | ----- | ------------ | ------- | -------- | ------------- | ------- | ------- |
| 1    | 1.97kB/s - 15808Bd | 2000m | 11000µs      | 110µs   | 44µs     | 11µs          | 13µs    | 20µs/B  |
| 2    | 2.21kB/s - 17696Bd | 1600m | 9200µs       | 92µs    | 40µs     | 10µs          | 10µs    | 20µs/B  |
| 3    | 3.10kB/s - 24844Bd | 1200m | 7000µs       | 70µs    | 28µs     | 7µs           | 8µs     | 20µs/B  |
| 4    | 3.34kB/s - 26755Bd |  800m | 6500µs       | 65µs    | 26µs     | 6.5µs         | 5µs     | 20µs/B  |

The following table specifies the exclusive acceptable tolerance of each bit type:

| Mode | Preamble bit   | Padding bit | Data bit nonet | Keep busy bit |
| ---- | -------------- | ----------- | -------------- | ------------- |
| 1    | -11000us +0us  | -5us +17us  | -5us +17us     | -5µs +10µs    |
| 2    | -9200us +0us   | -4us +16us  | -4us +16us     | -5µs +10µs    |
| 3    | -7000us +0us   | -3us +11us  | -3us +11us     | -3µs +10µs    |
| 4    | -6500us +0us   | -3us +10us  | -3us +10us     | -3µs +10µs    |

Padding bit, data bit and keep busy bit have higher positive tolerance to accept bit-banged signals that are generally longer than expected.

### Medium access control
PJDL specifies a variation of the carrier-sense, non-persistent random multiple access method (non-persistent CSMA). Devices can detect an ongoing transmission for this reason collisions can only occur in multi-master mode when 2 or more devices start to transmit at the same time. When a collision occurs it can be detected by the receiver because of synchronization loss or by the transmitter if an active collision avoidance procedure is implemented.

### Byte transmission
Byte transmission is composed by 10 bits, the first two are called synchronization pad and are used to obtain sampling synchronization. The synchronization pad is composed by a high padding bit 2.5 times longer than data bits and a low data bit. The following 8 data bits contain information in LSB-first (least significant bit first) order.

The reception technique is based on 3 steps:
1. Find a high bit which matches a padding bit
2. Synchronize with its falling edge
3. Ensure it is followed by a low data bit

If so reception starts, if not, interference, synchronization loss or simply absence of communication is detected. While receiving a sequence of bytes a synchronization pad is acceptable even if prepended by a 0 of up to the maximum positive data bit nonet tolerance.

```cpp  
 ___________ ___________________________
| SYNC PAD  | DATA                      |
|_______    |___       ___     _____    |
|       |   |   |     |   |   |     |   |
|   1   | 0 | 1 | 0 0 | 1 | 0 | 1 1 | 0 |
|_______|___|___|_____|___|___|_____|___|
```
The synchronization pad adds overhead although it includes synchronization along with the data and eliminates the need of a dedicated clock line. The presence of the synchronization pad between each byte also ensures that a frame composed of a series of bytes with decimal value 0 can be transmitted safely without risk of collision.

### Frame transmission
Before a frame transmission the communication medium's state is analysed, if high communication is detected and collision is avoided, if low for a duration of one byte plus the latency and a small random time, frame transmission starts with a frame preamble and a frame initializer composed by 3 consecutive synchronization pads followed by data bytes. The synchronization pad is used for both byte and frame initialization to reduce the implementation complexity. PJDL frames do not have an intrinsic length limit.
```cpp  
 ________ __________ _________________ ________________
|ANALYSIS| PREAMBLE |   FRAME INIT    | DATA BYTES     |
|________|__________|_____ _____ _____|________________|
|        |          |Sync |Sync |Sync |Sync | Byte     |
|        |__________|___  |___  |___  |___  |     __   |
|        |          |   | |   | |   | |   | |    |  |  |
|00000000|    1     | 1 |0| 1 |0| 1 |0| 1 |0|0000|11|00|
|________|__________|___|_|___|_|___|_|___|_|____|__|__|
```
When a frame is received a low performance microcontroller with an inaccurate clock can correctly identify a preamble, synchronize with transmitter during the frame initializer and consequently each byte is received. Frame initialization is 100% reliable, false positives can only occur because of externally induced interference. If the implementation applies polling, the preamble can be used to reduce the chances of transmission failure when the receiver's polling frequency is too low to detect incoming frames. The preamble's maximum length is 100 times the length of a padding bit.

### Synchronous response
A frame transmission can be optionally followed by a synchronous response sent by its recipient. Between frame transmission and a synchronous response there is a variable time which duration is influenced by latency.
```cpp  
Transmission end                                   Response
 ______  ______  ______                             _____
| BYTE || BYTE || BYTE | CRC COMPUTATION / LATENCY | ACK |
|------||------||------|---------------------------|-----|
|      ||      ||      |                           |  6  |
|______||______||______|                           |_____|
```  
In order to avoid other devices to detect the medium free for use and disrupt an ongoing exchange, the sender cyclically transmits a high 1/4 data bit and consequently attempts to receive a response for up to twice the maximum expected latency. The receiver must synchronize to the falling edge of the last high bit and, in order to avoid false positives in case of collision, must transmit its response prepended with an additional synchronization pad. If the response is not transmitted or not received the transmitter continues to keep busy the medium up to the response timeout.
```cpp  
Transmission end            Bus is kept busy        Response
 ______  ______  ______   _   _   _   _   _   _ ____ _____  
| BYTE || BYTE || BYTE | | | | | | | | | | | | |SYNC| ACK |
|------||------||------| | | | | | | | | | | | |----|-----|
|      ||      ||      | | | | | | | | | | | | |    |  6  |
|______||______||______|_| |_| |_| |_| |_| |_| |____|_____|
```
The response timeout is determined multiplying 20µs by the length of the frame and then adding the maximum expected latency.
