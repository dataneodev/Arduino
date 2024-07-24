/* Instalacja
Dodać board https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
Zainstalować board STM32 MCU based boards
Zainstalować  Arduino SAM Boards
Podmienić pliki w MySensors dla STM32F1 libraries\MySensors\hal\architecture\STM32F1\

Dodatek do VS Code #region folding for VS Code
-RGBW
  - 1 kanał - biały
  - 2 kanał - red
  - 3 kanał - green
  - 4 kanał - blue

-RGB
  - 1 kanał - red
  - 2 kanał - green
  - 3 kanał - blue

-Single
  - 1 kanał - white


  tylko 1 z poniższych opcji moze być wybrana :
*/

/* #region  user configuration */

// #define NODE_1_RGBW
// #define NODE_1_RGB

#define NODE_1_SINGLE
#define NODE_1_MIN_LIGHT_LEVEL 5     // minimalna jasnosc
#define NODE_1_MAX_LIGHT_LEVEL 100   // maksymalna jasnosc
#define NODE_1_STARTUP_LIGHT_LEVEL 5 // 0-100 początkowa jasnosc jak włączono sterownik a poziom jasnosci jest 0

#if !defined NODE_1_RGBW && !defined NODE_1_RGB 
   #define NODE_2_SINGLE
   #define NODE_2_MIN_LIGHT_LEVEL 5     // minimalna jasnosc
   #define NODE_2_MAX_LIGHT_LEVEL 100   // maksymalna jasnosc
   #define NODE_2_STARTUP_LIGHT_LEVEL 5 // 0-100 początkowa jasnosc jak włączono sterownik a poziom jasnosci jest 0
#endif 

#if !defined NODE_1_RGBW && !defined NODE_1_RGB 
   #define NODE_3_SINGLE
   #define NODE_3_MIN_LIGHT_LEVEL 5     // minimalna jasnosc
   #define NODE_3_MAX_LIGHT_LEVEL 100   // maksymalna jasnosc
   #define NODE_3_STARTUP_LIGHT_LEVEL 5 // 0-100 początkowa jasnosc jak włączono sterownik a poziom jasnosci jest 0
#endif 

#if !defined NODE_1_RGBW
   #define NODE_4_SINGLE
   #define NODE_4_MIN_LIGHT_LEVEL 5     // minimalna jasnosc
   #define NODE_4_MAX_LIGHT_LEVEL 100   // maksymalna jasnosc
   #define NODE_4_STARTUP_LIGHT_LEVEL 5 // 0-100 początkowa jasnosc jak włączono sterownik a poziom jasnosci jest 0
#endif 

/* #endregion  user configuration */

/* #region  const configuration */
// MY SENSORS
#define SOFTWARE_VERION "1.0"

#define SKETCH_NAME "LED_DRIVER"

#if defined(NODE_1_RGBW)
#undef SKETCH_NAME
#define SKETCH_NAME "RGBW_LED_DRIVER"
#endif

#if defined(NODE_1_RGB)
#undef SKETCH_NAME
#define SKETCH_NAME "RGB_LED_DRIVER"
#endif

#define MY_NODE_ID 60 // id węzła my sensors - każdy sterownik musi miec inny numer

#if defined NODE_1_RGBW || defined NODE_1_RGB 
#define RGBW_ID 1
#endif

#if defined NODE_1_SINGLE || defined NODE_1_RGBW || defined NODE_1_RGB
#define DIMMER_ID_1 1
#endif 

#if defined NODE_2_SINGLE
#define DIMMER_ID_2 2
#endif 

#if defined NODE_3_SINGLE
#define DIMMER_ID_3 3
#endif 

#if defined NODE_4_SINGLE
#define DIMMER_ID_4 4
#endif

// RS485
#define ARDUINO_ARCH_STM32F1
#define MY_DISABLED_SERIAL        // manual configure Serial1
#define MY_RS485                  // Enable RS485 transport layer
#define MY_RS485_DE_PIN PA1       // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600   // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL Serial2 //
#define MY_RS485_SOH_COUNT 6
#define MY_TRANSPORT_WAIT_READY_MS 1

// 24C32
#define SCL_PIN PB10
#define SDA_PIN PB11

// relay
#define RELAY_PIN PB7

// pwm
#define PWM_1 PA10
#define PWM_2 PB6
#define PWM_3 PA8
#define PWM_4 PA9

// INPUT
#define IN_1 PB1
#define IN_2 PB0
#define IN_3 PA5
#define IN_4 PA4

/* #endregion */

/* #region Imports */


#include <MySensors.h>
#include <24C32.h>
#include <Wire.h>

/* #endregion */

/* #region Class definition */

class StateTime
{
private:
  unsigned long startState;

public:
  void stateStart()
  {
    startState = millis();
  }

  bool isElapsed(unsigned long elapsed)
  {
    unsigned long current = millis();

    if (startState > current)
    {
      startState = current;
      return false;
    }

    if (startState + elapsed < current)
    {
      return true;
    }

    return false;
  }
};

class StateChangeManager
{
private:
  bool _states[10];

public:
  bool isStateChanged(bool state, int index)
  {
    bool changed = _states[index] != state;
    _states[index] = state;

    return changed;
  }
};
/* #endregion */

/* #region  global variable */
EE EEPROM24C32;
MyMessage mMessage;
StateChangeManager SCM;

#if defined NODE_1_SINGLE || defined NODE_1_RGBW || defined NODE_1_RGB
bool node1Enabled = false;
uint8_t node1LightLevel = 50; // 0 -100
#endif 

#if defined NODE_1_RGBW
uint8_t node1WhiteChannel = 125; 
#endif 

#if defined NODE_1_RGBW || defined NODE_1_RGB
uint8_t node1RedChannel = 30;
uint8_t node1GreenChannel = 180;
uint8_t node1BlueChannel = 30;
#endif 

#if defined NODE_2_SINGLE
bool node2Enabled = false;
uint8_t node2LightLevel = 50; // 0 -100
#endif 

#if defined NODE_3_SINGLE
bool node3Enabled = false;
uint8_t node3LightLevel = 50; // 0 -100
#endif 

#if defined NODE_4_SINGLE
bool node4Enabled = false;
uint8_t node4DeviceLightLevel = 50; // 0 -100
#endif 
/* #endregion */

/* #region  Power Optimization */

/**
 * @brief  System Clock Configuration Reduced with big peripheral reduction
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 72000000 --> 48000000
 *            HCLK(Hz)                       = 72000000 --> 12000000
 *            AHB Prescaler                  = 1        --> /4
 *            APB1 Prescaler                 = 2        --> /1
 *            APB2 Prescaler                 = 1
 *            PLL_Source                     = HSE     --> HSE Pre DIV2
 *            PLL_Mul                        = 9       --> 12
 *            Flash Latency(WS)              = 2       --> 1
 *            ADC Prescaler                  = 6       --> 2
 *            USB Prescaler                  = 1.5     --> 1
 * @param  None
 * @retval None
 */
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void setAllPinsAnalog(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0, 0, 0, 0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  // DAC:
  /**DAC1 GPIO Configuration
  PA4 ------> DAC1_OUT1
  PA5 ------> DAC1_OUT2
  */

  GPIO_InitStruct.Pin = GPIO_PIN_All;

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void disableClocks()
{
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();

  __HAL_RCC_AFIO_CLK_DISABLE();
  __HAL_RCC_TIM1_CLK_DISABLE();
  __HAL_RCC_SPI1_CLK_DISABLE();
  __HAL_RCC_USART1_CLK_DISABLE();
  __HAL_RCC_DMA1_CLK_DISABLE();
  __HAL_RCC_TIM2_CLK_DISABLE();
  __HAL_RCC_TIM3_CLK_DISABLE();
  __HAL_RCC_WWDG_CLK_DISABLE();

  //__HAL_RCC_USART2_CLK_DISABLE();
  //__HAL_RCC_I2C1_CLK_DISABLE();

  //__HAL_RCC_BKP_CLK_DISABLE(); //Powe consupction goes up
  //__HAL_RCC_PWR_CLK_DISABLE(); //Powe consupction goes up
  //__HAL_RCC_HSI_DISABLE(); //Powe consupction goes up

  __HAL_RCC_LSI_DISABLE();
  __HAL_RCC_PLL_DISABLE();
  __HAL_RCC_RTC_DISABLE();
}

/* #endregion  Power Optimalization */



/* #region  inicjalize */
void inicjalizePins()
{
  // PWM
  digitalWrite(PWM_1, LOW);
  pinMode(PWM_1, OUTPUT);
  digitalWrite(PWM_1, LOW);

  digitalWrite(PWM_2, LOW);
  pinMode(PWM_2, OUTPUT);
  digitalWrite(PWM_2, LOW);

  digitalWrite(PWM_3, LOW);
  pinMode(PWM_3, OUTPUT);
  digitalWrite(PWM_3, LOW);

  digitalWrite(PWM_4, LOW);
  pinMode(PWM_4, OUTPUT);
  digitalWrite(PWM_4, LOW);

  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(MY_RS485_DE_PIN, OUTPUT);

  pinMode(IN_1, INPUT);
  pinMode(IN_2, INPUT);
  pinMode(IN_3, INPUT);
  pinMode(IN_4, INPUT);
}

void inicjalizeI2C()
{
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);
  EEPROM24C32.begin(0x50, false);
}
/* #endregion */

/* #region  data read / save */
#define CHECK_NUMBER 0x65
#define TRUE_BYTE 0X05

void readSettingFromEPPROM()
{
  if (EEPROM24C32.readByte(50) != CHECK_NUMBER)
  {
    saveDefaultSettingToEPPROM();
    return;
  }

#if defined NODE_1_RGBW
  node1WhiteChannel = EEPROM24C32.readByte(501);
 #endif 
 
 #if defined NODE_1_RGBW || defined NODE_1_RGB 
  node1RedChannel = EEPROM24C32.readByte(502);
  node1GreenChannel = EEPROM24C32.readByte(503);
  node1BlueChannel = EEPROM24C32.readByte(504);
 #endif 

#if defined NODE_1_SINGLE || defined NODE_1_RGBW || defined NODE_1_RGB
  node1Enabled = EEPROM24C32.readByte(100) == TRUE_BYTE;
  node1LightLevel = EEPROM24C32.readByte(101);
#endif 

#if defined NODE_2_SINGLE
  node2Enabled = EEPROM24C32.readByte(200) == TRUE_BYTE;
  node2LightLevel = EEPROM24C32.readByte(201);
#endif 

#if defined NODE_3_SINGLE
  node3Enabled = EEPROM24C32.readByte(300) == TRUE_BYTE;
  node3LightLevel = EEPROM24C32.readByte(301);
#endif 

#if defined NODE_4_SINGLE
  node4Enabled = EEPROM24C32.readByte(400) == TRUE_BYTE;
  node4LightLevel = EEPROM24C32.readByte(401);
 #endif 
}

void saveDefaultSettingToEPPROM()
{
#if defined(MY_DEBUG)
  Serial.println("Zapisuje domyslne ustawienia");
#endif

  EEPROM24C32.writeByte(105, CHECK_NUMBER, false, false);

#if defined NODE_1_SINGLE || defined NODE_1_RGBW || defined NODE_1_RGB
  saveNode1EnableToEPPROM(node1Enabled);
  setNode1LightLevelToEPPROM(node1LightLevel);
#endif 

#if defined NODE_1_RGBW
    setNode1WhiteChannel( node1WhiteChannel);
#endif 

#if defined NODE_1_RGBW || defined NODE_1_RGB
  setNode1RGBValues(node1RedChannel, node1GreenChannel, node1BlueChannel);
#endif 

#if defined NODE_2_SINGLE
  saveNode2EnableToEPPROM(node2Enabled);
  setNode2LightLevelToEPPROM(node2LightLevel);
#endif 

#if defined NODE_3_SINGLE
  saveNode3EnableToEPPROM(node3Enabled);
  setNode3LightLevelToEPPROM(node3LightLevel);
#endif 

#if defined NODE_4_SINGLE
  saveNode4EnableToEPPROM(node4Enabled);
  setNode4LightLevelToEPPROM(node4LightLevel);
  #endif 
}

#if defined NODE_1_SINGLE || defined NODE_1_RGBW || defined NODE_1_RGB
void saveNode1EnableToEPPROM(bool deviceEabled)
{
  EEPROM24C32.writeByte(100, deviceEabled ? TRUE_BYTE : 0x06, false, false);
}

void setNode1LightLevelToEPPROM(uint8_t lightLevel)
{
  EEPROM24C32.writeByte(101, lightLevel, false, false);
}
#endif 

#if defined NODE_1_RGBW
void setNode1WhiteChannel(uint8_t value)
{
  EEPROM24C32.writeByte(501, value, false, false);
}
#endif

#if defined NODE_1_RGBW || defined NODE_1_RGB
void setNode1RGBValues(uint8_t red, uint8_t green, uint8_t blue)
{
  EEPROM24C32.writeByte(502, red, false, false);
  EEPROM24C32.writeByte(503, green, false, false);
  EEPROM24C32.writeByte(504, blue, false, false);
}
#endif 

#if defined NODE_2_SINGLE
void saveNode2EnableToEPPROM(bool deviceEabled)
{
  EEPROM24C32.writeByte(200, deviceEabled ? TRUE_BYTE : 0x07, false, false);
}

void setNode2LightLevelToEPPROM(uint8_t lightLevel)
{
  EEPROM24C32.writeByte(201, lightLevel, false, false);
}
#endif 

#if defined NODE_3_SINGLE
void saveNode3EnableToEPPROM(bool deviceEabled)
{
  EEPROM24C32.writeByte(300, deviceEabled ? TRUE_BYTE : 0x08, false, false);
}

void setNode3LightLevelToEPPROM(uint8_t lightLevel)
{
  EEPROM24C32.writeByte(301, lightLevel, false, false);
}
#endif 

#if defined NODE_4_SINGLE
void saveNode4EnableToEPPROM(bool deviceEabled)
{
  EEPROM24C32.writeByte(400, deviceEabled ? TRUE_BYTE : 0x09, false, false);
}

void setNode4LightLevelToEPPROM(uint8_t lightLevel)
{
  EEPROM24C32.writeByte(401, lightLevel, false, false);
}
#endif 


/* #endregion */

/* #region  mysensors */

bool isPresentedToController = false;
void presentation() // MySensors
{
  sendSketchInfo(SKETCH_NAME, SOFTWARE_VERION);
  presentToControler();
  isPresentedToController = true;
}

void presentToControler()
{
#if defined NODE_1_RGBW || defined NODE_1_RGB || defined NODE_1_SINGLE 
  present(DIMMER_ID_1, S_DIMMER, "LED dimmer 1");
#endif

#if defined(NODE_1_RGBW)
  present(RGBW_ID, S_RGBW_LIGHT, "RGBW controller");
#endif

#if defined(NODE_1_RGB)
  present(RGBW_ID, S_RGB_LIGHT, "RGB controller");
#endif

#if defined NODE_2_SINGLE 
  present(DIMMER_ID_2, S_DIMMER, "LED dimmer 2");
#endif

#if defined NODE_3_SINGLE 
  present(DIMMER_ID_3, S_DIMMER, "LED dimmer 3");
#endif

#if defined NODE_4_SINGLE 
  present(DIMMER_ID_4, S_DIMMER, "LED dimmer 4");
#endif
}

void receive(const MyMessage &message) // MySensors
{
  if (message.isAck())
    return;

  if (message.sensor == DIMMER_ID && message.type == V_STATUS)
  {
    setDeviceEnabledFromControler(message.getBool());
    return;
  }

  if (message.sensor == DIMMER_ID && message.type == V_PERCENTAGE)
  {
    int val = atoi(message.data);
    if (val >= 0 && val <= 100)
    {
      setLightLevelFromControler(val);
    }
    return;
  }

#if defined(RGBW_MODE)
  if (message.sensor == RGBW_ID && message.type == V_RGBW)
  {
    unsigned long number = (unsigned long)strtoul(message.data, NULL, 16);
    uint8_t red = unsigned(number >> 24 & 0xFF);
    uint8_t green = unsigned(number >> 16 & 0xFF);
    uint8_t blue = unsigned(number >> 8 & 0xFF);
    uint8_t white = unsigned(number & 0xFF);

#if defined(MY_DEBUG)
    Serial.println("Otrzymano RGBW.");
    Serial.print("Decimal: ");
    Serial.println(number);

    Serial.print("Red: ");
    Serial.println(red);

    Serial.print("Green: ");
    Serial.println(green);

    Serial.print("Blue: ");
    Serial.println(blue);

    Serial.print("White: ");
    Serial.println(white);
#endif

    setRGBWvalueFromControler(red, green, blue, white);
    return;
  }
#endif

#if defined(RGB_MODE)
  if (message.sensor == RGBW_ID && message.type == V_RGB)
  {
    unsigned long number = (unsigned long)strtoul(message.data, NULL, 16);
    uint8_t red = unsigned(number >> 16 & 0xFF);
    uint8_t green = unsigned(number >> 8 & 0xFF);
    uint8_t blue = unsigned(number & 0xFF);

    setRGBValueFromControler(red, green, blue);
    return;
  }
#endif
}

#if defined NODE_1_RGBW || defined NODE_1_RGB || defined NODE_1_SINGLE 
void sendNode1EnabledStatus()
{
  mMessage.setSensor(DIMMER_ID_1);
  mMessage.setType(V_STATUS);
  send(mMessage.set(node1Enabled));
}

void sendNode1LightLevel()
{
   mMessage.setSensor(DIMMER_ID_1);
  mMessage.setType(V_PERCENTAGE);
  send(mMessage.set(node1LightLevel));
}
#endif

#if defined NODE_2_SINGLE 
void sendNode2EnabledStatus()
{
  mMessage.setSensor(DIMMER_ID_2);
  mMessage.setType(V_STATUS);
  send(mMessage.set(node2Enabled));
}

void sendNode2LightLevel()
{
   mMessage.setSensor(DIMMER_ID_2);
  mMessage.setType(V_PERCENTAGE);
  send(mMessage.set(node2LightLevel));
}
#endif

#if defined NODE_3_SINGLE 
void sendNode3EnabledStatus()
{
  mMessage.setSensor(DIMMER_ID_3);
  mMessage.setType(V_STATUS);
  send(mMessage.set(node3Enabled));
}

void sendNode3LightLevel()
{
   mMessage.setSensor(DIMMER_ID_3);
  mMessage.setType(V_PERCENTAGE);
  send(mMessage.set(node3LightLevel));
}
#endif

#if defined NODE_4_SINGLE 
void sendNode4EnabledStatus()
{
  mMessage.setSensor(DIMMER_ID_4);
  mMessage.setType(V_STATUS);
  send(mMessage.set(node4Enabled));
}

void sendNode4LightLevel()
{
   mMessage.setSensor(DIMMER_ID_4);
  mMessage.setType(V_PERCENTAGE);
  send(mMessage.set(node4LightLevel));
}
#endif

#if defined(NODE_1_RGBW)
void sendNode1RGBWColor()
{
  mMessage.setSensor(RGBW_ID);
  mMessage.setType(V_RGBW);

  char str[8];
  sprintf(&str[0], "%02x", channel2Level);
  sprintf(&str[2], "%02x", channel3Level);
  sprintf(&str[4], "%02x", channel4Level);
  sprintf(&str[6], "%02x", channel1Level);
  send(mMessage.set(str));
}
#endif

#if defined(NODE_1_RGB)
void sendNode1RGBColor()
{
  mMessage.setSensor(RGBW_ID);
  mMessage.setType(V_RGB);

  char str[6];
  sprintf(&str[0], "%02x", channel1Level);
  sprintf(&str[2], "%02x", channel2Level);
  sprintf(&str[4], "%02x", channel3Level);
  send(mMessage.set(str));
}
#endif

void sendAllMySensorsStatus()
{
  #if defined NODE_1_SINGLE || defined NODE_1_RGBW || defined NODE_1_RGB
  sendNode1EnabledStatus();
  sendNode1LightLevel();
#endif 

  #if defined NODE_1_RGBW
  sendNode1RGBWColor();
#endif 

    #if defined NODE_1_RGB
 sendNode1RGBColor();
    #endif 

      #if defined NODE_2_SINGLE
  sendNode2EnabledStatus();
  sendNode2LightLevel();
#endif 

 #if defined NODE_3_SINGLE
  sendNode3EnabledStatus();
  sendNode3LightLevel();
#endif 

 #if defined NODE_4_SINGLE
  sendNode4EnabledStatus();
  sendNode4LightLevel();
#endif 
}


void setDeviceEnabledFromControler(bool deviceEnbledToSet)
{
  node1Enabled = deviceEnbledToSet;

  saveNode1EnableToEPPROM(node1Enabled);

  if (node1Enabled && node1LightLevel == 0)
  {
    setLightLevelFromControler(STARTUP_LIGHT_LEVEL);
    sendAllMySensorsStatus();
    return;
  }

  calculate();
  if (node1Enabled)
  {
    sendAllMySensorsStatus();
  }
  else
  {
    sendNode1EnabledStatus();
  }
}

void setLightLevelFromControler(uint8_t lightLevel)
{
  node1LightLevel = lightLevel; // 0 -100

  setNode1LightLevelToEPPROM(node1LightLevel);

  if (node1LightLevel == 0 && node1Enabled)
  {
    setDeviceEnabledFromControler(false);
    sendNode1EnabledStatus();
    return;
  }

  if (node1LightLevel > 0 && !node1Enabled)
  {
    setDeviceEnabledFromControler(true);
    sendNode1EnabledStatus();
    return;
  }

  calculate();
  sendNode1LightLevel();
}

#if defined(RGBW_MODE)
void setRGBWvalueFromControler(uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
  channel1Level = white;
  channel2Level = red;
  channel3Level = green;
  channel4Level = blue;

  calculate();
  sendColor();

  setChannelValue(1, channel1Level);
  setChannelValue(2, channel2Level);
  setChannelValue(3, channel3Level);
  setChannelValue(4, channel4Level);
}
#endif

#if defined(RGB_MODE)
void setRGBValueFromControler(uint8_t red, uint8_t green, uint8_t blue)
{
  channel1Level = red;
  channel2Level = green;
  channel3Level = blue;

  calculate();
  sendColor();

  setChannelValue(1, channel1Level);
  setChannelValue(2, channel2Level);
  setChannelValue(3, channel3Level);
}
#endif
/* #endregion */

void calculate()
{
  if (node1Enabled)
  {
    unsigned int duty_1 = getChannel1Duty();
    unsigned int duty_2 = getChannel2Duty();
    unsigned int duty_3 = getChannel3Duty();
    unsigned int duty_4 = getChannel4Duty();

    // analogWrite
    //  pwm_set_duty(duty_1, 0);
    //  pwm_set_duty(duty_2, 1);
    //  pwm_set_duty(duty_3, 2);
    //  pwm_set_duty(duty_4, 3);
    //  pwm_start();

    setRelayStatus(duty_1 > 0 || duty_2 > 0 || duty_3 > 0 || duty_4 > 0);
  }
  else
  {
    setRelayStatus(false);

    // pwm_set_duty(0, 0);
    // pwm_set_duty(0, 1);
    // pwm_set_duty(0, 2);
    // pwm_set_duty(0, 3);
    // pwm_start();
  }
}

void setRelayStatus(bool enabled)
{
  digitalWrite(RELAY_PIN, enabled ? HIGH : LOW);
}

unsigned int getChannel1Duty()
{
  return map(node1WhiteChannel * node1LightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
}

unsigned int getChannel2Duty()
{
#if defined(RGBW_MODE) || defined(RGB_MODE)
  return map(channel2Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
#endif

#if defined(SINGLE_LED_MODE)
  return 0;
#endif
}

unsigned int getChannel3Duty()
{
#if defined(RGBW_MODE) || defined(RGB_MODE)
  return map(channel3Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
#endif

#if defined(SINGLE_LED_MODE)
  return 0;
#endif
}

unsigned int getChannel4Duty()
{
#if defined(RGBW_MODE)
  return map(channel4Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
#endif

#if defined(RGB_MODE) || defined(SINGLE_LED_MODE)
  return 0;
#endif

}

/* #region main functions */

void before()
{
  setAllPinsAnalog();

  Serial2.begin(9600);
  inicjalizePins();
  inicjalizeI2C();
  readSettingFromEPPROM();
}

void setup()
{

  calculate();
}

void loop()
{
  if (SCM.isStateChanged(isPresentedToController, 0))
  {
    sendAllMySensorsStatus();
  }
}

/* #endregion */
