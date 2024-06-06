

/* Instalacja
Dodać board https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
Zainstalować board STM32 MCU based boards
Zainstalować  Arduino SAM Boards
Podmienić pliki w MySensors dla STM32F1 libraries\MySensors\hal\architecture\STM32F1\
*/

/*
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

#define RGBW_MODE
// #define RGB_MODE
// #define RGB_MODE_SINGLE
// #define SINGLE_LED_MODE_1
// #define SINGLE_LED_MODE_2
// #define SINGLE_LED_MODE_3
// #define SINGLE_LED_MODE_4

#define MIN_LIGHT_LEVEL 5     // minimalna jasnosc
#define MAX_LIGHT_LEVEL 100   // maksymalna jasnosc
#define STARTUP_LIGHT_LEVEL 5 // 0-100 początkowa jasnosc jak włączono sterownik a poziom jasnosci jest 0

/* #endregion */

/* #region  const configuration */
// MY SENSORS
#define SOFTWARE_VERION "1.0"

#if defined(RGBW_MODE)
#define SKETCH_NAME "RGBW_LED"
#endif

#if defined(RGB_MODE)
#define SKETCH_NAME "RGB_LED"
#endif

#if defined(SINGLE_LED_MODE)
#define SKETCH_NAME "SingleLED"
#endif

#define MY_NODE_ID 60 // id węzła my sensors - każdy sterownik musi miec inny numer
#define RGBW_ID 1
#define DIMMER_ID_1 1
#define DIMMER_ID_2 1
#define DIMMER_ID_3 1
#define DIMMER_ID_4 1

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

#include <MySensors.h>
#include <24C32.h>
#include <Wire.h>
/* #region  global variable */

EE EEPROM24C32;
MyMessage mMessage;
StateChangeManager SCM;

bool deviceEnabled = true;
uint8_t deviceLightLevel = 50; // 0 -100

#if defined(RGBW_MODE)
#define DEFAULT_CH2 255
#define DEFAULT_CH3 255
#define DEFAULT_CH4 255
#endif

#if defined(RGB_MODE)
#define DEFAULT_CH2 255
#define DEFAULT_CH3 255
#define DEFAULT_CH4 0
#endif

#if defined(SINGLE_LED_MODE)
#define DEFAULT_CH2 0
#define DEFAULT_CH3 0
#define DEFAULT_CH4 0
#endif

uint8_t channel1Level = 255; // 0-255
uint8_t channel2Level = DEFAULT_CH2;
uint8_t channel3Level = DEFAULT_CH3;
uint8_t channel4Level = DEFAULT_CH4;

// unsigned long lastMessageRevice;
/* #endregion */

/* #region  Power Optimalization */

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

/* #endregion  Power Optimalization */

// HAL_Init();
//   SystemClock_Config();
//   MX_GPIO_Init();
//   MX_I2C3_Init();
//   MX_SPI1_Init();
//   MX_USART1_UART_Init();

void before()
{
  setAllPinsAnalog();

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

  Serial2.begin(9600);
  inicjalizePins();
  inicjalizeI2C();
  readSettingFromEprom();
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

bool isPresentedToController = false;
void presentation() // MySensors
{
  sendSketchInfo(SKETCH_NAME, SOFTWARE_VERION);
  presentToControler();
  isPresentedToController = true;
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
/* #endregion */

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
#if defined(RGBW_MODE)
#define CHECK_NUMBER 0x65
#endif

#if defined(RGB_MODE)
#define CHECK_NUMBER 0x62
#endif

#if defined(SINGLE_LED_MODE)
#define CHECK_NUMBER 0x63
#endif

void readSettingFromEprom()
{
  if (EEPROM24C32.readByte(105) != CHECK_NUMBER)
  {
    setDefaultSetting();
    return;
  }

  deviceEnabled = EEPROM24C32.readByte(106) == 0x05;
  deviceLightLevel = EEPROM24C32.readByte(107);
  channel1Level = EEPROM24C32.readByte(108);
  channel2Level = EEPROM24C32.readByte(109);
  channel3Level = EEPROM24C32.readByte(110);
  channel4Level = EEPROM24C32.readByte(111);

#if defined(MY_DEBUG)
  Serial.println("Odczytano wartosci:");
  Serial.print("channel1Level:");
  Serial.println(channel1Level);
#endif
}

void setDefaultSetting()
{
#if defined(MY_DEBUG)
  Serial.println("Zapisuje domyslne ustawienia");
#endif
  EEPROM24C32.writeByte(105, CHECK_NUMBER, false, false);
  setDeviceEnableToEeprom(deviceEnabled);
  setLightLevelToEeprom(deviceLightLevel);
  setChannelValue(1, channel1Level);
  setChannelValue(2, channel2Level);
  setChannelValue(3, channel3Level);
  setChannelValue(4, channel4Level);
}

void setDeviceEnableToEeprom(bool deviceEabled)
{
  EEPROM24C32.writeByte(106, deviceEabled ? 0x05 : 0x06, false, false);
}

void setLightLevelToEeprom(uint8_t lightLevel)
{
  EEPROM24C32.writeByte(107, lightLevel, false, false);
}

void setChannelValue(uint8_t channelNo, uint8_t value)
{
  uint8_t channelAdress = 107 + channelNo;
  EEPROM24C32.writeByte(channelAdress, value, false, false);
}
/* #endregion */

/* #region  present to controler */
void presentToControler()
{
#if defined(SINGLE_LED_MODE)
  present(DIMMER_ID, S_DIMMER, "LED dimmer");
#endif

#if defined(RGBW_MODE)
  present(RGBW_ID, S_RGBW_LIGHT, "RGBW controller");
#endif

#if defined(RGB_MODE)
  present(RGBW_ID, S_RGB_LIGHT, "RGB controller");
#endif
}

void sendEnabledStatus()
{
  mMessage.setSensor(DIMMER_ID);
  mMessage.setType(V_STATUS);
  send(mMessage.set(deviceEnabled));
}

void sendPercentageStatus()
{
  mMessage.setType(V_PERCENTAGE);
  send(mMessage.set(deviceLightLevel));
}

void sendColor()
{
#if defined(RGBW_MODE)
#if defined(MY_DEBUG)
  Serial.println("Wysyłam RGBW.");
  Serial.print("Red: ");
  Serial.println(channel2Level);

  Serial.print("Green: ");
  Serial.println(channel3Level);

  Serial.print("Blue: ");
  Serial.println(channel4Level);

  Serial.print("White: ");
  Serial.println(channel1Level);
#endif

  mMessage.setSensor(RGBW_ID);
  mMessage.setType(V_RGBW);

  char str[8];
  sprintf(&str[0], "%02x", channel2Level);
  sprintf(&str[2], "%02x", channel3Level);
  sprintf(&str[4], "%02x", channel4Level);
  sprintf(&str[6], "%02x", channel1Level);
  send(mMessage.set(str));
#endif

#if defined(RGB_MODE)
  mMessage.setSensor(RGBW_ID);
  mMessage.setType(V_RGB);

  char str[6];
  sprintf(&str[0], "%02x", channel1Level);
  sprintf(&str[2], "%02x", channel2Level);
  sprintf(&str[4], "%02x", channel3Level);
  send(mMessage.set(str));
#endif
}

void sendAllMySensorsStatus()
{

  sendEnabledStatus();
  sendPercentageStatus();
  sendColor();
}
/* #endregion */

void setDeviceEnabledFromControler(bool deviceEnbledToSet)
{
  deviceEnabled = deviceEnbledToSet;

  setDeviceEnableToEeprom(deviceEnabled);

  if (deviceEnabled && deviceLightLevel == 0)
  {
    setLightLevelFromControler(STARTUP_LIGHT_LEVEL);
    sendAllMySensorsStatus();
    return;
  }

  calculate();
  if (deviceEnabled)
  {
    sendAllMySensorsStatus();
  }
  else
  {
    sendEnabledStatus();
  }
}

void setLightLevelFromControler(uint8_t lightLevel)
{
  deviceLightLevel = lightLevel; // 0 -100

  setLightLevelToEeprom(deviceLightLevel);

  if (deviceLightLevel == 0 && deviceEnabled)
  {
    setDeviceEnabledFromControler(false);
    sendEnabledStatus();
    return;
  }

  if (deviceLightLevel > 0 && !deviceEnabled)
  {
    setDeviceEnabledFromControler(true);
    sendEnabledStatus();
    return;
  }

  calculate();
  sendPercentageStatus();
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

void calculate()
{
  if (deviceEnabled)
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
  return map(channel1Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
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
