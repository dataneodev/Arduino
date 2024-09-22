/* #region  user configuration */

#define EEPROM_RESET 0x68  // zmienić wartość aby zresetować ustawienia

/* #endregion  user configuration */

/* #region  const configuration */
// MY SENSORS
#define SOFTWARE_VERION "1.0"
#define SKETCH_NAME "M17_FeederManager"

// RS485
HardwareSerial Serial2(USART2);

#define MY_NODE_ID 80  // id węzła my sensors - każdy sterownik musi miec inny numer

#define MY_24V_STATUS_SENSOR_ID 1
#define MY_5_1V_STATUS_SENSOR_ID 2
#define MY_5_2V_STATUS_SENSOR_ID 3
#define MY_MOTION_DETECTION_STATUS_SENSOR_ID 4
#define MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID 5

#define ARDUINO_ARCH_STM32F1
#define MY_DISABLED_SERIAL         // manual configure Serial
#define MY_RS485                   // Enable RS485 transport layer
#define MY_RS485_DE_PIN PB5        // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600    // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL Serial2  //
#define MY_RS485_SOH_COUNT 6
#define MY_TRANSPORT_WAIT_READY_MS 1

// 24C32
#define SCL_PIN PB8
#define SDA_PIN PB9

#define MOTION_PIN PA0
#define CLOCK_PIN PB4

#define OUT_5V_1 PC15
#define OUT_5V_2 PC14

#define OUT_24V PC13

#define SLEEP_START_UP 30000  // 10 sekund
#define SLEEP_MAX_TIME 10000  // 10 sekund
#define SLEEP_RS485_TIME 5   //5 ms

/* #endregion */

/* #region Imports */
#include <MySensors.h>
#include <24C32.h>
#include <Wire.h>
#include <STM32LowPower.h>
#include "Storage.h"

#import <DS3231.h>
/* #endregion */

/* #region Class definition */

class StateTime {
private:
  unsigned long startState;

public:
  void stateStart() {
    startState = millis();
  }

  bool isElapsed(unsigned long elapsed) {
    unsigned long current = millis();

    if (startState > current) {
      startState = current;
      return false;
    }

    if (startState + elapsed < current) {
      return true;
    }

    return false;
  }
};

class StateChangeManager {
private:
  bool _states[10];

public:
  bool isStateChanged(bool state, int index) {
    bool changed = _states[index] != state;
    _states[index] = state;

    return changed;
  }
};
/* #endregion */

/* #region  global variable */
MyMessage mMessage;
StateChangeManager SCM;
EE EEPROM24C32;
Storage EEStorage(&EEPROM24C32);
DS3231 myRTC;  // Set up access to the DS3231
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
// extern "C" void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//   RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
//   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//   RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
//   RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//   RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
//   PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
//   PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
//   if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

void setAllPinsAnalog(void) {
  GPIO_InitTypeDef GPIO_InitStruct = { 0, 0, 0, 0 };

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

void disableClocks() {
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
void inicjalizePins() {

  pinMode(OUT_5V_1, OUTPUT);
  digitalWrite(OUT_5V_1, LOW);

  pinMode(OUT_5V_2, OUTPUT);
  digitalWrite(OUT_5V_2, LOW);

  pinMode(OUT_24V, OUTPUT);
  digitalWrite(OUT_24V, LOW);

  pinMode(MY_RS485_DE_PIN, OUTPUT);

  pinMode(CLOCK_PIN, INPUT_PULLUP);
  pinMode(MOTION_PIN, INPUT);
}

void updatePinsState() {
  digitalWrite(OUT_5V_1, EEStorage.enable5V1Output() ? HIGH : LOW);
  digitalWrite(OUT_5V_2, EEStorage.enable5V2Output() ? HIGH : LOW);
  digitalWrite(OUT_24V, EEStorage.enable24VOutput() ? HIGH : LOW);
}

void inicjalizeI2C() {
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);
}
/* #endregion */

/* #region  mysensors */
bool isPresentedToController = false;
void presentation()  // MySensors
{
  sendSketchInfo(SKETCH_NAME, SOFTWARE_VERION);
  presentToControler();
  isPresentedToController = true;
}

void presentToControler() {
  present(MY_24V_STATUS_SENSOR_ID, S_BINARY, "24V Relay");
  present(MY_5_1V_STATUS_SENSOR_ID, S_BINARY, "5V 1 Relay");
  present(MY_5_2V_STATUS_SENSOR_ID, S_BINARY, "5V 2 Relay");
  present(MY_MOTION_DETECTION_STATUS_SENSOR_ID, S_BINARY, "Motion detection");
  present(MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID, S_BINARY, "Clock schedule");
}

void sendAllMySensorsStatus() {
  sendEnable24VOutputStatus();
  sendEnable5V1Output();
  sendEnable5V2Output();

  sendEnableMotionDetection();
  sendMotionDetectedEnabledTime();

  sendEnableClockSchedule();
  sendClockScheduleEnabledTime();
  sendClockScheduleIntervalHour();
}

void sendEnable24VOutputStatus() {
  mMessage.setSensor(MY_24V_STATUS_SENSOR_ID);
  mMessage.setType(V_STATUS);
  send(mMessage.set(EEStorage.enable24VOutput()));
}

void sendEnable5V1Output() {
  mMessage.setSensor(MY_5_1V_STATUS_SENSOR_ID);
  mMessage.setType(V_STATUS);
  send(mMessage.set(EEStorage.enable5V1Output()));
}

void sendEnable5V2Output() {
  mMessage.setSensor(MY_5_2V_STATUS_SENSOR_ID);
  mMessage.setType(V_STATUS);
  send(mMessage.set(EEStorage.enable5V2Output()));
}

void sendEnableMotionDetection() {
  mMessage.setSensor(MY_MOTION_DETECTION_STATUS_SENSOR_ID);
  mMessage.setType(V_STATUS);
  send(mMessage.set(EEStorage.enableMotionDetection()));
}

void sendMotionDetectedEnabledTime() {
  mMessage.setSensor(MY_MOTION_DETECTION_STATUS_SENSOR_ID);
  mMessage.setType(V_VAR1);
  send(mMessage.set(EEStorage.motionDetectedEnabledTime()));
}

void sendEnableClockSchedule() {
  mMessage.setSensor(MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID);
  mMessage.setType(V_STATUS);
  send(mMessage.set(EEStorage.enableClockSchedule()));
}

void sendClockScheduleEnabledTime() {
  mMessage.setSensor(MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID);
  mMessage.setType(V_VAR1);
  send(mMessage.set(EEStorage.clockScheduleEnabledTime()));
}

void sendClockScheduleIntervalHour() {
  mMessage.setSensor(MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID);
  mMessage.setType(V_VAR2);
  send(mMessage.set(EEStorage.clockScheduleIntervalHour()));
}

void receive(const MyMessage &message)  // MySensors
{
  if (message.isAck())
    return;

  delaySleep(SLEEP_MAX_TIME);

  if (message.sensor == MY_24V_STATUS_SENSOR_ID && message.type == V_STATUS) {
    setEnable24VOutput(message.getBool(), false);
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_5_1V_STATUS_SENSOR_ID && message.type == V_STATUS) {
    setEnable5V_1Output(message.getBool(), false);
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_5_2V_STATUS_SENSOR_ID && message.type == V_STATUS) {
    setEnable5V_2Output(message.getBool(), false);
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_MOTION_DETECTION_STATUS_SENSOR_ID && message.type == V_STATUS) {
    setEnableMotionDetection(message.getBool(), false);
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_MOTION_DETECTION_STATUS_SENSOR_ID && message.type == V_VAR1) {
    setMotionDetectedEnabledTime(message.getUInt(), false);
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID && message.type == V_STATUS) {
    setEnableClockSchedule(message.getBool(), false);
    delaySleep(SLEEP_MAX_TIME);
    return;
  }


  if (message.sensor == MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID && message.type == V_VAR1) {
    setClockScheduleEnabledTime(message.getUInt(), false);
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID && message.type == V_VAR2) {
    setClockScheduleIntervalHour(message.getByte(), false);
    delaySleep(SLEEP_MAX_TIME);
    return;
  }
}

/* #endregion */


/* #region actions */
void setEnable24VOutput(bool enabled, bool onlyDataSave) {
  EEStorage.setEnable24VOutput(enabled);

  if (onlyDataSave) {
    return;
  }

  digitalWrite(OUT_24V, enabled ? HIGH : LOW);

  sendEnable24VOutputStatus();
}

void setEnable5V_1Output(bool enabled, bool onlyDataSave) {
  EEStorage.setEnable5V_1Output(enabled);

  if (onlyDataSave) {
    return;
  }

  digitalWrite(OUT_5V_1, enabled ? HIGH : LOW);

  sendEnable5V1Output();
}

void setEnable5V_2Output(bool enabled, bool onlyDataSave) {
  EEStorage.setEnable5V_2Output(enabled);

  if (onlyDataSave) {
    return;
  }

  digitalWrite(OUT_5V_2, enabled ? HIGH : LOW);

  sendEnable5V2Output();
}

void setEnableClockSchedule(bool enabled, bool onlyDataSave) {
  EEStorage.setEnableClockSchedule(enabled);

  if (onlyDataSave) {
    return;
  }

  sendEnableClockSchedule();
}

void setClockScheduleEnabledTime(uint16_t time, bool onlyDataSave) {
  EEStorage.setClockScheduleEnabledTime(time);

  if (onlyDataSave) {
    return;
  }

  sendClockScheduleEnabledTime();
}

void setClockScheduleIntervalHour(uint8_t time, bool onlyDataSave) {
  EEStorage.setClockScheduleIntervalHour(time);

  if (onlyDataSave) {
    return;
  }

  sendClockScheduleIntervalHour();
}


void setEnableMotionDetection(bool enabled, bool onlyDataSave) {
  EEStorage.setEnableMotionDetection(enabled);

  if (onlyDataSave) {
    return;
  }

  sendEnableMotionDetection();
}

void setMotionDetectedEnabledTime(uint16_t time, bool onlyDataSave) {
  EEStorage.setMotionDetectedEnabledTime(time);

  if (onlyDataSave) {
    return;
  }

  sendMotionDetectedEnabledTime();
}
/* #endregion */

/* #region sleep */
volatile u_int32_t sleepWaitTime;
volatile u_int32_t sleepSetTime;
volatile bool canSleep;

void delaySleep(u_int32_t delay) {
  u_int32_t now = millis();

  canSleep = false;
  sleepSetTime = now;
  sleepWaitTime = now + delay;
}

void compensateSleepDelay() {
  if (canSleep) {
    return;
  }

  u_int32_t now = millis();

  if (sleepSetTime > now) {
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (sleepWaitTime < now) {
    canSleep = true;
    return;
  }
}


void buttonInterrupt() {
  delaySleep(SLEEP_MAX_TIME);
}

void serialWakeup() {
  delaySleep(SLEEP_RS485_TIME);
}

void clockInterrupt() {
  setEnable24VOutput(!EEStorage.enable24VOutput(), false);
}
/* #endregion */


/* #region main functions */
void before() {
  setAllPinsAnalog();
  disableClocks();
  delaySleep(SLEEP_START_UP);

  Serial2.begin(9600);

  inicjalizePins();
  inicjalizeI2C();
  EEStorage.Inicjalize();
  updatePinsState();
}

void setup() {
  myRTC.enable32kHz(false);
  myRTC.enableOscillator(true, false, 0);
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), clockInterrupt, FALLING);

  LowPower.begin();
  LowPower.attachInterruptWakeup(MOTION_PIN, buttonInterrupt, CHANGE, SLEEP_MODE);
  LowPower.attachInterruptWakeup(PA3, serialWakeup, CHANGE, SLEEP_MODE);

 // LowPower.enableWakeupFrom(&Serial2, serialWakeup);
}

void loop() {
  if (SCM.isStateChanged(isPresentedToController, 0)) {
    delaySleep(SLEEP_MAX_TIME);
    sendAllMySensorsStatus();
    requestTime();
    delaySleep(SLEEP_MAX_TIME);
  }


  if (canSleep) {
    myRTC.enableOscillator(false, false, 0);
    detachInterrupt(digitalPinToInterrupt(CLOCK_PIN));
    //Wire.end();
    Serial2.flush();
    LowPower.deepSleep();
    Serial2.flush();

    //Wire.begin();
    attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), clockInterrupt, FALLING);
    myRTC.enableOscillator(true, false, 0);
  } else {
    compensateSleepDelay();
  }
}

/* #endregion */