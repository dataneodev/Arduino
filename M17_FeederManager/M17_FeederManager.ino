/* #region  user configuration */

#define EEPROM_RESET 0x68  // zmienić wartość aby zresetować ustawienia

/* #endregion  user configuration */

/* #region  const configuration */
// RS485
HardwareSerial RS485Serial(PB7, PB6);
// MY SENSORS
#define SOFTWARE_VERION "1.0"
#define SKETCH_NAME "M17_FeederManager"

#define MY_NODE_ID 96  // id wezła dla my sensors
#define MY_PASSIVE_NODE
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC
#define MY_PASSIVE_NODE
#define MY_TRANSPORT_WAIT_READY_MS 1

#define MY_24V_STATUS_SENSOR_ID 1
#define MY_5_1V_STATUS_SENSOR_ID 2
#define MY_5_2V_STATUS_SENSOR_ID 3
#define MY_MOTION_DETECTION_STATUS_SENSOR_ID 4
#define MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID 5
#define MY_TIME_SENSOR_ID 6

#define ARDUINO_ARCH_STM32F1
#define MY_DISABLED_SERIAL             // manual configure Serial
#define MY_RS485                       // Enable RS485 transport layer
#define MY_RS485_DE_PIN PB5            // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600        // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL RS485Serial  //
#define MY_RS485_SOH_COUNT 6

// 24C32
#define SCL_PIN PB8
#define SDA_PIN PB9

#define MOTION_PIN PA0

#define OUT_5V_1 PA1
#define OUT_5V_2 PA2

#define OUT_24V PA3

#define SLEEP_START_UP 20000  // 20 sekund
#define SLEEP_MAX_TIME 10000  // 10 sekund
#define SLEEP_RS485_TIME 20  // 20 ms

#define BEFORE_CLOCK_ENABLE_TIME_PART 0.6
#define NEXT_CLOCK_ENABLE_TIME_PART 0.4
/* #endregion */

/* #region Imports */
#include <MySensors.h>
#include <24C32.h>
#include <Wire.h>
#include <STM32LowPower.h>
#include <STM32RTC.h>
#include "Storage.h"
#include "Timezone.h"

/* #endregion */

/* #region Class definition */
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
STM32RTC &rtc = STM32RTC::getInstance();

#if defined(RTC_SSR_SS)
static uint32_t atime = 678;
#else
static uint32_t atime = 1000;
#endif

TimeChangeRule myDST = { "EDT", Last, Sun, Mar, 2, 120 };  // Daylight time = UTC - 4 hours
TimeChangeRule mySTD = { "EST", Last, Sun, Nov, 2, 60 };   // Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);
volatile u_int32_t lastRequestTime;
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
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

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

  SCM.isStateChanged(false, 0);
  isPresentedToController = true;
}

void presentToControler() {
  present(MY_24V_STATUS_SENSOR_ID, S_BINARY, "24V Relay");
  present(MY_5_1V_STATUS_SENSOR_ID, S_BINARY, "5V 1 Relay");
  present(MY_5_2V_STATUS_SENSOR_ID, S_BINARY, "5V 2 Relay");
  present(MY_MOTION_DETECTION_STATUS_SENSOR_ID, S_BINARY, "Motion detection");
  present(MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID, S_BINARY, "Clock schedule");
  present(MY_TIME_SENSOR_ID, S_CUSTOM, "Internal clock");
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
  sendClockValue();
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

void sendClockValue() {
  mMessage.setSensor(MY_TIME_SENSOR_ID);
  mMessage.setType(V_VAR1);
  send(mMessage.set((uint32_t)myTZ.toLocal(rtc.getEpoch())));
}


void receive(const MyMessage &message)  // MySensors
{
  if (message.isAck())
    return;

  delaySleep(SLEEP_MAX_TIME);

  if (message.sensor == MY_24V_STATUS_SENSOR_ID && message.type == V_STATUS) {
    setEnable24VOutput(message.getBool(), true);
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_5_1V_STATUS_SENSOR_ID && message.type == V_STATUS) {
    setEnable5V_1Output(message.getBool(), true);
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_5_2V_STATUS_SENSOR_ID && message.type == V_STATUS) {
    setEnable5V_2Output(message.getBool(), true);
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_MOTION_DETECTION_STATUS_SENSOR_ID && message.type == V_STATUS) {

    setEnableMotionDetection(message.getBool());
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_MOTION_DETECTION_STATUS_SENSOR_ID && message.type == V_VAR1) {
    setMotionDetectedEnabledTime(message.getUInt());
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID && message.type == V_STATUS) {
    setEnableClockSchedule(message.getBool());
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID && message.type == V_VAR1) {

    setClockScheduleEnabledTime(message.getUInt());
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID && message.type == V_VAR2) {

    setClockScheduleIntervalHour(message.getByte());
    delaySleep(SLEEP_MAX_TIME);
    return;
  }

  if (message.sensor == MY_TIME_SENSOR_ID && message.type == V_VAR1) {
    setClock((time_t)message.getULong());
    delaySleep(SLEEP_MAX_TIME);
    return;
  }
}

void receiveTime(uint32_t ts) {
  delaySleep(SLEEP_MAX_TIME);
  rtc.setEpoch(myTZ.toUTC(ts), false);
  lastRequestTime = ts;
}
/* #endregion */

/* #region actions */
void setEnable24VOutput(bool enabled, bool save) {
  EEStorage.setEnable24VOutput(enabled, save);

  digitalWrite(OUT_24V, enabled ? HIGH : LOW);

  sendEnable24VOutputStatus();
}

void setEnable5V_1Output(bool enabled, bool save) {
  EEStorage.setEnable5V_1Output(enabled, save);

  digitalWrite(OUT_5V_1, enabled ? HIGH : LOW);

  sendEnable5V1Output();
}

void setEnable5V_2Output(bool enabled, bool save) {
  EEStorage.setEnable5V_2Output(enabled, save);

  digitalWrite(OUT_5V_2, enabled ? HIGH : LOW);

  sendEnable5V2Output();
}

void setEnableClockSchedule(bool enabled) {
  EEStorage.setEnableClockSchedule(enabled);
  sendEnableClockSchedule();
}

void setClockScheduleEnabledTime(uint16_t time) {
  EEStorage.setClockScheduleEnabledTime(time);
  sendClockScheduleEnabledTime();
}

void setClock(time_t time) {
  rtc.setEpoch(myTZ.toUTC(time));
  sendClockValue();
}

void setClockScheduleIntervalHour(uint8_t time) {
  EEStorage.setClockScheduleIntervalHour(time);
  sendClockScheduleIntervalHour();
}

void setEnableMotionDetection(bool enabled) {
  EEStorage.setEnableMotionDetection(enabled);
  sendEnableMotionDetection();
}

void setMotionDetectedEnabledTime(uint16_t time) {
  EEStorage.setMotionDetectedEnabledTime(time);
  sendMotionDetectedEnabledTime();
}
/* #endregion */

/* #region auto action */
// motion
volatile uint32_t motionEnableTime = 0;
void startMotionAutoAction() {
  motionEnableTime = myTZ.toLocal(rtc.getEpoch()) + EEStorage.motionDetectedEnabledTime();

  if (!isAllMotionAutoActionEnabled()) {
    setEnable5V_2Output(true, false);
  }
}

void stopMotionAutoAction(bool clockAutoActionInProgress) {
  if (isAnyMotionAutoActionEnabled() && !clockAutoActionInProgress) {
    setEnable5V_2Output(false, true);
  }

  clearMotionAutoAction();
}

void clearMotionAutoAction() {
  motionEnableTime = 0;
}

bool isAnyMotionAutoActionEnabled() {
  return EEStorage.enable5V2Output();
}

bool isAllMotionAutoActionEnabled() {
  return isAnyMotionAutoActionEnabled();
}

// clock
volatile uint32_t clockEnableTime = 0;

void startClockAutoAction() {
  clockEnableTime = myTZ.toLocal(rtc.getEpoch()) + EEStorage.clockScheduleEnabledTime();

  setEnable5V_1Output(true, false);
  setEnable5V_2Output(true, false);
  setEnable24VOutput(true, false);
  sendClockValue();
}

bool isAllClockActionEnabled() {
  return EEStorage.enable24VOutput() && EEStorage.enable5V1Output() && EEStorage.enable5V2Output();
}

bool isAnyClockActionEnabled() {
  return EEStorage.enable24VOutput() || EEStorage.enable5V1Output() || EEStorage.enable5V2Output();
}

void stopClockAutoAction(bool motionAutoActionInProgress) {
  if (isAnyClockActionEnabled()) {
    setEnable5V_1Output(false, true);

    if (!motionAutoActionInProgress) {
      setEnable5V_2Output(false, true);
    }

    setEnable24VOutput(false, true);
  }

  clearClockAutoAction();
}

void clearClockAutoAction() {
  clockEnableTime = 0;
}

void clearAllAutoActions() {
  clearClockAutoAction();
  clearMotionAutoAction();
}

// clock interrupt

void checkTimeRequest(uint32_t now) {
  if (lastRequestTime == 0) {
    lastRequestTime = now;
    return;
  }

  if (lastRequestTime > now) {
    lastRequestTime = now;
    return;
  }

  if (lastRequestTime + 1209600 < now) {  //14days
    lastRequestTime -= 14400;             //4h

    SCM.isStateChanged(!isPresentedToController, 0);

    requestTime();
  }
}

void clockInterrupt(void *) {
  uint32_t nowUnixtime = myTZ.toLocal(rtc.getEpoch());

  checkTimeRequest(nowUnixtime);

  if (EEStorage.enableMotionDetection() && !isAllMotionAutoActionEnabled()) {
    clearMotionAutoAction();
  }

  if (EEStorage.enableClockSchedule() && !isAllClockActionEnabled()) {
    clearClockAutoAction();
  }

  if (EEStorage.enableMotionDetection() && motionEnableTime != 0 && motionEnableTime < nowUnixtime) {
    bool clockAutoActionInProgress = clockEnableTime != 0 && clockEnableTime > nowUnixtime + 10;
    stopMotionAutoAction(clockAutoActionInProgress && isAnyClockActionEnabled());
  }


  if (EEStorage.enableClockSchedule() && clockEnableTime != 0 && clockEnableTime < nowUnixtime) {
    bool motionAutoActionInProgress = motionEnableTime != 0 && motionEnableTime > nowUnixtime + 10;
    stopClockAutoAction(motionAutoActionInProgress && isAnyMotionAutoActionEnabled());
  }

  if (EEStorage.enableClockSchedule() && clockEnableTime == 0 && isClockAutoActionTime(nowUnixtime)) {
    startClockAutoAction();
  }
}

void setNewRTCClockAwake() {
  if (!EEStorage.enableClockSchedule()) {
    return;
  }

  time_t now = myTZ.toLocal(rtc.getEpoch());
  time_t sleepTime = getSleepTime(now);
  rtc.setAlarmEpoch(myTZ.toUTC(sleepTime), rtc.MATCH_YYMMDDHHMMSS);
}

time_t getSleepTime(time_t now) {
  time_t next = now;
  byte i = 0;

  while (i <= 24) {
    i += 1;
    next += 3600;

    if (isHourHandledBySchedule(hour(next)) && !isClockAutoActionTimeCompare(next, now)) {
      break;
    }
  }

  time_t awake = getTimeAtStartHour(next) - (BEFORE_CLOCK_ENABLE_TIME_PART * (float)EEStorage.clockScheduleEnabledTime());

  if (awake - now < 10) {
    awake = now + 10;
  }

  return awake;
}

bool isClockAutoActionTime(time_t now) {
  if (!EEStorage.enableClockSchedule()) {
    return false;
  }

  if (isClockAutoActionTimeCompare(now, now)) {
    return true;
  }

  if (isClockAutoActionTimeCompare(now + 3600, now)) {
    return true;
  }

  return false;
}

bool isClockAutoActionTimeCompare(time_t test, time_t now) {
  if (!isHourHandledBySchedule(hour(test))) {
    return false;
  }

  time_t startHour = getTimeAtStartHour(test);

  time_t startScheduleTime = startHour - (BEFORE_CLOCK_ENABLE_TIME_PART * (float)EEStorage.clockScheduleEnabledTime());
  if (now < startScheduleTime) {
    return false;
  }

  time_t endScheduleTime = startHour + (NEXT_CLOCK_ENABLE_TIME_PART * (float)EEStorage.clockScheduleEnabledTime());
  if (now > endScheduleTime) {
    return false;
  }

  return true;
}

time_t getTimeAtStartHour(time_t now) {
  struct tm tmnow;
  gmtime_r(&now, &tmnow);
  now -= (tmnow.tm_sec + tmnow.tm_min * 60);
  return now;
}

bool isHourHandledBySchedule(uint8_t hour) {
  uint8_t check = 0;

  while (check < 24) {
    if (check == hour) {
      return true;
    }
    check += EEStorage.clockScheduleIntervalHour();
  }
  return false;
}

/* #endregion */

/* #region sleep */
volatile int32_t sleepWaitTime;
volatile u_int32_t lastNow;

volatile bool canSleep;
volatile bool serialAwake;

void delaySleep(int32_t delay) {
  serialAwake = false;
  canSleep = false;

  if (sleepWaitTime > delay) {
    return;
  }

  sleepWaitTime = delay;
}

void compensateSleepDelay() {
  if (canSleep) {
    return;
  }

  if (EEStorage.enable24VOutput()) {
    return;
  }

  if (EEStorage.enable5V1Output()) {
    return;
  }

  if (EEStorage.enable5V2Output()) {
    return;
  }

  u_int32_t now = millis();

  if (lastNow == 0) {
    lastNow = now;
    return;
  }

  if (lastNow > now) {
    lastNow = now;
    return;
  }

  sleepWaitTime -= (now - lastNow);

  lastNow = now;

  if (sleepWaitTime <= 0) {
    canSleep = true;
    sleepWaitTime = 0;
    return;
  }
}

void serialWakeup() {
  delaySleep(SLEEP_RS485_TIME);
  serialAwake = true;
}

void gpioAwakeInterrupt() {

  buttonInterrupt();
}

void buttonInterrupt() {
  if (!EEStorage.enableMotionDetection()) {
    return;
  }

  if (digitalRead(MOTION_PIN) != HIGH) {
    return;
  }

  delaySleep(SLEEP_MAX_TIME);

  startMotionAutoAction();
}

void alarmAwake(void *data) {
  if (!EEStorage.enableClockSchedule()) {
    return;
  }

  if (isAllClockActionEnabled()) {
    return;
  }

  delaySleep(SLEEP_MAX_TIME);
}
/* #endregion */

/* #region main functions */
void before() {
  delaySleep(SLEEP_START_UP);

  setAllPinsAnalog();
  disableClocks();

  RS485Serial.begin(9600);

  inicjalizePins();
  inicjalizeI2C();
  EEStorage.Inicjalize();
  updatePinsState();
}


void setup() {
  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.begin();

  rtc.attachSecondsInterrupt(clockInterrupt);
  attachInterrupt(digitalPinToInterrupt(MOTION_PIN), buttonInterrupt, RISING);

  LowPower.begin();

  LowPower.attachInterruptWakeup(MOTION_PIN, gpioAwakeInterrupt, RISING, SLEEP_MODE);
  LowPower.attachInterruptWakeup(PB7, serialWakeup, RISING, SLEEP_MODE);
  //LowPower.enableWakeupFrom(&RS485Serial, serialWakeup);
  LowPower.enableWakeupFrom(&rtc, alarmAwake, &atime);
  delaySleep(SLEEP_START_UP);
}


void loop() {
  if (SCM.isStateChanged(isPresentedToController, 0)) {
    delaySleep(SLEEP_MAX_TIME);
    sendAllMySensorsStatus();
    requestTime();
    delaySleep(SLEEP_MAX_TIME);
  }

  if (canSleep) {
    if (!serialAwake) {      
      clearAllAutoActions();
      setNewRTCClockAwake();
    }

    RS485Serial.flush();
    LowPower.deepSleep();  // sleep
    RS485Serial.flush();

  } else {
    compensateSleepDelay();
  }
}

/* #endregion */