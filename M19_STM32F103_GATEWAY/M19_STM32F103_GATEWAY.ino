/*  instalacja 

Podmienić na PRODUCT ID: 0x5741 w pliku: C:\Users\Smith\AppData\Local\Arduino15\packages\STMicroelectronics\hardware\stm32\2.9.0\boards.txt

GenF1.vid.0=0x0483
GenF1.pid.0=0x5740

na

GenF1.vid.0=0x0483
GenF1.pid.0=0x5741

Podmienić pliki MySensors hal/architecture/STM32F1 z lib(nie działa odczyć i zapis ustawień Ms do EEPROMu)
DO gatwaya musi by oryginał STM32F103, klon CH32F103 nie działa poprawnie USB
Board: STM32 MCU based boards -> https://github.com/stm32duino -> 2.10.1
Generic STM32F1 Series -> Generic F103C8Tx
Włączyć w opcjach USB support CDC (generic Serial)
Zainstalować sterowniki C:\Programowanie\7.Projekty\5.Arduino\M_Library\ch32\Arduino_STM32-master\Arduino_STM32-master\drivers\win\install_drivers.bat
*/


/* #region  user configuration */

//#define USE_24C32
#define EEPROM_RESET 0x63  // zmienić wartość aby zresetować ustawienia
/* #endregion  user configuration */

/* #region  const configuration */
// RS485
HardwareSerial RS485Serial(PA3, PA2);
// MY SENSORS
#define SOFTWARE_VERION "1.0"
#define SKETCH_NAME "M19_STM32F103_GATEWAY"

#define MY_GATEWAY_SERIAL
#define MY_BAUD_RATE 115200
#define MY_SERIALDEVICE SerialUSB
#define MY_NODE_ID 50  // id wezła dla my sensors
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC

#define ARDUINO_ARCH_STM32F1
#define MY_RS485                       // Enable RS485 transport layer
#define MY_RS485_DE_PIN PA1            // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600        // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL RS485Serial  //
#define MY_RS485_SOH_COUNT 12
#define MY_DEFAULT_RX_LED_PIN PC13

// 24C32
#define SCL_24C32_PIN PB8
#define SDA_24C32_PIN PB9

#define SCL_PIN PB10
#define SDA_PIN PB11

#define GP_1 PB12
#define GP_2 PB13

#define GP_3 PB15
#define GP_4 PA8

#define GP_5 PA9   //TX1
#define GP_6 PA10  //RX1

#define GP_7 PA15
#define GP_8 PB3

#define GP_9 PB4
#define GP_10 PB5

#define GP_11 PB6
#define GP_12 PB7
/* #endregion */

/* #region Imports */
#include <MySensors.h>

#if defined USE_24C32
#include <24C32.h>
#endif

#include <Wire.h>
/* #endregion */

#if defined USE_24C32
void inicjalizeI2C() {
  Wire.setSDA(SDA_24C32_PIN);
  Wire.setSCL(SCL_24C32_PIN);
  Wire.begin();
  Wire.setClock(400000);
}
#endif


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


void before() {

  setAllPinsAnalog();
  disableClocks();

  RS485Serial.begin(9600);

#if defined USE_24C32
  inicjalizeI2C();
#endif
}


void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}
