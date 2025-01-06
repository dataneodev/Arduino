/*Instalacja

Zmienić w Bounce2.h

#if defined(ARDUINO_ARCH_STM32F1)
		pinMode(pin, (WiringPinMode)mode);
#else
		pinMode(pin, mode);
#endif


na 

#if defined(ARDUINO_ARCH_STM32F1)
		pinMode(pin, mode);
#else
		pinMode(pin, mode);
#endif

*/


/*
#define PA0                     PIN_A0
#define PA1                     PIN_A1
#define PA2                     PIN_A2
#define PA3                     PIN_A3
#define PA4                     PIN_A4
#define PA5                     PIN_A5
#define PA6                     PIN_A6
#define PA7                     PIN_A7
#define PA8                     8
#define PA9                     9
#define PA10                    10
#define PA11                    11
#define PA12                    12
#define PA13                    13
#define PA14                    14
#define PA15                    15
#define PB0                     PIN_A8
#define PB1                     PIN_A9
#define PB2                     18
#define PB3                     19
#define PB4                     20
#define PB5                     21
#define PB6                     22
#define PB7                     23
#define PB8                     24
#define PB9                     25
#define PB10                    26
#define PB11                    27
#define PB12                    28
#define PB13                    29
#define PB14                    30
#define PB15                    31
#define PC0                     PIN_A10
#define PC1                     PIN_A11
#define PC2                     PIN_A12
#define PC3                     PIN_A13
#define PC4                     PIN_A14
#define PC5                     PIN_A15
#define PC6                     38
#define PC7                     39
#define PC8                     40
#define PC9                     41
#define PC10                    42
#define PC11                    43
#define PC12                    44
#define PC13                    45
#define PC14                    46
#define PC15                    47
#define PD0                     48
#define PD1                     49
#define PD2                     50
#define PD3                     51
#define PD4                     52
#define PD5                     53
#define PD6                     54
#define PD7                     55
#define PD8                     56
#define PD9                     57
#define PD10                    58
#define PD11                    59
#define PD12                    60
#define PD13                    61
#define PD14                    62
#define PD15                    63
#define PE0                     64
#define PE1                     65
#define PE2                     66
#define PE3                     67
#define PE4                     68
#define PE5                     69
#define PE6                     70
#define PE7                     71
#define PE8                     72
#define PE9                     73
#define PE10                    74
#define PE11                    75
#define PE12                    76
#define PE13                    77
#define PE14                    78
#define PE15                    79
#define PF0                     80
#define PF1                     81
#define PF2                     82
#define PF3                     83
#define PF4                     84
#define PF5                     85
#define PF6                     PIN_A16
#define PF7                     PIN_A17
#define PF8                     PIN_A18
#define PF9                     PIN_A19
#define PF10                    PIN_A20
#define PF11                    91
#define PF12                    92
#define PF13                    93
#define PF14                    94
#define PF15                    95
#define PG0                     96
#define PG1                     97
#define PG2                     98
#define PG3                     99
#define PG4                     100
#define PG5                     101
#define PG6                     102
#define PG7                     103
#define PG8                     104
#define PG9                     105
#define PG10                    106
#define PG11                    107
#define PG12                    108
#define PG13                    109
#define PG14                    110
#define PG15                    111
*/



/* #region  user configuration */

#define EEPROM_RESET 0x63  // zmienić wartość aby zresetować ustawienia
/* #endregion  user configuration */

/* #region  const configuration */

//I
#define P_I_1 PB12
#define P_I_2 PB14
#define P_I_3 PD8
#define P_I_4 PD10
#define P_I_5 PD12
#define P_I_6 PD14
#define P_I_7 PG2
#define P_I_8 PG4

//II
#define P_II_1 PB13
#define P_II_2 PB15
#define P_II_3 PD9
#define P_II_4 PD11
#define P_II_5 PD13
#define P_II_6 PD15
#define P_II_7 PG3
#define P_II_8 PG5


//III
#define P_III_1 PA15
#define P_III_2 PC11
#define P_III_3 PD0
#define P_III_4 PD2
#define P_III_5 PD4
#define P_III_6 PD6
#define P_III_7 PG9
#define P_III_8 PG11

//IV
#define P_IV_1 PC10
#define P_IV_2 PC12
#define P_IV_3 PD1
#define P_IV_4 PD3
#define P_IV_5 PD5
#define P_IV_6 PD7
#define P_IV_7 PG10
#define P_IV_8 PG12

//V
#define P_V_1 PB11
#define P_V_2 PE15
#define P_V_3 PE13
#define P_V_4 PE11
#define P_V_5 PE9
#define P_V_6 PE7
#define P_V_7 PG0
#define P_V_8 PF14

//VI
#define P_VI_1 PB10
#define P_VI_2 PE14
#define P_VI_3 PE12
#define P_VI_4 PE10
#define P_VI_5 PE8
#define P_VI_6 PG1
#define P_VI_7 PF15
#define P_VI_8 PF13

//VII
#define P_VII_1 PB2
#define P_VII_2 PB0
#define P_VII_3 PC4
#define P_VII_4 PA6
#define P_VII_5 PA4
#define P_VII_6 PA2
#define P_VII_7 PA0
#define P_VII_8 PC2

//VIII
#define P_VIII_1 PB1
#define P_VIII_2 PC5
#define P_VIII_3 PA7
#define P_VIII_4 PA5
#define P_VIII_5 PA3
#define P_VIII_6 PA1
#define P_VIII_7 PC3
#define P_VIII_8 PC1

//IX
#define P_IX_1 PF9
#define P_IX_2 PF7
#define P_IX_3 PF5
#define P_IX_4 PF3
#define P_IX_5 PF1
#define P_IX_6 PC13
#define P_IX_7 PE5
#define P_IX_8 PE3

//X
#define P_X_1 PF8
#define P_X_2 PF6
#define P_X_3 PF4
#define P_X_4 PF2
#define P_X_5 PF0
#define P_X_6 PE6
#define P_X_7 PE4
#define P_X_8 PE2

//XI
#define P_XI_1 PA8
#define P_XI_2 PC8
#define P_XI_3 PC6
#define P_XI_4 PG7
#define P_XI_5 PG6
#define P_XI_6 PG8
#define P_XI_7 PC7
#define P_XI_8 PC9

//XII
#define P_XII_1 PB7
#define P_XII_2 PB5
#define P_XII_3 PB3
#define P_XII_4 PG14
#define P_XII_5 PG13
#define P_XII_6 PG15
#define P_XII_7 PB4
#define P_XII_8 PB6

#define P_C_I A9 //Connector I
#define P_C_II A10 //Connector II

#define ARDUINO_ARCH_STM32F1

// MY SENSORS
#define SOFTWARE_VERION "1.0"
#define SKETCH_NAME "M19_STM32F103ZE"

#define MY_GATEWAY_SERIAL
#define MY_BAUD_RATE 115200
#define MY_SERIALDEVICE SerialUSB
#define MY_NODE_ID 51  // id wezła dla my sensors
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC

// 24C32
#define SCL_24C32_PIN PB8
#define SDA_24C32_PIN PB9


/* #endregion */

/* #region Imports */
#include <MySensors.h>
#include <24C32.h>
#include <Wire.h>

EE EEPROM24C32;

//#include "C:\Programowanie\7.Projekty\5.Arduino\M_Library\RelayManager\RelayManager.h"
/* #endregion */

//RelayManager myRelayController = RelayManager( true); //controller

#include "C:\Programowanie\7.Projekty\5.Arduino\M_Library\SwitchManager\SwitchManager.h"

SwitchManager mySwitchManager = SwitchManager();


void inicjalizeI2C() {
  Wire.setSDA(SDA_24C32_PIN);
  Wire.setSCL(SCL_24C32_PIN);
  Wire.begin();
  Wire.setClock(400000);
  EEPROM24C32.begin(0x50, false);
}

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

/* #endregion  Power Optimalization */

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Switch Sensor Manager", "1.0");

  mySwitchManager.presentAllToControler();  //M4_MS_SwitchSensorManager
}


void before() {
  /* M4_MS_SwitchSensorManager */

  // I
  mySwitchManager.addSwitch(P_I_1, NORMAL_CLOSE, "P_I_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_I_2, NORMAL_CLOSE, "P_I_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_I_3, NORMAL_CLOSE, "P_I_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_I_4, NORMAL_CLOSE, "P_I_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_I_5, NORMAL_CLOSE, "P_I_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_I_6, NORMAL_CLOSE, "P_I_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_I_7, NORMAL_CLOSE, "P_I_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_I_8, NORMAL_CLOSE, "P_I_8");  // M4_MS_SwitchSensorManager

  // II
  mySwitchManager.addSwitch(P_II_1, NORMAL_CLOSE, "P_II_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_II_2, NORMAL_CLOSE, "P_II_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_II_3, NORMAL_CLOSE, "P_II_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_II_4, NORMAL_CLOSE, "P_II_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_II_5, NORMAL_CLOSE, "P_II_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_II_6, NORMAL_CLOSE, "P_II_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_II_7, NORMAL_CLOSE, "P_II_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_II_8, NORMAL_CLOSE, "P_II_8");  // M4_MS_SwitchSensorManager

  // III
  mySwitchManager.addSwitch(P_III_1, NORMAL_CLOSE, "P_III_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_III_2, NORMAL_CLOSE, "P_III_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_III_3, NORMAL_CLOSE, "P_III_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_III_4, NORMAL_CLOSE, "P_III_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_III_5, NORMAL_CLOSE, "P_III_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_III_6, NORMAL_CLOSE, "P_III_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_III_7, NORMAL_CLOSE, "P_III_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_III_8, NORMAL_CLOSE, "P_III_8");  // M4_MS_SwitchSensorManager

  // IV
  mySwitchManager.addSwitch(P_IV_1, NORMAL_CLOSE, "P_IV_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IV_2, NORMAL_CLOSE, "P_IV_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IV_3, NORMAL_CLOSE, "P_IV_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IV_4, NORMAL_CLOSE, "P_IV_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IV_5, NORMAL_CLOSE, "P_IV_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IV_6, NORMAL_CLOSE, "P_IV_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IV_7, NORMAL_CLOSE, "P_IV_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IV_8, NORMAL_CLOSE, "P_IV_8");  // M4_MS_SwitchSensorManager

  // V
  mySwitchManager.addSwitch(P_V_1, NORMAL_CLOSE, "P_V_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_V_2, NORMAL_CLOSE, "P_V_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_V_3, NORMAL_CLOSE, "P_V_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_V_4, NORMAL_CLOSE, "P_V_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_V_5, NORMAL_CLOSE, "P_V_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_V_6, NORMAL_CLOSE, "P_V_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_V_7, NORMAL_CLOSE, "P_V_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_V_8, NORMAL_CLOSE, "P_V_8");  // M4_MS_SwitchSensorManager

  // VI
  mySwitchManager.addSwitch(P_VI_1, NORMAL_CLOSE, "P_VI_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VI_2, NORMAL_CLOSE, "P_VI_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VI_3, NORMAL_CLOSE, "P_VI_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VI_4, NORMAL_CLOSE, "P_VI_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VI_5, NORMAL_CLOSE, "P_VI_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VI_6, NORMAL_CLOSE, "P_VI_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VI_7, NORMAL_CLOSE, "P_VI_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VI_8, NORMAL_CLOSE, "P_VI_8");  // M4_MS_SwitchSensorManager

  // VII
  mySwitchManager.addSwitch(P_VII_1, NORMAL_CLOSE, "P_VII_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VII_2, NORMAL_CLOSE, "P_VII_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VII_3, NORMAL_CLOSE, "P_VII_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VII_4, NORMAL_CLOSE, "P_VII_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VII_5, NORMAL_CLOSE, "P_VII_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VII_6, NORMAL_CLOSE, "P_VII_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VII_7, NORMAL_CLOSE, "P_VII_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VII_8, NORMAL_CLOSE, "P_VII_8");  // M4_MS_SwitchSensorManager

  // VIII
  mySwitchManager.addSwitch(P_VIII_1, NORMAL_CLOSE, "P_VIII_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VIII_2, NORMAL_CLOSE, "P_VIII_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VIII_3, NORMAL_CLOSE, "P_VIII_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VIII_4, NORMAL_CLOSE, "P_VIII_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VIII_5, NORMAL_CLOSE, "P_VIII_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VIII_6, NORMAL_CLOSE, "P_VIII_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VIII_7, NORMAL_CLOSE, "P_VIII_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_VIII_8, NORMAL_CLOSE, "P_VIII_8");  // M4_MS_SwitchSensorManager

  // IX
  mySwitchManager.addSwitch(P_IX_1, NORMAL_CLOSE, "P_IX_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IX_2, NORMAL_CLOSE, "P_IX_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IX_3, NORMAL_CLOSE, "P_IX_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IX_4, NORMAL_CLOSE, "P_IX_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IX_5, NORMAL_CLOSE, "P_IX_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IX_6, NORMAL_CLOSE, "P_IX_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IX_7, NORMAL_CLOSE, "P_IX_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_IX_8, NORMAL_CLOSE, "P_IX_8");  // M4_MS_SwitchSensorManager

  // X
  mySwitchManager.addSwitch(P_X_1, NORMAL_CLOSE, "P_X_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_X_2, NORMAL_CLOSE, "P_X_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_X_3, NORMAL_CLOSE, "P_X_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_X_4, NORMAL_CLOSE, "P_X_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_X_5, NORMAL_CLOSE, "P_X_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_X_6, NORMAL_CLOSE, "P_X_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_X_7, NORMAL_CLOSE, "P_X_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_X_8, NORMAL_CLOSE, "P_X_8");  // M4_MS_SwitchSensorManager

  // XI
  mySwitchManager.addSwitch(P_XI_1, NORMAL_CLOSE, "P_XI_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XI_2, NORMAL_CLOSE, "P_XI_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XI_3, NORMAL_CLOSE, "P_XI_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XI_4, NORMAL_CLOSE, "P_XI_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XI_5, NORMAL_CLOSE, "P_XI_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XI_6, NORMAL_CLOSE, "P_XI_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XI_7, NORMAL_CLOSE, "P_XI_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XI_8, NORMAL_CLOSE, "P_XI_8");  // M4_MS_SwitchSensorManager

  // XII
  mySwitchManager.addSwitch(P_XII_1, NORMAL_CLOSE, "P_XII_1");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XII_2, NORMAL_CLOSE, "P_XII_2");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XII_3, NORMAL_CLOSE, "P_XII_3");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XII_4, NORMAL_CLOSE, "P_XII_4");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XII_5, NORMAL_CLOSE, "P_XII_5");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XII_6, NORMAL_CLOSE, "P_XII_6");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XII_7, NORMAL_CLOSE, "P_XII_7");  // M4_MS_SwitchSensorManager
  mySwitchManager.addSwitch(P_XII_8, NORMAL_CLOSE, "P_XII_8");  // M4_MS_SwitchSensorManager


}

void setup() {
  // put your setup code here, to run once:
  inicjalizeI2C();
}

void loop() {
  // put your main code here, to run repeatedly:
  mySwitchManager.switchCheckState();  //M4_MS_SwitchSensorManager
}
