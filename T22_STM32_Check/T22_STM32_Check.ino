
// MY SENSORS
#define SOFTWARE_VERION "1.0"
#define SKETCH_NAME "M17_FeederManager"

#define I2C1_REMAP 1

// RS485
HardwareSerial KKKKKK(PB7, PB6);

#define MY_NODE_ID 80  // id węzła my sensors - każdy sterownik musi miec inny numer

#define MY_24V_STATUS_SENSOR_ID 1
#define MY_5_1V_STATUS_SENSOR_ID 2
#define MY_5_2V_STATUS_SENSOR_ID 3
#define MY_MOTION_DETECTION_STATUS_SENSOR_ID 4
#define MY_CLOCK_SCHEDULE_STATUS_SENSOR_ID 5
#define MY_TIME_SENSOR_ID 6

#define ARDUINO_ARCH_STM32F1
#define MY_DISABLED_SERIAL         // manual configure Serial
#define MY_RS485                   // Enable RS485 transport layer
#define MY_RS485_DE_PIN PB5        // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600    // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL KKKKKK  //
#define MY_RS485_SOH_COUNT 6
#define MY_TRANSPORT_WAIT_READY_MS 1

//#include <STM32LowPower.h>
#include <MySensors.h>

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
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
/* #region main functions */

#include <Wire.h>
#include <SPI.h>
void before() {
//LL_GPIO_AF_EnableRemap_I2C1();
//LL_GPIO_AF_EnableRemap_USART1();
 // afio_remap(AFIO_REMAP_USART1);
 //enableDebugPorts();

//KKKKKK.setRx(PB7);
//KKKKKK.setTx(PB6);
KKKKKK.begin(9600);


// Serial2.setRx(PA3);
// Serial2.setTx(PA2);
//   Serial2.begin(9600);

}

void presentation()  // MySensors
{
  sendSketchInfo(SKETCH_NAME, SOFTWARE_VERION);
 present(1, S_BINARY, "24V Relay");

}



void setup() {
// Serial.begin(115200);
//   while (!Serial){ delay(100); }
//   Serial.println("START!");


  // GPIO_InitTypeDef GPIO_InitStruct;
  // /* Configure all GPIO as analog to reduce current consumption on non used IOs */
  // /* Enable GPIOs clock */
  // __HAL_RCC_GPIOA_CLK_ENABLE();
  // __HAL_RCC_GPIOB_CLK_ENABLE();
  // __HAL_RCC_GPIOC_CLK_ENABLE();
  // __HAL_RCC_GPIOD_CLK_ENABLE();
  // __HAL_RCC_GPIOE_CLK_ENABLE();

  // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  // GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Pin = GPIO_PIN_All;
  // HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  // HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  // HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // GPIO_InitStruct.Pin = GPIO_PIN_All ^ GPIO_PIN_5; // with or without : same current 

  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // /* Disable GPIOs clock */

  // __HAL_RCC_GPIOB_CLK_DISABLE();
  // __HAL_RCC_GPIOC_CLK_DISABLE();
  // __HAL_RCC_GPIOD_CLK_DISABLE();
  // __HAL_RCC_GPIOE_CLK_DISABLE();
 
pinMode(PC13, OUTPUT);
pinMode(PB7, OUTPUT);

digitalWrite(PC13, LOW);


    delay(2000);
    digitalWrite(PC13, HIGH);
    delay(2000);
    //LowPower.deepSleep();
    digitalWrite(PC13, HIGH);
   // LowPower.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
    

  //   Serial.println("------------------------------------");
  // Serial.printf("SYCLK= %dMHz\n", HAL_RCC_GetSysClockFreq()/1000000);
 
  // Serial.printf("HCLK = %dMHz\n", HAL_RCC_GetHCLKFreq()/1000000);
  // Serial.printf("APB1 = %dMHz\n", HAL_RCC_GetPCLK1Freq()/1000000);
  // Serial.printf("APB2 = %dMHz\n", HAL_RCC_GetPCLK2Freq()/1000000);
  // Serial.println("------------------------------------");
  // Serial.println();
  // delay(5000);
}
