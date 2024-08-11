#include <STM32LowPower.h>

void setup() {

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


    LowPower.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(PC13, LOW);
    digitalWrite(PB7, LOW);

    delay(2000);
    digitalWrite(PC13, HIGH);
    delay(2000);
    LowPower.deepSleep();
    digitalWrite(PC13, HIGH);
}
