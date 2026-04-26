/*Instalacja

- Podmienić na PRODUCT ID: 0x5742 w pliku: C:\Users\Smith\AppData\Local\Arduino15\packages\STMicroelectronics\hardware\stm32\2.9.0\boards.txt

GenF1.vid.0=0x0483
GenF1.pid.0=0x5740

na

GenF1.vid.0=0x0483
GenF1.pid.0=0x5742


- Zmienić w Bounce2.h

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

Zmienić w pliku C:\Programowanie\7.Projekty\5.Arduino\libraries\MySensors\core\MySensorsCore.h
//GPIO_Gateway 4 - 103

z ->
#define GATEWAY_ADDRESS					((uint8_t)0)

na ->
#define GATEWAY_ADDRESS					((uint8_t)103)
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

#define P_C_I A9    //Connector I
#define P_C_II A10  //Connector II

// GRUPA I
#define P_I_1_GPIO GPIOB
#define P_I_1_IDR GPIO_IDR_IDR12
#define P_I_2_GPIO GPIOB
#define P_I_2_IDR GPIO_IDR_IDR14
#define P_I_3_GPIO GPIOD
#define P_I_3_IDR GPIO_IDR_IDR8
#define P_I_4_GPIO GPIOD
#define P_I_4_IDR GPIO_IDR_IDR10
#define P_I_5_GPIO GPIOD
#define P_I_5_IDR GPIO_IDR_IDR12
#define P_I_6_GPIO GPIOD
#define P_I_6_IDR GPIO_IDR_IDR14
#define P_I_7_GPIO GPIOG
#define P_I_7_IDR GPIO_IDR_IDR2
#define P_I_8_GPIO GPIOG
#define P_I_8_IDR GPIO_IDR_IDR4

// GRUPA II
#define P_II_1_GPIO GPIOB
#define P_II_1_IDR GPIO_IDR_IDR13
#define P_II_2_GPIO GPIOB
#define P_II_2_IDR GPIO_IDR_IDR15
#define P_II_3_GPIO GPIOD
#define P_II_3_IDR GPIO_IDR_IDR9
#define P_II_4_GPIO GPIOD
#define P_II_4_IDR GPIO_IDR_IDR11
#define P_II_5_GPIO GPIOD
#define P_II_5_IDR GPIO_IDR_IDR13
#define P_II_6_GPIO GPIOD
#define P_II_6_IDR GPIO_IDR_IDR15
#define P_II_7_GPIO GPIOG
#define P_II_7_IDR GPIO_IDR_IDR3
#define P_II_8_GPIO GPIOG
#define P_II_8_IDR GPIO_IDR_IDR5

// GRUPA III
#define P_III_1_GPIO PA15_GPIO
#define P_III_1_IDR GPIO_IDR_IDR15
#define P_III_2_GPIO GPIOC
#define P_III_2_IDR GPIO_IDR_IDR11
#define P_III_3_GPIO GPIOD
#define P_III_3_IDR GPIO_IDR_IDR0
#define P_III_4_GPIO GPIOD
#define P_III_4_IDR GPIO_IDR_IDR2
#define P_III_5_GPIO GPIOD
#define P_III_5_IDR GPIO_IDR_IDR4
#define P_III_6_GPIO GPIOD
#define P_III_6_IDR GPIO_IDR_IDR6
#define P_III_7_GPIO GPIOG
#define P_III_7_IDR GPIO_IDR_IDR9
#define P_III_8_GPIO GPIOG
#define P_III_8_IDR GPIO_IDR_IDR11

// GRUPA IV
#define P_IV_1_GPIO GPIOC
#define P_IV_1_IDR GPIO_IDR_IDR10
#define P_IV_2_GPIO GPIOC
#define P_IV_2_IDR GPIO_IDR_IDR12
#define P_IV_3_GPIO GPIOD
#define P_IV_3_IDR GPIO_IDR_IDR1
#define P_IV_4_GPIO GPIOD
#define P_IV_4_IDR GPIO_IDR_IDR3
#define P_IV_5_GPIO GPIOD
#define P_IV_5_IDR GPIO_IDR_IDR5
#define P_IV_6_GPIO GPIOD
#define P_IV_6_IDR GPIO_IDR_IDR7
#define P_IV_7_GPIO GPIOG
#define P_IV_7_IDR GPIO_IDR_IDR10
#define P_IV_8_GPIO GPIOG
#define P_IV_8_IDR GPIO_IDR_IDR12

// GRUPA V
#define P_V_1_GPIO GPIOB
#define P_V_1_IDR GPIO_IDR_IDR11
#define P_V_2_GPIO GPIOE
#define P_V_2_IDR GPIO_IDR_IDR15
#define P_V_3_GPIO GPIOE
#define P_V_3_IDR GPIO_IDR_IDR13
#define P_V_4_GPIO GPIOE
#define P_V_4_IDR GPIO_IDR_IDR11
#define P_V_5_GPIO GPIOE
#define P_V_5_IDR GPIO_IDR_IDR9
#define P_V_6_GPIO GPIOE
#define P_V_6_IDR GPIO_IDR_IDR7
#define P_V_7_GPIO GPIOG
#define P_V_7_IDR GPIO_IDR_IDR0
#define P_V_8_GPIO GPIOF
#define P_V_8_IDR GPIO_IDR_IDR14

// GRUPA VI
#define P_VI_1_GPIO GPIOB
#define P_VI_1_IDR GPIO_IDR_IDR10
#define P_VI_2_GPIO GPIOE
#define P_VI_2_IDR GPIO_IDR_IDR14
#define P_VI_3_GPIO GPIOE
#define P_VI_3_IDR GPIO_IDR_IDR12
#define P_VI_4_GPIO GPIOE
#define P_VI_4_IDR GPIO_IDR_IDR10
#define P_VI_5_GPIO GPIOE
#define P_VI_5_IDR GPIO_IDR_IDR8
#define P_VI_6_GPIO GPIOG
#define P_VI_6_IDR GPIO_IDR_IDR1
#define P_VI_7_GPIO GPIOF
#define P_VI_7_IDR GPIO_IDR_IDR15
#define P_VI_8_GPIO GPIOF
#define P_VI_8_IDR GPIO_IDR_IDR13

// GRUPA VII
#define P_VII_1_GPIO GPIOB
#define P_VII_1_IDR GPIO_IDR_IDR2
#define P_VII_2_GPIO GPIOB
#define P_VII_2_IDR GPIO_IDR_IDR0
#define P_VII_3_GPIO GPIOC
#define P_VII_3_IDR GPIO_IDR_IDR4
#define P_VII_4_GPIO GPIOA
#define P_VII_4_IDR GPIO_IDR_IDR6
#define P_VII_5_GPIO GPIOA
#define P_VII_5_IDR GPIO_IDR_IDR4
#define P_VII_6_GPIO GPIOA
#define P_VII_6_IDR GPIO_IDR_IDR2
#define P_VII_7_GPIO GPIOA
#define P_VII_7_IDR GPIO_IDR_IDR0
#define P_VII_8_GPIO GPIOC
#define P_VII_8_IDR GPIO_IDR_IDR2

// GRUPA VIII
#define P_VIII_1_GPIO GPIOB
#define P_VIII_1_IDR GPIO_IDR_IDR1
#define P_VIII_2_GPIO GPIOC
#define P_VIII_2_IDR GPIO_IDR_IDR5
#define P_VIII_3_GPIO GPIOA
#define P_VIII_3_IDR GPIO_IDR_IDR7
#define P_VIII_4_GPIO GPIOA
#define P_VIII_4_IDR GPIO_IDR_IDR5
#define P_VIII_5_GPIO GPIOA
#define P_VIII_5_IDR GPIO_IDR_IDR3
#define P_VIII_6_GPIO GPIOA
#define P_VIII_6_IDR GPIO_IDR_IDR1
#define P_VIII_7_GPIO GPIOC
#define P_VIII_7_IDR GPIO_ID_IDR3
#define P_VIII_8_GPIO GPIOC
#define P_VIII_8_IDR GPIO_IDR_IDR1

// GRUPA IX
#define P_IX_1_GPIO GPIOF
#define P_IX_1_IDR GPIO_IDR_IDR9
#define P_IX_2_GPIO GPIOF
#define P_IX_2_IDR GPIO_IDR_IDR7
#define P_IX_3_GPIO GPIOF
#define P_IX_3_IDR GPIO_IDR_IDR5
#define P_IX_4_GPIO GPIOF
#define P_IX_4_IDR GPIO_IDR_IDR3
#define P_IX_5_GPIO GPIOF
#define P_IX_5_IDR GPIO_IDR_IDR1
#define P_IX_6_GPIO GPIOC
#define P_IX_6_IDR GPIO_IDR_IDR13
#define P_IX_7_GPIO GPIOE
#define P_IX_7_IDR GPIO_IDR_IDR5
#define P_IX_8_GPIO GPIOE
#define P_IX_8_IDR GPIO_IDR_IDR3

// GRUPA X
#define P_X_1_GPIO GPIOF
#define P_X_1_IDR GPIO_IDR_IDR8
#define P_X_2_GPIO GPIOF
#define P_X_2_IDR GPIO_IDR_IDR6
#define P_X_3_GPIO GPIOF
#define P_X_3_IDR GPIO_IDR_IDR4
#define P_X_4_GPIO GPIOF
#define P_X_4_IDR GPIO_IDR_IDR2
#define P_X_5_GPIO GPIOF
#define P_X_5_IDR GPIO_IDR_IDR0
#define P_X_6_GPIO GPIOE
#define P_X_6_IDR GPIO_IDR_IDR6
#define P_X_7_GPIO GPIOE
#define P_X_7_IDR GPIO_IDR_IDR4
#define P_X_8_GPIO GPIOE
#define P_X_8_IDR GPIO_IDR_IDR2

// GRUPA XI
#define P_XI_1_GPIO GPIOA
#define P_XI_1_IDR GPIO_IDR_IDR8
#define P_XI_2_GPIO GPIOC
#define P_XI_2_IDR GPIO_IDR_IDR8
#define P_XI_3_GPIO GPIOC
#define P_XI_3_IDR GPIO_IDR_IDR6
#define P_XI_4_GPIO GPIOG
#define P_XI_4_IDR GPIO_IDR_IDR7
#define P_XI_5_GPIO GPIOG
#define P_XI_5_IDR GPIO_IDR_IDR6
#define P_XI_6_GPIO GPIOG
#define P_XI_6_IDR GPIO_IDR_IDR8
#define P_XI_7_GPIO GPIOC
#define P_XI_7_IDR GPIO_IDR_IDR7
#define P_XI_8_GPIO GPIOC
#define P_XI_8_IDR GPIO_IDR_IDR9

// GRUPA XII
#define P_XII_1_GPIO GPIOB
#define P_XII_1_IDR GPIO_IDR_IDR7
#define P_XII_2_GPIO GPIOB
#define P_XII_2_IDR GPIO_IDR_IDR5
#define P_XII_3_GPIO GPIOB
#define P_XII_3_IDR GPIO_IDR_IDR3
#define P_XII_4_GPIO GPIOG
#define P_XII_4_IDR GPIO_IDR_IDR14
#define P_XII_5_GPIO GPIOG
#define P_XII_5_IDR GPIO_IDR_IDR13
#define P_XII_6_GPIO GPIOG
#define P_XII_6_IDR GPIO_IDR_IDR15
#define P_XII_7_GPIO GPIOB
#define P_XII_7_IDR GPIO_IDR_IDR4
#define P_XII_8_GPIO GPIOB
#define P_XII_8_IDR GPIO_IDR_IDR6

// Connectors (Analog / GPIO)
#define P_C_I_GPIO GPIOA
#define P_C_I_IDR GPIO_IDR_IDR9
#define P_C_II_GPIO GPIOA
#define P_C_II_IDR GPIO_IDR_IDR10


//SWITCHES
unsigned long Delay = 100;
#define P_I_1_SWITCH

#ifdef P_I_1_SWITCH
uint32_t _P_I_1_LastDebounceTime = 0;
uint8_t _P_I_1_LastStateBtn = HIGH;

READ_PORT(P_I_1_GPIO, P_I_1_IDR)
#endif

// // PORT A
// #define PA0_GPIO  GPIOA
// #define PA0_IDR   GPIO_IDR_IDR0
// #define PA1_GPIO  GPIOA
// #define PA1_IDR   GPIO_IDR_IDR1
// #define PA2_GPIO  GPIOA
// #define PA2_IDR   GPIO_IDR_IDR2
// #define PA3_GPIO  GPIOA
// #define PA3_IDR   GPIO_IDR_IDR3
// #define PA4_GPIO  GPIOA
// #define PA4_IDR   GPIO_IDR_IDR4
// #define PA5_GPIO  GPIOA
// #define PA5_IDR   GPIO_IDR_IDR5
// #define PA6_GPIO  GPIOA
// #define PA6_IDR   GPIO_IDR_IDR6
// #define PA7_GPIO  GPIOA
// #define PA7_IDR   GPIO_IDR_IDR7
// #define PA8_GPIO  GPIOA
// #define PA8_IDR   GPIO_IDR_IDR8
// #define PA9_GPIO  GPIOA
// #define PA9_IDR   GPIO_IDR_IDR9
// #define PA10_GPIO GPIOA
// #define PA10_IDR  GPIO_IDR_IDR10
// #define PA11_GPIO GPIOA
// #define PA11_IDR  GPIO_IDR_IDR11
// #define PA12_GPIO GPIOA
// #define PA12_IDR  GPIO_IDR_IDR12
// #define PA13_GPIO GPIOA
// #define PA13_IDR  GPIO_IDR_IDR13
// #define PA14_GPIO GPIOA
// #define PA14_IDR  GPIO_IDR_IDR14
// #define PA15_GPIO GPIOA
// #define PA15_IDR  GPIO_IDR_IDR15

// // PORT B
// #define PB0_GPIO  GPIOB
// #define PB0_IDR   GPIO_IDR_IDR0
// #define PB1_GPIO  GPIOB
// #define PB1_IDR   GPIO_IDR_IDR1
// #define PB2_GPIO  GPIOB
// #define PB2_IDR   GPIO_IDR_IDR2
// #define PB3_GPIO  GPIOB
// #define PB3_IDR   GPIO_IDR_IDR3
// #define PB4_GPIO  GPIOB
// #define PB4_IDR   GPIO_IDR_IDR4
// #define PB5_GPIO  GPIOB
// #define PB5_IDR   GPIO_IDR_IDR5
// #define PB6_GPIO  GPIOB
// #define PB6_IDR   GPIO_IDR_IDR6
// #define PB7_GPIO  GPIOB
// #define PB7_IDR   GPIO_IDR_IDR7
// #define PB8_GPIO  GPIOB
// #define PB8_IDR   GPIO_IDR_IDR8
// #define PB9_GPIO  GPIOB
// #define PB9_IDR   GPIO_IDR_IDR9
// #define PB10_GPIO GPIOB
// #define PB10_IDR  GPIO_IDR_IDR10
// #define PB11_GPIO GPIOB
// #define PB11_IDR  GPIO_IDR_IDR11
// #define PB12_GPIO GPIOB
// #define PB12_IDR  GPIO_IDR_IDR12
// #define PB13_GPIO GPIOB
// #define PB13_IDR  GPIO_IDR_IDR13
// #define PB14_GPIO GPIOB
// #define PB14_IDR  GPIO_IDR_IDR14
// #define PB15_GPIO GPIOB
// #define PB15_IDR  GPIO_IDR_IDR15

// // PORT C
// #define PC0_GPIO  GPIOC
// #define PC0_IDR   GPIO_IDR_IDR0
// #define PC1_GPIO  GPIOC
// #define PC1_IDR   GPIO_IDR_IDR1
// #define PC2_GPIO  GPIOC
// #define PC2_IDR   GPIO_IDR_IDR2
// #define PC3_GPIO  GPIOC
// #define PC3_IDR   GPIO_IDR_IDR3
// #define PC4_GPIO  GPIOC
// #define PC4_IDR   GPIO_IDR_IDR4
// #define PC5_GPIO  GPIOC
// #define PC5_IDR   GPIO_IDR_IDR5
// #define PC6_GPIO  GPIOC
// #define PC6_IDR   GPIO_IDR_IDR6
// #define PC7_GPIO  GPIOC
// #define PC7_IDR   GPIO_IDR_IDR7
// #define PC8_GPIO  GPIOC
// #define PC8_IDR   GPIO_IDR_IDR8
// #define PC9_GPIO  GPIOC
// #define PC9_IDR   GPIO_IDR_IDR9
// #define PC10_GPIO GPIOC
// #define PC10_IDR  GPIO_IDR_IDR10
// #define PC11_GPIO GPIOC
// #define PC11_IDR  GPIO_IDR_IDR11
// #define PC12_GPIO GPIOC
// #define PC12_IDR  GPIO_IDR_IDR12
// #define PC13_GPIO GPIOC
// #define PC13_IDR  GPIO_IDR_IDR13
// #define PC14_GPIO GPIOC
// #define PC14_IDR  GPIO_IDR_IDR14
// #define PC15_GPIO GPIOC
// #define PC15_IDR  GPIO_IDR_IDR15

// // PORT D
// #define PD0_GPIO  GPIOD
// #define PD0_IDR   GPIO_IDR_IDR0
// #define PD1_GPIO  GPIOD
// #define PD1_IDR   GPIO_IDR_IDR1
// #define PD2_GPIO  GPIOD
// #define PD2_IDR   GPIO_IDR_IDR2
// #define PD3_GPIO  GPIOD
// #define PD3_IDR   GPIO_IDR_IDR3
// #define PD4_GPIO  GPIOD
// #define PD4_IDR   GPIO_IDR_IDR4
// #define PD5_GPIO  GPIOD
// #define PD5_IDR   GPIO_IDR_IDR5
// #define PD6_GPIO  GPIOD
// #define PD6_IDR   GPIO_IDR_IDR6
// #define PD7_GPIO  GPIOD
// #define PD7_IDR   GPIO_IDR_IDR7
// #define PD8_GPIO  GPIOD
// #define PD8_IDR   GPIO_IDR_IDR8
// #define PD9_GPIO  GPIOD
// #define PD9_IDR   GPIO_IDR_IDR9
// #define PD10_GPIO GPIOD
// #define PD10_IDR  GPIO_IDR_IDR10
// #define PD11_GPIO GPIOD
// #define PD11_IDR  GPIO_IDR_IDR11
// #define PD12_GPIO GPIOD
// #define PD12_IDR  GPIO_IDR_IDR12
// #define PD13_GPIO GPIOD
// #define PD13_IDR  GPIO_IDR_IDR13
// #define PD14_GPIO GPIOD
// #define PD14_IDR  GPIO_IDR_IDR14
// #define PD15_GPIO GPIOD
// #define PD15_IDR  GPIO_IDR_IDR15

// // PORT E
// #define PE0_GPIO  GPIOE
// #define PE0_IDR   GPIO_IDR_IDR0
// #define PE1_GPIO  GPIOE
// #define PE1_IDR   GPIO_IDR_IDR1
// #define PE2_GPIO  GPIOE
// #define PE2_IDR   GPIO_IDR_IDR2
// #define PE3_GPIO  GPIOE
// #define PE3_IDR   GPIO_IDR_IDR3
// #define PE4_GPIO  GPIOE
// #define PE4_IDR   GPIO_IDR_IDR4
// #define PE5_GPIO  GPIOE
// #define PE5_IDR   GPIO_IDR_IDR5
// #define PE6_GPIO  GPIOE
// #define PE6_IDR   GPIO_IDR_IDR6
// #define PE7_GPIO  GPIOE
// #define PE7_IDR   GPIO_IDR_IDR7
// #define PE8_GPIO  GPIOE
// #define PE8_IDR   GPIO_IDR_IDR8
// #define PE9_GPIO  GPIOE
// #define PE9_IDR   GPIO_IDR_IDR9
// #define PE10_GPIO GPIOE
// #define PE10_IDR  GPIO_IDR_IDR10
// #define PE11_GPIO GPIOE
// #define PE11_IDR  GPIO_IDR_IDR11
// #define PE12_GPIO GPIOE
// #define PE12_IDR  GPIO_IDR_IDR12
// #define PE13_GPIO GPIOE
// #define PE13_IDR  GPIO_IDR_IDR13
// #define PE14_GPIO GPIOE
// #define PE14_IDR  GPIO_IDR_IDR14
// #define PE15_GPIO GPIOE
// #define PE15_IDR  GPIO_IDR_IDR15

// // PORT F
// #define PF0_GPIO  GPIOF
// #define PF0_IDR   GPIO_IDR_IDR0
// #define PF1_GPIO  GPIOF
// #define PF1_IDR   GPIO_IDR_IDR1
// #define PF2_GPIO  GPIOF
// #define PF2_IDR   GPIO_IDR_IDR2
// #define PF3_GPIO  GPIOF
// #define PF3_IDR   GPIO_IDR_IDR3
// #define PF4_GPIO  GPIOF
// #define PF4_IDR   GPIO_IDR_IDR4
// #define PF5_GPIO  GPIOF
// #define PF5_IDR   GPIO_IDR_IDR5
// #define PF6_GPIO  GPIOF
// #define PF6_IDR   GPIO_IDR_IDR6
// #define PF7_GPIO  GPIOF
// #define PF7_IDR   GPIO_IDR_IDR7
// #define PF8_GPIO  GPIOF
// #define PF8_IDR   GPIO_IDR_IDR8
// #define PF9_GPIO  GPIOF
// #define PF9_IDR   GPIO_IDR_IDR9
// #define PF10_GPIO GPIOF
// #define PF10_IDR  GPIO_IDR_IDR10
// #define PF11_GPIO GPIOF
// #define PF11_IDR  GPIO_IDR_IDR11
// #define PF12_GPIO GPIOF
// #define PF12_IDR  GPIO_IDR_IDR12
// #define PF13_GPIO GPIOF
// #define PF13_IDR  GPIO_IDR_IDR13
// #define PF14_GPIO GPIOF
// #define PF14_IDR  GPIO_IDR_IDR14
// #define PF15_GPIO GPIOF
// #define PF15_IDR  GPIO_IDR_IDR15

// // PORT G
// #define PG0_GPIO  GPIOG
// #define PG0_IDR   GPIO_IDR_IDR0
// #define PG1_GPIO  GPIOG
// #define PG1_IDR   GPIO_IDR_IDR1
// #define PG2_GPIO  GPIOG
// #define PG2_IDR   GPIO_IDR_IDR2
// #define PG3_GPIO  GPIOG
// #define PG3_IDR   GPIO_IDR_IDR3
// #define PG4_GPIO  GPIOG
// #define PG4_IDR   GPIO_IDR_IDR4
// #define PG5_GPIO  GPIOG
// #define PG5_IDR   GPIO_IDR_IDR5
// #define PG6_GPIO  GPIOG
// #define PG6_IDR   GPIO_IDR_IDR6
// #define PG7_GPIO  GPIOG
// #define PG7_IDR   GPIO_IDR_IDR7
// #define PG8_GPIO  GPIOG
// #define PG8_IDR   GPIO_IDR_IDR8
// #define PG9_GPIO  GPIOG
// #define PG9_IDR   GPIO_IDR_IDR9
// #define PG10_GPIO GPIOG
// #define PG10_IDR  GPIO_IDR_IDR10
// #define PG11_GPIO GPIOG
// #define PG11_IDR  GPIO_IDR_IDR11
// #define PG12_GPIO GPIOG
// #define PG12_IDR  GPIO_IDR_IDR12
// #define PG13_GPIO GPIOG
// #define PG13_IDR  GPIO_IDR_IDR13
// #define PG14_GPIO GPIOG
// #define PG14_IDR  GPIO_IDR_IDR14
// #define PG15_GPIO GPIOG
// #define PG15_IDR  GPIO_IDR_IDR15

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

#include <MySensors.h>
#include <24C32.h>
#include <Wire.h>

EE EEPROM24C32;


void inicjalizeI2C() {
  Wire.setSDA(SDA_24C32_PIN);
  Wire.setSCL(SCL_24C32_PIN);
  Wire.begin();
  Wire.setClock(400000);
  EEPROM24C32.begin(0x50, false);
}

/* #region  Power Optimization */

/**
  * @brief  System Clock Configuration Reduced with good main performance
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000 --> 48000000
  *            HCLK(Hz)                       = 72000000 --> 48000000
  *            AHB Prescaler                  = 1        
  *            APB1 Prescaler                 = 2        
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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

#ifdef P_I_1_SWITCH
  present(P_I_1, S_DOOR, "P_I_1");
#endif



  isPresentedToController = true;
}


void before() {
  /* M4_MS_SwitchSensorManager */

#ifdef P_I_1_SWITCH
pinMode(P_I_1, INPUT_PULLDOWN);
#endif

  // // I
  // mySwitchManager.addSwitch(P_I_1, NORMAL_CLOSE, "P_I_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_I_2, NORMAL_CLOSE, "P_I_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_I_3, NORMAL_CLOSE, "P_I_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_I_4, NORMAL_CLOSE, "P_I_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_I_5, NORMAL_CLOSE, "P_I_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_I_6, NORMAL_CLOSE, "P_I_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_I_7, NORMAL_CLOSE, "P_I_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_I_8, NORMAL_CLOSE, "P_I_8");  // M4_MS_SwitchSensorManager

  // // II
  // mySwitchManager.addSwitch(P_II_1, NORMAL_CLOSE, "P_II_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_II_2, NORMAL_CLOSE, "P_II_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_II_3, NORMAL_CLOSE, "P_II_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_II_4, NORMAL_CLOSE, "P_II_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_II_5, NORMAL_CLOSE, "P_II_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_II_6, NORMAL_CLOSE, "P_II_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_II_7, NORMAL_CLOSE, "P_II_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_II_8, NORMAL_CLOSE, "P_II_8");  // M4_MS_SwitchSensorManager

  // // III
  // mySwitchManager.addSwitch(P_III_1, NORMAL_CLOSE, "P_III_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_III_2, NORMAL_CLOSE, "P_III_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_III_3, NORMAL_CLOSE, "P_III_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_III_4, NORMAL_CLOSE, "P_III_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_III_5, NORMAL_CLOSE, "P_III_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_III_6, NORMAL_CLOSE, "P_III_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_III_7, NORMAL_CLOSE, "P_III_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_III_8, NORMAL_CLOSE, "P_III_8");  // M4_MS_SwitchSensorManager

  // // IV
  // mySwitchManager.addSwitch(P_IV_1, NORMAL_CLOSE, "P_IV_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IV_2, NORMAL_CLOSE, "P_IV_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IV_3, NORMAL_CLOSE, "P_IV_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IV_4, NORMAL_CLOSE, "P_IV_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IV_5, NORMAL_CLOSE, "P_IV_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IV_6, NORMAL_CLOSE, "P_IV_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IV_7, NORMAL_CLOSE, "P_IV_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IV_8, NORMAL_CLOSE, "P_IV_8");  // M4_MS_SwitchSensorManager

  // // V
  // mySwitchManager.addSwitch(P_V_1, NORMAL_CLOSE, "P_V_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_V_2, NORMAL_CLOSE, "P_V_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_V_3, NORMAL_CLOSE, "P_V_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_V_4, NORMAL_CLOSE, "P_V_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_V_5, NORMAL_CLOSE, "P_V_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_V_6, NORMAL_CLOSE, "P_V_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_V_7, NORMAL_CLOSE, "P_V_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_V_8, NORMAL_CLOSE, "P_V_8");  // M4_MS_SwitchSensorManager

  // // VI
  // mySwitchManager.addSwitch(P_VI_1, NORMAL_CLOSE, "P_VI_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VI_2, NORMAL_CLOSE, "P_VI_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VI_3, NORMAL_CLOSE, "P_VI_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VI_4, NORMAL_CLOSE, "P_VI_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VI_5, NORMAL_CLOSE, "P_VI_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VI_6, NORMAL_CLOSE, "P_VI_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VI_7, NORMAL_CLOSE, "P_VI_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VI_8, NORMAL_CLOSE, "P_VI_8");  // M4_MS_SwitchSensorManager

  // // VII
  // mySwitchManager.addSwitch(P_VII_1, NORMAL_CLOSE, "P_VII_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VII_2, NORMAL_CLOSE, "P_VII_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VII_3, NORMAL_CLOSE, "P_VII_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VII_4, NORMAL_CLOSE, "P_VII_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VII_5, NORMAL_CLOSE, "P_VII_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VII_6, NORMAL_CLOSE, "P_VII_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VII_7, NORMAL_CLOSE, "P_VII_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VII_8, NORMAL_CLOSE, "P_VII_8");  // M4_MS_SwitchSensorManager

  // // VIII
  // mySwitchManager.addSwitch(P_VIII_1, NORMAL_CLOSE, "P_VIII_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VIII_2, NORMAL_CLOSE, "P_VIII_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VIII_3, NORMAL_CLOSE, "P_VIII_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VIII_4, NORMAL_CLOSE, "P_VIII_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VIII_5, NORMAL_CLOSE, "P_VIII_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VIII_6, NORMAL_CLOSE, "P_VIII_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VIII_7, NORMAL_CLOSE, "P_VIII_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_VIII_8, NORMAL_CLOSE, "P_VIII_8");  // M4_MS_SwitchSensorManager

  // // IX
  // mySwitchManager.addSwitch(P_IX_1, NORMAL_CLOSE, "P_IX_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IX_2, NORMAL_CLOSE, "P_IX_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IX_3, NORMAL_CLOSE, "P_IX_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IX_4, NORMAL_CLOSE, "P_IX_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IX_5, NORMAL_CLOSE, "P_IX_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IX_6, NORMAL_CLOSE, "P_IX_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IX_7, NORMAL_CLOSE, "P_IX_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_IX_8, NORMAL_CLOSE, "P_IX_8");  // M4_MS_SwitchSensorManager

  // // X
  // mySwitchManager.addSwitch(P_X_1, NORMAL_CLOSE, "P_X_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_X_2, NORMAL_CLOSE, "P_X_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_X_3, NORMAL_CLOSE, "P_X_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_X_4, NORMAL_CLOSE, "P_X_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_X_5, NORMAL_CLOSE, "P_X_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_X_6, NORMAL_CLOSE, "P_X_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_X_7, NORMAL_CLOSE, "P_X_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_X_8, NORMAL_CLOSE, "P_X_8");  // M4_MS_SwitchSensorManager

  // // XI
  // mySwitchManager.addSwitch(P_XI_1, NORMAL_CLOSE, "P_XI_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XI_2, NORMAL_CLOSE, "P_XI_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XI_3, NORMAL_CLOSE, "P_XI_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XI_4, NORMAL_CLOSE, "P_XI_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XI_5, NORMAL_CLOSE, "P_XI_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XI_6, NORMAL_CLOSE, "P_XI_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XI_7, NORMAL_CLOSE, "P_XI_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XI_8, NORMAL_CLOSE, "P_XI_8");  // M4_MS_SwitchSensorManager

  // // XIIc:\Programowanie\7.Projekty\5.Arduino\M1_MS_RelayManager\M1_MS_RelayManager.ino
  // mySwitchManager.addSwitch(P_XII_1, NORMAL_CLOSE, "P_XII_1");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XII_2, NORMAL_CLOSE, "P_XII_2");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XII_3, NORMAL_CLOSE, "P_XII_3");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XII_4, NORMAL_CLOSE, "P_XII_4");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XII_5, NORMAL_CLOSE, "P_XII_5");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XII_6, NORMAL_CLOSE, "P_XII_6");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XII_7, NORMAL_CLOSE, "P_XII_7");  // M4_MS_SwitchSensorManager
  // mySwitchManager.addSwitch(P_XII_8, NORMAL_CLOSE, "P_XII_8");  // M4_MS_SwitchSensorManager
  
}

void setup() {
  // put your setup code here, to run once:
  inicjalizeI2C();
}

#define READ_PORT(port, pin) ((port->IDR & pin) != 0)

void loop() {
  if (SCM.isStateChanged(isPresentedToController, 0)) {
    wait(50);

    #ifdef P_I_1_SWITCH
pinMode(P_I_1, INPUT_PULLDOWN);
#endif
  }

  // put your main code here, to run repeatedly:
  mySwitchManager.switchCheckState();  //M4_MS_SwitchSensorManager
}
