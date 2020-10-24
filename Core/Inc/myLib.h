/**
 * @file myLib.h
 * @author Nam Nguyen (ndnam198@gmail.com)
 * @brief This lib is used only for extra peripheral definitions, such as button, led or buzzer, etc...
 *        Any others could be Print function which's for debug purposes
 * @version 0.1
 * @date 2020-09-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef __MYLIB_H
#define __MYLIB_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "main.h"

/* GENERAL-DEFINE-BEGIN */
char ucGeneralString[100];
#define configLL_UART
#define BKFET_BOARD
#define isString(src, des) ((strcmp((char *)src, des)) == 0 ? 1 : 0)
#define PRINTF(str) print(str)
#define print(str)                                          \
    do                                                      \
    {                                                       \
        sprintf(ucGeneralString, "%s", (char *)str);        \
        vUARTSend(DEBUG_USART, (uint8_t *)ucGeneralString); \
    } while (0)

#define printVar(var)                                             \
    do                                                            \
    {                                                             \
        sprintf(ucGeneralString, "Value of " #var " = %lu", var); \
        vUARTSend(DEBUG_USART, (uint8_t *)ucGeneralString);       \
        print("\r\n");                                            \
    } while (0)

#define newline vUARTSend(DEBUG_USART, (uint8_t *)"\r\n");

/* GENERAL-DEFINE-END */

/**
 * @brief Choose whether to use LL or HAL library for UART -----(OPTIONAL)
 * 
 */
//#define configHAL_UART

#if defined(configHAL_UART) /* configHAL_UART */
#define DEBUG_USART huart2
void vUARTSend(UART_HandleTypeDef huart, uint8_t *String);

#elif defined(configLL_UART) /* configLL_UART */
#define DEBUG_USART USART2
void vUARTSend(USART_TypeDef *USARTx, uint8_t *String);
#endif

// int fputc(int ch, FILE *f)
// {
// #if defined(configLL_UART) /* configHAL_UART */
//     LL_USART_TransmitData8(DEBUG_USART, (uint8_t)ch);
// #elif defined(configHAL_UART)
//     HAL_UART_Transmit(&DEBUG_USART, (uint8_t *)&ch, 1, 0xFFFF);
// #endif
//     return ch;
// }

#define toggleLed1 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
#define toggleLed2 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
#define toggleLed3 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
#define toggleLed4 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
#define toggleAllLed \
    do               \
    {                \
        toggleLed1;  \
        toggleLed2;  \
        toggleLed3;  \
        toggleLed4;  \
    } while (0)
#define toggleBuzzer HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
#define onLed1 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
#define onLed2 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
#define onLed3 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
#define onLed4 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
#define onAllLed \
    do           \
    {            \
        onLed1;  \
        onLed2;  \
        onLed3;  \
        onLed4;  \
    } while (0)
#define onBuzzer HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
#define offLed1 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
#define offLed2 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
#define offLed3 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
#define offLed4 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
#define offAllLed \
    do            \
    {             \
        offLed1;  \
        offLed2;  \
        offLed3;  \
        offLed4;  \
    } while (0)
#define offBuzzer HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
#endif /* __MYLIB_H */
