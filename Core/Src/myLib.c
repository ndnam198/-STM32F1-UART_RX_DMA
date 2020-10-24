#include "myLib.h"

#if defined(configHAL_UART)
void vUARTSend(UART_HandleTypeDef huart, uint8_t *String)
{
	HAL_UART_Transmit(&huart, (uint8_t *)String, strlen((char *)String), defineUART_TX_DELAY);
}
#endif /* configHAL_UART */

#if defined(configLL_UART)
void vUARTSend(USART_TypeDef *USARTx, uint8_t *String)
{
	uint32_t ulStringLen = 0;
	uint32_t i = 0;
	uint32_t ulBlockTime = 10000;
	ulStringLen = strlen((char *)String);

	while (ulStringLen)
	{
		LL_USART_TransmitData8(USARTx, (uint8_t)String[i]);
		i++;
		ulStringLen--;
		/* Check if transfer 1 byte completed */
		while (!LL_USART_IsActiveFlag_TC(USARTx))
		{
			if ((ulBlockTime--) == 0)
				break;
		}
	}
}
#endif /* configLL_UART */
