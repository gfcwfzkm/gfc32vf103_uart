/* UART FIFO LIBRARY
 *
 * Basic FIFO UART library ported to the RISC-V GD32V microcontroller family
 * uart.h
 * Version 1
 * Created: 12.10.2021
 *  Author: gfcwfzkm
 */ 

#include "gfc32vf103_uart.h"

/**
 * Diverse defines um das "Porten" zu anderen Mikrokontrollern zu erleichtern
 */
#define _UART_FLAG_OVERRUN(ptr)	usart_flag_get(ptr, USART_FLAG_ORERR)
#define _UART_FLAG_FRAME(ptr)	usart_flag_get(ptr, USART_FLAG_FERR)
#define _UART_FLAG_PARITY(ptr)	usart_flag_get(ptr, USART_FLAG_PERR)
#define _UART_DATA_RX(ptr)		usart_data_receive(ptr)
#define _UART_DATA_TX(ptr,ch)	usart_data_transmit(ptr, ch);
#define _UART_IRQ_TX_EN(ptr)	usart_interrupt_enable(ptr, USART_INT_TBE)
#define _UART_IRQ_TX_DIS(ptr)	usart_interrupt_disable(ptr, USART_INT_TBE)


/**
 * @brief Interrupt-Empfangsfunktion (Interrupt Service Routine)
 */
static void uart_receive_interrupt(BUF_UART_t *uartPtr)
{
	uint8_t tmphead,data,lastRxError = 0;
		
	/* Sicherstellen das der Pointer �berhaupt wohin zeigt! */
	if (uartPtr == 0)	return;
	
	/* Statusregister abarbeiten */
	if (_UART_FLAG_OVERRUN(uartPtr->hw_usart))		lastRxError |= UART_OVERRUN_ERROR;
	if (_UART_FLAG_FRAME(uartPtr->hw_usart))		lastRxError |= UART_FRAME_ERROR;
	if (_UART_FLAG_PARITY(uartPtr->hw_usart))		lastRxError |= UART_PARITY_ERROR;
	
	/* UART Datenregister einlesen */
	data = _UART_DATA_RX(uartPtr->hw_usart);
	
	/* Neuer Bufferindex berechnen und �berpr�fen */
	tmphead = (uartPtr->rxBufHead + 1) & (uartPtr->rxBufLen - 1);
	
	if (tmphead == uartPtr->rxBufTail)
	{
		/* Fehler! Der Empfangsbuffer ist voll! */
		lastRxError |= UART_BUFFER_OVERFLOW;
	}
	else
	{
		lastRxError |= UART_DATA_AVAILABLE;
		/* Neuer Bufferindex speichern */
		uartPtr->rxBufHead = tmphead;
		/* Empfangenes Byte im Buffer speichern */
		uartPtr->rxBuffer[tmphead] = data;
		/* Empfangenes Byte ggf. �berpr�fen */
		if ( (uartPtr->lookForChar != 0) && (uartPtr->lookForChar == data) )
		{
			uartPtr->specialCharFound = 1;
		}
	}
	uartPtr->lastError |= lastRxError;
	/* Empfangs / Rx Interrupt-Flag l�scht sich beim Lesevorgang von selber */
}

/**
 * @brief Interrupt-Sendefunktion (Interrupt Service Routine)
 */
static void uart_transmit_interrupt(BUF_UART_t *uartPtr)
{
	uint8_t tmptail;
		
	/* Sicherstellen das der Pointer �berhaupt wohin zeigt! */
	if (uartPtr == 0)	return;
	
	if (uartPtr->txBufHead != uartPtr->txBufTail)
	{
		/* Neuer Bufferindex berechnen und speichern */
		tmptail = (uartPtr->txBufTail + 1) & (uartPtr->txBufLen - 1);
		uartPtr->txBufTail = tmptail;
		/* Byte aus dem Buffer ins UART Senderegister legen */
		_UART_DATA_TX(uartPtr->hw_usart, uartPtr->txBuffer[tmptail]);
		
	}
	else
	{
		/* Tx Buffer leer, Sendeinterrupt deaktivieren */
		_UART_IRQ_TX_DIS(uartPtr->hw_usart);
	}
}

void uart_interrupt(BUF_UART_t *uartPtr)
{
	if (usart_interrupt_flag_get(uartPtr->hw_usart, USART_INT_FLAG_RBNE))
    {
        uart_receive_interrupt(uartPtr);
    }
    if (usart_interrupt_flag_get(uartPtr->hw_usart, USART_INT_FLAG_TBE))
    {
        uart_transmit_interrupt(uartPtr);
    }
}

void uart_init(BUF_UART_t *uartPtr, uint32_t _usart, uint32_t uartBaudRate, enum UART_SETTING _setting, uint8_t *recivBuf,  const uint8_t recivBufLen, uint8_t *transBuf, const uint8_t transBufLen)
{
	/* UARTInstanz Pointer sauber zur�cksetzten / definieren */
	uartPtr->rxBuffer = recivBuf;
	uartPtr->rxBufLen = recivBufLen;
	uartPtr->rxBufHead = 0;
	uartPtr->rxBufTail = 0;
	uartPtr->txBuffer = transBuf;
	uartPtr->txBufLen = transBufLen;
	uartPtr->txBufTail = 0;
	uartPtr->txBufHead = 0;
	uartPtr->lastError = 0;
	uartPtr->lookForChar = 0;
	uartPtr->specialCharFound = 0;
	uartPtr->hw_usart = _usart;
	
	/* Initialisiere die UART Peripherie */
	usart_deinit(_usart);
	usart_baudrate_set(_usart, uartBaudRate);
	usart_word_length_set(_usart, USART_WL_8BIT);
	if (_setting & 0x01){
		usart_stop_bit_set(_usart, USART_STB_2BIT);
	}else{
		usart_stop_bit_set(_usart, USART_STB_1BIT);
	}
	if (_setting & 0x02){
		usart_parity_config(_usart, USART_PM_EVEN);
	}else if (_setting & 0x04){
		usart_parity_config(_usart, USART_PM_ODD);
	}else{
		usart_parity_config(_usart, USART_PM_NONE);
	}
	usart_hardware_flow_cts_config(_usart, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(_usart, USART_CTS_DISABLE);
    usart_receive_config(_usart, USART_RECEIVE_ENABLE);
    usart_transmit_config(_usart, USART_TRANSMIT_ENABLE);
	usart_interrupt_enable(_usart, USART_INT_RBNE);
	usart_enable(_usart);
}

void uart_putc(BUF_UART_t *uartPtr, const uint8_t data)
{
	uint8_t tmphead, tmpBufTail;
	
	
	tmphead = (uartPtr->txBufHead + 1) & (uartPtr->txBufLen - 1);
		
	do 
	{
		tmpBufTail = uartPtr->txBufTail;		
		asm("nop");	/* Warten bis Platz im Buffer ist */		
	}while(tmphead == tmpBufTail);
	
	/* Zu sendendes Byte in den Buffer legen */
	uartPtr->txBuffer[tmphead] = data;
	uartPtr->txBufHead = tmphead;
	
	/* 'Data Register Empty' Interrupt aktivieren */
	_UART_IRQ_TX_EN(uartPtr->hw_usart);

}

void uart_send(BUF_UART_t *uartPtr, const uint8_t *dataBuffer, const uint8_t bytesToSend)
{
	/* Anzahl Bytes vom Buffer senden */
	for (uint8_t i = 0; i < bytesToSend; i++)
	{
		uart_putc(uartPtr, dataBuffer[i]);
	}
}

void uart_print(BUF_UART_t *uartPtr, const char *data)
{
	/* String bis Null-Terminator senden */
	while(*data)
	{
		uart_putc(uartPtr, *data++);
	}
}

UART_ERROR uart_rxStatus(BUF_UART_t *uartPtr)
{
	uint8_t lastRxError = uartPtr->lastError;
	
	if (uartPtr->rxBufHead == uartPtr->rxBufTail)
	{
		lastRxError |= UART_NO_DATA;
	}
	
	/* Aktuellster Status zur�ckgeben, ohne ihn zu l�schen */
	return (lastRxError);
}

uint16_t uart_peek(BUF_UART_t *uartPtr)
{
	uint8_t data, tmptail, lastRxError;
	
	if (uartPtr->rxBufHead == uartPtr->rxBufTail)
	{
		return (UART_NO_DATA << 8);	// Keine Daten!
	}
	
	/* Bufferindex berechnen */
	tmptail = (uartPtr->rxBufTail + 1) & (uartPtr->rxBufLen);
	
	/* FIFO Buffer -> �ltestes empfangene Byte aus dem Buffer lesen */
	data = uartPtr->rxBuffer[tmptail];
	lastRxError = uartPtr->lastError;
	
	return ((lastRxError << 8) | data);
}

uint16_t uart_getc(BUF_UART_t *uartPtr)
{
	uint8_t data, tmptail, lastRxError;
	
	if (uartPtr->rxBufHead == uartPtr->rxBufTail)
	{
		return (UART_NO_DATA << 8);	// Keine Daten!
	}
	
	/* Bufferindex berechnen */
	tmptail = (uartPtr->rxBufTail + 1) & (uartPtr->rxBufLen - 1);
	
	/* FIFO Buffer -> �ltestes empfangene Byte aus dem Buffer lesen */
	data = uartPtr->rxBuffer[tmptail];
	lastRxError = uartPtr->lastError;
	
	/* Bufferindex speichern */
	uartPtr->rxBufTail = tmptail;
	uartPtr->specialCharFound = 0;
	
	uartPtr->lastError = UART_DATA_AVAILABLE;
	return ((lastRxError << 8) | data);
}

UART_ERROR uart_isTxFull(BUF_UART_t *uartPtr)
{
	uint8_t tmphead = (uartPtr->txBufHead + 1) & (uartPtr->txBufLen - 1);
	
	if (tmphead == uartPtr->txBufTail)	return UART_TX_BUFFER_FULL;
	return UART_DATA_AVAILABLE;
}

uint8_t uart_charDetected(BUF_UART_t *uartPtr)
{
	uint8_t tmpHead,tmpTail,tmpLen,txtlen = 0;
	
	if (uartPtr->specialCharFound)
	{
		tmpHead = uartPtr->rxBufHead;
		tmpTail = uartPtr->rxBufTail;
		tmpLen = uartPtr->rxBufLen - 1;
		
		do
		{
			tmpTail = (tmpTail + 1) & tmpLen;
			
			txtlen++;
			
			if (tmpTail == tmpHead)
			{
				txtlen = 0;
				break;
			}
		} while (uartPtr->rxBuffer[tmpTail] != uartPtr->lookForChar);
		
		uartPtr->specialCharFound = 0;
	}
	
	return txtlen;
}

void uart_searchForCharacter(BUF_UART_t *uartPtr, char charToSearch)
{
	uartPtr->specialCharFound = 0;
	uartPtr->lookForChar = charToSearch;
}