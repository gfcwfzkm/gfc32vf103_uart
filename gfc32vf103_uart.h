/* UART FIFO LIBRARY
 *
 * Basic FIFO UART library ported to the RISC-V GD32V microcontroller family
 * uart.h
 * Version 1
 * Created: 12.10.2021
 *  Author: gfcwfzkm
 */ 


#ifndef GFC32VF103_UART_H_
#define GFC32VF103_UART_H_

#include <stdio.h>
#include <inttypes.h>
#include "gd32vf103.h"

/**
 * @brief Settings for the serial port
 *
 * The first number stands for the bit width (currently only default 8-bit supported),
 * the letter for either no parity bit (N), even parity bit (E) or odd parity bit (O),
 * and the final number defines the amount of stop bits used
 */

enum UART_SETTING {
	SERIAL_8N1 = 0,
	SERIAL_8N2 = 1,
	SERIAL_8E1 = 2,
	SERIAL_8E2 = 3,
	SERIAL_8O1 = 4,
	SERIAL_8O2 = 5
};

/**
 * @brief Error return values / status of the uart receiver
 */
typedef enum {
	UART_DATA_AVAILABLE		= 0x00,	/**< Data available, rx buffer not empty */
	UART_NO_DATA			= 0x01,	/**< No data to read */
	UART_BUFFER_OVERFLOW	= 0x02,	/**< Buffer overflow warning */
	UART_PARITY_ERROR		= 0x04,	/**< Parity bit error */
	UART_OVERRUN_ERROR		= 0x08,	/**< USART overrun error */
	UART_FRAME_ERROR		= 0x10,	/**< Frame error */
	UART_TX_BUFFER_FULL		= 0x20	/**< tx buffer full */
}UART_ERROR;

/**
 * @brief Typedefinition for the struct containing the serial instance
 *
 * Contains the nessesary pointer/value to the uart peripheral as well
 * as the buffers and pointers.
 */
typedef struct {
	uint8_t *rxBuffer;	/**< uint8_t pointer to the receive buffer, max 255 bytes */
	uint8_t *txBuffer;	/**< uint8_t pointer to the transmit buffer, max 255 bytes */
	uint8_t rxBufLen;	/**< uint8_t size of the receive buffer */
	uint8_t txBufLen;	/**< uint8_t size of the transmit buffer */
	
	volatile uint8_t rxBufHead;	/**< uint8_t ringbuffer-counter */
	volatile uint8_t rxBufTail;	/**< uint8_t ringbuffer-counter */
	volatile uint8_t txBufHead;	/**< uint8_t ringbuffer-counter */
	volatile uint8_t txBufTail;	/**< uint8_t ringbuffer-counter */
	
	volatile UART_ERROR lastError:6;	/**< UART status / errors are stored here */
	uint8_t lookForChar; /**< special character to look for */
	
	uint8_t specialCharFound:1; /**< special character found -bit */
	
	uint32_t hw_usart;	/**< USART_t pointer / address to the hardware peripheral */
}BUF_UART_t;

/**
 * @brief UART receive interrupt event
 * 
 * Call this function from your ISR Interrupt function when a character has been received.
 * Example: \n \code{.c}
 * void USART1_IRQHandler(void)
 * {
 *     // Check if the receive interrupt flag of USART1 has been triggered
 *     if (usart_interrupt_flag_get(usart1Hnd.hw_usart, USART_INT_FLAG_RBNE))
 *     {
 *         uart_receive_interrupt(&usart1Hnd);
 *     }
 * 	   // Check if the transmit interrupt flag of USART1 has been triggered
 *     if (usart_interrupt_flag_get(usart1Hnd.hw_usart, USART_INT_FLAG_TBE))
 *     {
 *         uart_transmit_interrupt(&usart1Hnd);
 *     }
 * }
 * \endcode
 * @param uartPtr	Pointer to the initialised \a BUF_UART_t instance
 */
void uart_receive_interrupt(BUF_UART_t *uartPtr);

/**
 * @brief UART receive interrupt event
 * 
 * Call this function from your ISR Interrupt function when a character has been received.
 * Example: \n \code{.c}
 * void USART1_IRQHandler(void)
 * {
 *     // Check if the receive interrupt flag of USART1 has been triggered
 *     if (usart_interrupt_flag_get(usart1Hnd.hw_usart, USART_INT_FLAG_RBNE))
 *     {
 *         uart_receive_interrupt(&usart1Hnd);
 *     }
 * 	   // Check if the transmit interrupt flag of USART1 has been triggered
 *     if (usart_interrupt_flag_get(usart1Hnd.hw_usart, USART_INT_FLAG_TBE))
 *     {
 *         uart_transmit_interrupt(&usart1Hnd);
 *     }
 * }
 * \endcode
 * @param uartPtr	Pointer to the initialised \a BUF_UART_t instance
 */
void uart_transmit_interrupt(BUF_UART_t *uartPtr);

/**
 * @brief Initialise the serial port
 *
 * Initialise the uart library & peripheral, and set the flags for interrupt
 * Example: \n \code{.c}
 * // USART1 at 19200 Baud (one stopbit, no parity bit) 
 * #define UART_BUFFER_SIZE	128
 * #define UART_BAUD_RATE	19200
 *
 * uint8_t receiveBuffer[UART_BUFFER_SIZE];
 * uint8_t transmitBuffer[UART_BUFFER_SIZE];
 * BUF_UART_t uart_instance;
 * ...
 * // Configure interrupts for the USART peripheral, like this for example:
 * eclic_global_interrupt_enable();
 * eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
 * eclic_irq_enable(USART1_IRQn, 1, 0);
 * // now initialise this fifo uart library
 * uart_init(&uart_instance, USART1, UART_BAUD_RATE, SERIAL_8N1, \
 *     receiveBuffer, UART_BUFFER_SIZE, transmitBuffer, UART_BUFFER_SIZE);
 * \endcode
 * @param uartPtr		Pointer to a unused \a BUF_UART_t instance
 * @param _usart		Pointer / Address to the uart peripheral
 * @param _baud			The baudrate
 * @param _setting		The uart setting (bit length, parity, stop bits)
 * @param receivBuf		Pointer to the receive buffer
 * @param recivBufLen	Size of the receive buffer (max 255)
 * @param transBuf		Pointer to the transmit buffer
 * @param transBufLen	Size of the transmit buffer (max 255)
 */
void uart_init(BUF_UART_t *uartPtr, uint32_t _usart, uint32_t uartBaudRate, enum UART_SETTING _setting, uint8_t *recivBuf,  const uint8_t recivBufLen, uint8_t *transBuf, const uint8_t transBufLen);

/**
 * @brief Send a byte
 *
 * Puts a byte into the transmit buffer and enables the transmit interrupts.
 * If the buffer is full, it waits until free buffer space is available.
 * Example: \n \code{.c}
 * char data = '0';
 * uart_putc(&uart_instance, data);
 * uart_putc(&uart_instance, 'y');
 * \endcode 
 * @param uartPtr	Pointer to the initialised \a BUF_UART_t instance
 * @param data		8-bit value
 */
void uart_putc(BUF_UART_t *uartPtr, const uint8_t data);

/**
 * @brief Send multiple bytes
 *
 * Sends {bytesToSend} data pointed by {dataBuffer} out via the uart_putc function
 * Example: \n \code{.c}
 * uart_send(&uart_instance, &someArray[0], 12);
 * \endcode Send out 12 Bytes from someArray
 * @param uartPtr		Pointer to the initialised \a BUF_UART_t instance
 * @param dataBuffer	Pointer to the buffer/array
 * @param bytesToSend	Bytes of that pointer to send
 */
void uart_send(BUF_UART_t *uartPtr, const uint8_t *dataBuffer, const uint8_t bytesToSend);

/**
 * @brief Send a String
 *
 * Sends the data pointed at until a nullcharacter is detected (So a string, basically)
 * Example: \n \code{.c}
 * char *MyArray = "This is a Test";
 * uart_puts(&uart_instance, &MyArray);
 * uart_puts(&uart_instance, "Dies ist ein Test\n\r");
 * \endcode 
 * @param uartPtr	Pointer to the initialised \a BUF_UART_t instance
 * @param data		Pointer to the string byte array
 */
void uart_print(BUF_UART_t *uartPtr, const char *data);

/**
 * @brief Get error and status
 * 
 * Returns the sampled errors and the current status of the buffers.
 * Example: \n \code{.c}
 * UART_ERROR processError = uart_rxStatus(&uart_instance);
 * switch(processError)
 * {
	 *    case UART_DATA_AVAILABLE:
	 *       // Data available, everything cool
	 *       break;
	 *    case UART_NO_DATA:
	 *       // No data available
	 *       break;
	 * 	  case UART_FRAME_ERROR:
	 *        // UART Framing Error!
	 *       break;
	 *    case UART_OVERRUN_ERROR:
	 *       // UART Overrun condition Error
	 *       break;
	 *    case UART_PARITY_ERROR:
	 *       // UART Parity Error
	 *       break;
	 *    case UART_BUFFER_OVERFLOW:
	 *       // UART Receive Ringbuffer Overflow
	 *       break;
	 *    default:
	 *       // Mehrere Fehler auf ein Mal
	 *       break;
 * }
 * \endcode
 * @param uartPtr	Pointer to the initialised \a BUF_UART_t instance
 * @return 			Returns the status and errors back, see \a UART_ERROR for more
 */
UART_ERROR uart_rxStatus(BUF_UART_t *uartPtr);

/**
 * @brief Checking the next byte in the buffer without removing it
 *
 * Similar to \a uart_gets with the difference that the byte isn't removed from
 * the fifo-receive buffer.
 * @param uartPtr	Pointer to the initialised \a BUF_UART_t instance
 * @return			Returns the character in the lower byte and \a UART_ERROR in the upper byte
 */
uint16_t uart_peek(BUF_UART_t *uartPtr);

/**
 * @brief Read a byte
 *
 * Returns the last received byte from the receive fifo buffer. The upper 8 bits return the status
 * and possible errors back, while the lower 8 bits contain the data. Calling this function removes a 
 * received byte from the FIFO buffer (if there is any).
 * Example: \n \code{.c}
 * uint16_t data = uart_getc(&uart_instance);
 * char recevied_character;
 * switch(data >> 8)
 * {
 *    case UART_DATA_AVAILABLE:
 * 		 recevied_character = (char)data;
 *       // Data available, everything cool
 *       break;
 *    case UART_NO_DATA:
 *      // No data available
 *       break;
 * 	  case UART_FRAME_ERROR:
 *        // UART Framing Error!
 *       break;
 *    case UART_OVERRUN_ERROR:
 *       // UART Overrun condition Error
 *       break;
 *    case UART_PARITY_ERROR:
 *       // UART Parity Error
 *       break;
 *    case UART_BUFFER_OVERFLOW:
 *       // UART Receive Ringbuffer Overflow
 *       break;
 *    default:
 *       // Mehrere Fehler auf ein Mal
 *       break;
 * }
 * \endcode 
 * @param uartPtr	Pointer to the initialised \a BUF_UART_t instance
 * @return			Returns the character in the lower byte and \a UART_ERROR in the upper byte
 */
uint16_t uart_getc(BUF_UART_t *uartPtr);

/**
 * @brief Checks if the transmit buffer is full
 *
 * Checks if the transmit buffer is full, can be used to prevent the uart_putc function
 * from waiting / looping until a spot in the buffer would be free.
 * Example: \n\code{.c}
 * if(!(uart_isTxFull(&myUART)))
 * {
 *    uart_putc(&myUART, 0xAA);	// Send character if the buffer isn't full
 * }
 * \endcode 
 * @param uartPtr	Pointer to the initialised \a BUF_UART_t instance
 * @return 			Returns UART_TX_BUFFER_FULL if full
 */
UART_ERROR uart_isTxFull(BUF_UART_t *uartPtr);

/**
 * @brief Special character detected
 *
 * Checks if a special character has been detected.
 * Example: \n\code{.c}
 * if(uart_charDetected(&myUART))
 * {
 *     // Special character has been received, trigger some action
 * }
 * \endcode 
 * @param uartPtr	Pointer to the initialised \a BUF_UART_t instance
 * @return 			Returns the amount of bytes in the receive buffer from start up to the special character, else zero
 */
uint8_t uart_charDetected(BUF_UART_t *uartPtr);

/**
 * @brief Serch / Scan special character
 *
 * Checks within the receive interrupt routine if a certain character is found. That character can be defined with
 * this function. Use \a uart_charDetected to check if the character has been detected.
 * Example: \n\code{.c}
 * uart_searchForCharacter(&myUART, '\n');
 * ...
 * if(uart_charDetected(&myUART))
 * {
 *     // Special character has been received, trigger some action
 * }
 * \endcode 
 * @param uartPtr 		Pointer to the initialised \a BUF_UART_t instance
 * @param charToSearch	Character (char) to look out for
 */
void uart_searchForCharacter(BUF_UART_t *uartPtr, char charToSearch);

#endif /* GFC32VF103_UART_H_ */