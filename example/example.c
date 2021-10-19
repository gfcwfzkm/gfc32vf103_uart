/**
 * USART FIFO library example
 * This example shows the basic usage of the FIFO serial library for
 * the GD32V RISC-V microcontrollers. This demo uses the USART 1 peripheral
 * at PortA pin 2 (tx) and 3 (rx)
 * 
 */

#include "gd32vf103.h"
#include "gfc32vf103_uart.h"	// Include library
 
/* USART pins */
#define	MCU_TX	BIT(2)
#define MCU_RX	BIT(3)

/* Buffer sizes, for this example lets use different buffer sizes */
#define USART1_TXBUF	100
#define USART1_RXBUF	128

/* FIFO UART struct */
BUF_UART_t fifoUART;
uint8_t receiveBuffer[USART1_RXBUF];
uint8_t transmitBuffer[USART1_TXBUF];

/* USART1 Interrupt handler, call the library's interrupt function */
void USART1_IRQHandler(void)
{
	uart_interrupt(&fifoUART);
}

int main(void)
{
	uint16_t ch;

	/* Enable the required peripheral clocks */
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_USART1);

	/* Initalise the UART pins */
	gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_2MHZ, MCU_TX);
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, MCU_RX);

	/* Configure & enable interrupts for the USART peripheral */
	eclic_global_interrupt_enable();
	eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
	eclic_irq_enable(USART1_IRQn, 1, 0);

	/* Initialise the library: */
	uart_init(&fifoUART, USART1, 460800, SERIAL_8N1, 
		receiveBuffer, USART1_RXBUF, transmitBuffer, USART1_TXBUF);
	
	uart_print(&fifoUART, "\r\nUART loop demo on RISC-V GD32VF103:\r\n");

	while(1)
	{
		/* Lets loop forever, check for received characters and 
		 * send them directly back */
		ch = uart_getc(&fifoUART);
		if ((ch >> 8) == UART_DATA_AVAILABLE)
		{
			// We got data - send it back
			uart_putc(&fifoUART, (uint8_t)ch);
		}
		else if ((ch >> 8) == UART_NO_DATA)
		{
			// no data here to handle
		}
		else
		{
			// Process errors here
		}
	}
}