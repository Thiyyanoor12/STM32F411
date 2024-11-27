# # STM32F4 UART Communication Program

## Overview

This project demonstrates the usage of UART communication on an STM32F4 series microcontroller (specifically STM32F411). The program configures the UART2 interface, sets up GPIO pins for USART TX and RX, initializes UART settings, and sends a simple message via UART.
### Key Features:
- **UART2 communication**: Used for transmitting and receiving data.
- **GPIO configuration**: Set up PA2 for UART TX and PA3 for UART RX.
- **Baud rate configuration**: Configures UART2 with a baud rate of 115200.
- **Polling method for transmission**: Waits for the TX register to be empty before sending data.

## File Structure

### 1. `main.c`

This file contains the main entry point of the program. The program initializes UART2 communication and sends a string via UART.
#### Key Functions:
- `main`: Initializes UART2 and sends a message in an infinite loop.

```c
int main()
{
    Uart2_Tx_init();
    while(1){
        USART_Write('t');
        printf("This is STM32F411X...");
    }
}
```
## 2. **uart.c**
This file contains the functions related to UART configuration, data transmission, and baud rate setup.

### Key Functions:
- **Uart2_Tx_init:** Initializes UART2 and configures the GPIO pins for TX and RX.

- **USART_Write:** Writes a character to the UART data register (DR) for transmission.

- **io_putchar:** A wrapper function to send characters via USART.
- **USART_Read:** Reads a character from the UART data register.
- **Baudrate_cal:** Calculates the baud rate register value based on the peripheral clock and the desired baud rate.
- **UART_Set_Baudrate:** Configures the baud rate for USART2.

```c
void Uart2_TxRX_init(void)
{
	/**********configure uart GPIO pin**********/
	/******enable clock access to GIPOA*******/
	RCC->AHB1ENR = GPIOAEN;
	/************set PA2 to AF mode********/
		GPIOA->MODER &= ~(1<<4);
		GPIOA->MODER |= (1<<5);
	/********set PA3 to AF mode*************/
		GPIOA->MODER &= ~(1<<6);
		GPIOA->MODER |= (1<<7);

	/********************set PA2 to AF type USART TX (AF07)********/
		GPIOA->AFR[0] |= (1<<8);
		GPIOA->AFR[0] |= (1<<9);
		GPIOA->AFR[0] |= (1<<10);
		GPIOA->AFR[0] &= ~(1<<11);
	/********************set PA3 to AF type USART TX (AF07)********/
		GPIOA->AFR[0] |= (1<<12);
		GPIOA->AFR[0] |= (1<<13);
		GPIOA->AFR[0] |= (1<<14);
		GPIOA->AFR[0] &= ~(1<<15);
		/*********configure uart mod*********/
		/******enable clock access to USART2*******/
		RCC->APB1ENR = UART2;
		/*******CONFIG BAUD RATE**********/
		UART_Set_Baudrate(USART2,APB1CLK,UART_BAUDRATE);
		/******CONFIG TRANSFER FUN**********/
		USART2->CR1 = CR1_TE;
		/******CONFIG TRANSFER FUN**********/
	    USART2->CR1 = CR1_RE;
		/*********ENABLE UART MOD*********/
		USART2->CR1 = CRI_UE;
}
void Uart2_Tx_init(void)
{
	/**********configure uart GPIO pin**********/
	/******enable clock access to GIPOA*******/
	RCC->AHB1ENR = GPIOAEN;
	/************set PA2 to AF mode********/
		GPIOA->MODER &= ~(1<<4);
		GPIOA->MODER |= (1<<5);
	/********************set PA2 to AF type USART TX (AF07)********/
		GPIOA->AFR[0] |= (1<<8);
		GPIOA->AFR[0] |= (1<<9);
		GPIOA->AFR[0] |= (1<<10);
		GPIOA->AFR[0] &= ~(1<<11);
		/*********configure uart mod*********/
		/******enable clock access to USART2*******/
		RCC->APB1ENR = UART2;
		/*******CONFIG BAUD RATE**********/
		UART_Set_Baudrate(USART2,APB1CLK,UART_BAUDRATE);
		/******CONFIG TRANSFER FUN**********/
		USART2->CR1 = CR1_TE;
		/*********ENABLE UART MOD*********/
		USART2->CR1 = CRI_UE;
}
static uint16_t Baudrate_cal(uint32_t periphclk,uint32_t baudrate)
{
	return ((periphclk + (baudrate/2)/baudrate));

}
void USART_Write(int ch) {
    // Wait until the Transmit Data Register (TXE) is empty
    while (!(USART2->SR & USART_SR_TXE)) {
        // Do nothing until TXE is set
    }

    // Send the character by writing it to the Data Register (DR)
    USART2->DR = ch & 0xFF;  // Mask to ensure only the lower 8 bits are sent
}
void UART_Set_Baudrate(USART_TypeDef *USARTx,uint32_t periphclk, uint32_t baudrate)
{
	USARTx->BRR=Baudrate_cal(periphclk, baudrate);
}
void io_putchar(int ch)
{
	USART_Write(ch);
	return ch;

}
void USART_Read(int ch)
{
	while (!(USART2->SR & USART_SR_RXNE)) {
	        // Do nothing until TXE is set
	    }
	return USART2->DR;

}
```

## GPIO and USART Configuration

### GPIOA: Configured for USART2 TX (PA2) and RX (PA3).

- **PA2 is set to Alternate Function (AF) mode to allow USART2 TX communication.**
- **PA3 is set to Alternate Function (AF) mode to allow USART2 RX communication.**

### USART2: Configured for serial communication.
- **Baud rate: Set to 115200.**
- **Transmission and reception: Enabled with appropriate control registers (CR1 for enabling TX/RX, BRR for baud rate).**
## Dependencies

- **This program is built for the STM32F4 series microcontroller, specifically the STM32F411. It relies on the STM32 HAL and peripheral drivers for low-level hardware access.**
## How to Use
- **Set up the STM32 development environment (e.g., STM32CubeIDE or KEIL).**
- **Compile and flash the program to the STM32F4 microcontroller.**
- **A UART-to-USB adapter or an oscilloscope to the TX and RX pins to observe the UART communication.**
- **The message "This is STM32F411X..." will be sent continuously over UART.**