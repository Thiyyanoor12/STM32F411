# STM32F4xx LED Blinking Example

#### This is a simple example for STM32F4xx microcontroller that toggles an LED connected to pin PA5 (GPIOA Pin 5) with a delay. It uses the STM32F4xx HAL (Hardware Abstraction Layer) to directly manipulate the GPIO register and control the LED.

## Code Overview

The code performs the following:

1. **Enables the clock for GPIOA**: This is done by setting the appropriate bit in the `RCC->AHB1ENR` register.
2. **Configures GPIOA Pin 5 as output**: This is done by setting the MODER (Mode Register) bits for pin 5 to output mode.
3. **Toggles the LED state**: The LED connected to pin 5 is toggled using the BSRR (Bit Set/Reset Register).
4. **Introduces a delay**: A simple delay loop is used to create a visible blink effect.
## Code

```c
#include <stm32f4xx.h>

#define GPIOAEN (1 << 0)     // Enable clock for GPIOA
#define PIN5    (1 << 5)     // Pin 5
#define LED_PIN PIN5         // LED connected to Pin 5

int main(void)
{
    // Enable GPIOA clock
    RCC->AHB1ENR |= GPIOAEN;

    // Configure Pin 5 as output (MODER)
    GPIOA->MODER |= (1 << 10);    // Set bit 10
    GPIOA->MODER &= ~(1 << 11);   // Clear bit 11

    while(1)
    {
        // Toggle LED (Set/Reset Pin 5)
        GPIOA->BSRR ^= LED_PIN;    // Toggle LED
        for(int i = 0; i <= 1000000; i++) {}  // Delay loop

        // Toggle LED (Set/Reset Pin 5)
        GPIOA->BSRR ^= (1 << 21);  // Reset LED (Pin 5)
        for(int i = 0; i <= 1000000; i++) {}  // Delay loop
    }
}
```
## Description
1. **Clock Enable (GPIOA)**:
The first step is to enable the clock for GPIOA. This is done by setting the 0th bit in the RCC->AHB1ENR register, which allows us to access GPIOA for configuration.

```c
RCC->AHB1ENR |= GPIOAEN;
```
2. **Configure GPIOA Pin 5 as Output**:
The GPIOA pin 5 is configured as an output pin. The MODER register controls the mode of each GPIO pin. Here, we set bits 10 and 11 to configure pin 5 as an output pin.

```c
GPIOA->MODER |= (1 << 10);  // Set bit 10
GPIOA->MODER &= ~(1 << 11); // Clear bit 11
```
3. **Toggle LED Pin:**
The BSRR register is used to toggle the LED. Writing a 1 to the 5th bit of BSRR will set the corresponding pin (PA5) to high, and writing a 1 to the 21st bit will reset it to low.
```c
GPIOA->BSRR ^= LED_PIN;  // Toggle LED
```
4. **Delay Loop:**
A simple delay is introduced using an empty for loop that will create a visible blinking effect. This delay is not precise and is only used to allow the LED to remain on or off for a short period.

```c
for(int i = 0; i <= 1000000; i++) {}
```
## Conclusion
This example demonstrates how to control an LED on STM32F4xx using direct register manipulation. The LED toggles on and off with a delay, which can be modified to adjust the blinking speed.

