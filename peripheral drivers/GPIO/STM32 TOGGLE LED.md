## This program toggles an LED connected to PORT A, PIN 5 of the STM32 microcontroller. It demonstrates:

### Enabling GPIOA peripheral clock using the RCC register.
+ Configuring GPIOA PIN 5 as an output.
+ Toggling the LED state by manipulating the Output Data Register (ODR).
+ LED Connected = PORT A PIN5

#### #include <stdint.h>

####  #define PERIPH_BASE         (0x40000000)

#### #define AHB1_PERIPH_OFFSET  (0x00020000)
#### #define AHB1_PERIPH_BASE    (PERIPH_BASE + AHB1_PERIPH_OFFSET)

#### #define GPIOA_OFFSET        (0x0000)
#### #define GPIOA_BASE          (AHB1_PERIPH_BASE + GPIOA_OFFSET)

#### #define RCC_OFFSET          (0x3800)
#### #define RCC_BASE            (AHB1_PERIPH_BASE + RCC_OFFSET)

#### #define RCC_OFFSET_AHB1EN_R (0x30)
#### #define RCC_AHB1EN_R (*(volatile unsigned int *)(RCC_BASE + RCC_OFFSET_AHB1EN_R))

#### #define GPIOAEN             (1<<0)
#### #define GPIOA_OFFSET_MODE_R (0x00)

#### #define GPIOA_MODE_R        (*(volatile unsigned int *)(GPIOA_BASE + GPIOA_OFFSET_MODE_R))

#### #define OFFSET_OD_R         (0x14)
#### #define OD_R                (*(volatile unsigned int *)(GPIOA_BASE +  OFFSET_OD_R))

#### #define PIN5                (1<<5)
#### #define LED_PIN             PIN5

#### typedef struct
#### {
	volatile uint32_t MODER;
	volatile uint32_t Dummy[4];
	volatile uint32_t ODR;
#### } GPIO_REG;

#### typedef struct
#### {
	volatile uint32_t DUMMY[12];
	volatile uint32_t AHB1ENR;
#### #} RCC_REG;

#### #define RCC    ((RCC_REG*)(RCC_BASE))
#### #define GPIOA  ((GPIO_REG*)(GPIOA_BASE))

#### int main(void)
#### {
	RCC->AHB1ENR |= GPIOAEN;         // Enable GPIOA clock
	GPIOA->MODER |= (1<<10);         // Set PA5 as output mode
	GPIOA->MODER &= ~(1<<11);        // Clear bit 11 to complete configuration as output

	while(1)
	{
		GPIOA->ODR ^= (1<<5);       // Toggle PA5
	}
#### }
### Explanation
#### Key Definitions:
###### Peripheral Base Address: 0x40000000
+ The base address of all peripherals in the STM32 memory map.
+ AHB1 Peripheral Base Address: 
PERIPH_BASE + 0x00020000
+ Base address for AHB1 peripherals (GPIOA, RCC, etc.).
+ GPIOA_BASE: AHB1_PERIPH_BASE + GPIOA_OFFSET
+ Base address for GPIOA peripheral.
## RCC (Reset and Clock Control):
### RCC->AHB1ENR:
Enables the clock for GPIOA by setting the 0th bit.
## GPIOA Configuration:
### GPIOA->MODER:
#### Configures PIN 5 as an output by setting MODER10 to 1 and MODER11 to 0.

### GPIOA->ODR:
#### Toggles the output state of PIN 5, which controls the LED.

## Code Breakdown:
### Enable GPIOA Clock:

###### RCC->AHB1ENR |= GPIOAEN;
### Set PA5 as Output:

###### GPIOA->MODER |= (1<<10);    // Set bit 10
###### GPIOA->MODER &= ~(1<<11);   // Clear bit 11
### Toggle LED in Infinite Loop:

###### GPIOA->ODR ^= (1<<5);       // Toggle the state of PIN 5
## Expected Behavior
The LED connected to PORT A, PIN 5 will toggle (turn ON and OFF repeatedly).
The toggling rate depends on the clock speed and may need a delay function for visible blinking.
## Notes

Ensure that an LED is physically connected to PORT A, PIN 5 with an appropriate resistor to prevent damage.
Adjust clock configurations as needed if using a custom setup.