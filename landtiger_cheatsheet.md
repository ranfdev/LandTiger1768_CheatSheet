# Landtiger LPC1768 ARM Cortex M3 - CheatSheet
**Revalor Riccardo - [Github Repo for this CheatSheet](https://github.com/RiccardoRevalor/LandTiger1768_CheatSheet)**

## Power Down Mode
Put at the end of `sample.c`:
```c
LPC_SC->PCON |= 0x1; /* power-down	mode */
LPC_SC->PCON &= ~(0x2);						
		
while (1) { /* Loop forever */	
	__ASM("wfi");
}
```

## RIT Timer
### Remember: if you can use RIT instead of the other 4 timers, USE RIT!
In `sample.c` you have to call the function to set the RIT timer:
```c
init_RIT(0x004C4B40);	/* RIT Initialization 50 msec, suitable for human input*/
enable_RIT();	/* RIT enabled */
```

### RIT header file 
#### File: `RIT.h`
```c
#ifndef __RIT_H
#define __RIT_H

/* lib_RIT.c */
extern uint32_t init_RIT( uint32_t RITInterval );
extern void enable_RIT( void );
extern void disable_RIT( void );
extern void reset_RIT( void );
/* IRQ_RIT.c */
extern void RIT_IRQHandler (void);

#endif /* end __RIT_H */
```

### RIT Initialization, enable, disable, reset
#### File: `lib_RIT.c`
```c
#include "LPC17xx.h"
#include "RIT.h"

void enable_RIT( void )
{
  LPC_RIT->RICTRL |= (1<<3);	
  return;
}

void disable_RIT( void )
{
	LPC_RIT->RICTRL &= ~(1<<3);	
  return;
}

void reset_RIT( void )
{
  LPC_RIT->RICOUNTER = 0;          // Set count value to 0
  return;
}

uint32_t init_RIT ( uint32_t RITInterval )
{
  
	
  LPC_SC->PCLKSEL1  &= ~(3<<26);
  LPC_SC->PCLKSEL1  |=  (1<<26);   // RIT Clock = CCLK
	LPC_SC->PCONP     |=  (1<<16);   // Enable power for RIT
	
	LPC_RIT->RICOMPVAL = RITInterval;      // Set match value		
	LPC_RIT->RICTRL    = (1<<1) |    // Enable clear on match	
											 (1<<2) ;		 // Enable timer for debug	
	LPC_RIT->RICOUNTER = 0;          // Set count value to 0
	
	NVIC_EnableIRQ(RIT_IRQn);
	NVIC_SetPriority(RIT_IRQn,4);
  return (0);
}

```

### RIT_IRQHandler()
#### File:  `IRQ_RIT.c` 
You can put whatever you want to perform the actions at every interval of the RIT.

## JOYSTICK 
In `sample.c` you have to call the function to set both joystick and RIT timer:
```c
joystick_init()         /* initialize joystick GPIO pins */
init_RIT(0x004C4B40);	/* RIT Initialization 50 msec, suitable for human input*/
enable_RIT();	/* RIT enabled */
```
### Joystick Initialization -> `joystick_init() `
#### File: `libjoystick.c`
Pin Sequence: DOWN (1.26), LEFT (1.27), RIGHT (1.28), UP (1.29)
```c
void joystick_init(void) {

    // Set joystick DOWN (pin 1.26)
    LPC_PINCON->PINSEL3 &= ~(3<<20); // SET pin 20, 21 to 00 in PINSEL3
    LPC_GPIO1->FIODIR   &= ~(1<<26); // P1.26 Input

    // Set joystick LEFT (pin 1.27)
    LPC_PINCON->PINSEL3 &= ~(3<<22); // SET pin 22, 23 to 00 in PINSEL3
    LPC_GPIO1->FIODIR   &= ~(1<<27);

    // Set joystick RIGHT (pin 1.28)
    LPC_PINCON->PINSEL3 &= ~(3<<24); // SET pin 24, 25 to 00 in PINSEL3
    LPC_GPIO1->FIODIR   &= ~(1<<28);

    // Set joystick UP (pin 1.29)
    LPC_PINCON->PINSEL3 &= ~(3<<26); // SET pin 26, 27 to 00 in PINSEL3
    LPC_GPIO1->FIODIR   &= ~(1<<29); // P1.29 Input (joystick on PORT1 defined as Input)

}
```

### Joystick RIT Polling -> `RIT_IRQHandler()`
#### File: `IRQ_RIT.c` 
The `if case` for Joystick UP contains an example using LEDs and considering continued pressing (when up == 60, so after 3 sec of continued pressing):
```c
void RIT_IRQHandler (void)
{					
	static int up=0;
	static int position=0;
	static int down_joystick = 0;
	
	if((LPC_GPIO1->FIOPIN & (1<<29)) == 0){	
		/* Joytick UP pressed */
		up++;
		switch(up){
			case 1:
				//turn off the led of currrent position and on the led at position 0
				LED_Off(position);
				LED_On(0);
				position = 0;
				break;
			case 60:	//3sec = 3000ms/50ms = 60
				//60: after 3 sec
				/*
			check after 3 seconds: 
			Tpoll = 50 ms (period of polling)
			Tcheck = 3 seconds = 3000 ms
			1 interval takes 50 ms
			2 interval take 100 ms
			4 intervals take 200 ms
			So I have to wait for 3 seconds is simply Tcheck / Tpoll
			So 3000 ms / 50 ms = 60 intervals of 50 ms each!!!
				*/
				LED_Off(position);
				LED_On(7);
				position = 7;
				break;
			default:
				break;
		}
	}
	else{
			up=0;
	}
		
	if((LPC_GPIO1->FIOPIN & (1<<26)) == 0){	
		/* Joytick DOWN pressed */
		down_joystick++;
	}
	else{
		down_joystick=0;
	}
	
	if ((LPC_GPIO1->FIOPIN & (1<<27)) == 0){	
        /* Joytick LEFT pressed */
		down++; //do something...
	}

    if ((LPC_GPIO1->FIOPIN & (1<<28)) == 0){	
        /* Joytick RIGHT pressed */
		down--;
	}
}
```
## ADC
Using the SIMULATOR, To disable potentiometer *non idealities* you have to go the the emulator Settings (rx click on emulator window)> uncheck *Enable/Disable Potentiometer non ideality* <br>
In `sample.c` you have to call the function to set the ADC:
```c
ADC_init();     /* ADC Initialization */
```
### ADC Header file
#### File: `adc.h`
```c
#include <string.h>

/* lib_adc.c */
void ADC_init (void);
void ADC_start_conversion (void);

/* IRQ_adc.c */
void ADC_IRQHandler(void);
```

### ADC Initialization and start_conversion -> `ADC_init()`, `ADC_start_conversion()`
#### File: `lib_adc.c`
AD Converter 0 (AD0) PINS: 
| PIN          | PINSEL       | AD0          |
|--------------|--------------|--------------|
| P0[23] (0.23)       | PINSEL1      | AD0[0]       |
| P0[24]       | PINSEL1       | AD0[1]       |
| P0[25]       | PINSEL1       | AD0[2]       |
| P0[26]       | PINSEL1       | AD0[3]       |
| P1[30]       | PINSEL3       | AD0[4]       |
| P1[31]       | PINSEL3       | AD0[5]       |

ADC_init() for enabling AD0.5 channel:
```c
void ADC_init (void) {
	
	//In this case I USE P1.31 AS AD0.5
	
	//setup pin function to ADC
  LPC_PINCON->PINSEL3 |=  (3UL<<30);      /* P1.31 is AD0.5                     */
	
	//By defaut, ADC is disabled to save energy, so I have to enabled it
	//BIT 12 IN PCONP IS THE ADC 
  LPC_SC->PCONP       |=  (1<<12);      /* Enable power to ADC block          */

  LPC_ADC->ADCR        =  (1<< 5) |     /* select AD0.5 pin -> set bit 5 to 1 to use AD0 channel 5                 */
                          (4<< 8) |     /* set 4. ADC clock is 25MHz/4    -> it is due to reach the maximum limitation for freq for the board           */
                          (1<<21);      /* enable ADC                         */ 

	/*
	enable interrupt on  AD0.5 
	Set bit 5 of AD0INTEN to 1
	In this case I do a global interrupt enable by setting to 1 bit 8 of AD0INTEN
	*/
  LPC_ADC->ADINTEN     =  (1<< 8);      /* global enable interrupt            */

  NVIC_EnableIRQ(ADC_IRQn);             /* enable ADC Interrupt               */
}
```
```c
void ADC_start_conversion (void) {
	/*
	to start conversion I have to act on ADCR register
	set bit 24 to 1. If I set 24 to 1 it means I wanna start the conversion NOW
	*/
	LPC_ADC->ADCR |=  (1<<24);            /* Start A/D Conversion 				*/
}	
```
### ADC Handler -> `ADC_IRQHandler()`
#### File:  `IRQ_adc.c` 
Executed after a signal is received from the ADC itself.
```c
#include "LPC17xx.h"
#include "adc.h"
#include "../led/led.h"
unsigned short AD_current;   				/* Current converted value */
unsigned short AD_last = 0xFF;     /* Last converted value */

void ADC_IRQHandler(void) {	
	/*
	Read the ADGDR Register (Global data reg of ADC), shift it to the right by 4
	because result is store at bit 4 to 15.
	Then I perform the and with twelve ones to get just the bits of the result.
	I AM ONLY INTERESTED IN THE BITS OF RESULT, THAT'S WHY I PERFORM ALSO THE AND TO SET TO ZERO ALL THE OTHER BITS
	*/
		
  AD_current = ((LPC_ADC->ADGDR>>4) & 0xFFF);/* Read Conversion Result             */
  if(AD_current != AD_last){
		//if current result is different from the left, save it
		LED_Off(AD_last*7/0xFFF);	  // ad_last : AD_max = x : 7 		LED_Off((AD_last*7/0xFFF));	
		LED_On(AD_current*7/0xFFF);	// ad_current : AD_max = x : 7 		LED_On((AD_current*7/0xFFF));	

        //the '*7/0xFFF' is used for scaling the value read from the potentiometer to the number of LEDs (7)
        //Map the value of the potentiometer to the one of the LEDs vector
		
		AD_last = AD_current;
  }	
}
```
### Use of RIT for multiple conversions at a certain rate
The conversion period is set by the RIT Interval! <br>
With a RIT Interval of 50 ms, the Conversion rate is set at 200 Hz. <br>
Initialize the RIT as seen in the [RIT Initialization, enable, disable, reset](#rit-initialization-enable-disable-reset), then start the ADC Conversion each time the [RIT_IRQHandler()](#rit_irqhandler) is triggered:
```c
void RIT_IRQHandler (void) {								
	/* ADC management */
	ADC_start_conversion();		
			
    LPC_RIT->RICTRL |= 0x1;	/* clear interrupt flag */
}
```

### Resources
#### PINSEL Register associated to each pin:
![alt text](image-1.png)

#### PCONP Register bits:
![alt text](image-2.png)

#### ADCR Register bits:
![alt text](image-3.png)

#### ADGDR (ADC Global Data) Register bits:
![alt text](image-4.png)