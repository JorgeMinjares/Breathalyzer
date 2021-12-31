/**
 * @file main.c
 * @author Jorge Minjares (https://github.com/JorgeMinjares)
 * @brief This program will emulate a Breathalyzer using the MSP432.
 * @version 0.1
 * @date 2021-12-23
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "msp.h"
#include <stdbool.h>
#define THRESHOLD 14500 // Threshold

bool exceedThreshold = 0;
uint16_t tic = 0;
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer
	// Enable GPIO P1.0 and P1.5
	P1->DIR |= BIT0 | BIT5;		// Delacre P1.0 and P1.5 as outputs
	P1->OUT &= ~(BIT0 | BIT5);	// Clear previous configuration of P1.0 and P1.5
	P1->SEL0 &= ~(BIT0 | BIT5); // Disable Sepcial fucntion for P1.0 and P1.5
	P1->SEL1 &= ~(BIT0 | BIT5); // Disabble Special Functions for P1.0 and P1.5

	// ADC Channel Enabled
	P4->SEL0 |= BIT0; // Enable Special Functions for P4.0
	P4->SEL1 |= BIT0; // Enable Special Functions for P4.0

	// Buzzer and Green LED Enabled
	P2->DIR |= BIT6 | BIT1;		// Declare P2.1 and P2.6 as outputs
	P2->OUT &= ~(BIT6 | BIT1);	// Clear previous configuration of P2.1 and P2.6
	P2->SEL0 &= ~(BIT6 | BIT1); // Disable Sepcial fucntion for P2.1 and P2.6
	P2->SEL1 &= ~(BIT6 | BIT1); // Disable Sepcial fucntion for P2.1 and P2.6

	// Pull-down interrupt for restart
	P5->DIR &= ~(BIT0);	 // declare P5.0 as interrupt output
	P5->OUT &= ~(BIT0);	 // Clear P5.0
	P5->REN |= BIT0;	 // Enable internal resistance for interrupt
	P5->SEL0 &= ~(BIT0); // Clear Port from Special functions
	P5->SEL1 &= ~(BIT0); // Clear port from Special functions

	P5->IE |= BIT0;		// Enable interrupt for P5.0
	P5->IES &= ~(BIT0); // Set interrupt as rising edge
	P5->IFG &= ~(BIT0); // Clear interrupt flag

	// SHP = Using signal from sampling timer
	// ON = Turn ADC On
	ADC14->CTL0 |= ADC14_CTL0_SHP | ADC14_CTL0_CONSEQ_0 | ADC14_CTL0_ON;
	ADC14->MCTL[0] = ADC14_MCTLN_INCH_13; // Enable P4.0 as ADC channel
	ADC14->CTL1 |= ADC14_CTL1_RES__14BIT; // Set ADC resolution as 14-bits
	ADC14->IER0 |= ADC14_IER0_IE0;		  // Enable ADC interrupt

	// PWM and TIMER SET UP
	TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_0; // Tassel = SMCLK, MC = TIMER OFF
	TIMER_A0->CCR[0] = 60000 - 1;							 // Set max Frequency for PWM, MAX = 50Hz
	TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;					 // Timer interrupt Enable

	NVIC->ISER[0] |= 1 << ((ADC14_IRQn)&31); // Enable NVIC for ADC channel
	NVIC->ISER[1] |= 1 << ((PORT5_IRQn)&31); // Enable NVIC for P5.0 interrupt
	NVIC->ISER[0] |= 1 << ((TA0_0_IRQn)&31); // Enable NVIC for Timer Interrupt

	__enable_irq(); // Enable Global Interrupts

	while (1)
	{
		if (exceedThreshold == true)
		{
			int i;
			P2->OUT |= BIT6; // Buzzer is activated
			for (i = 0; i < 10; i++)
			{
				P1->OUT ^= BIT0 | BIT5; // Toggle LED and Motor
				__delay_cycles(300000);
			}
			P1->OUT &= ~(BIT0 | BIT5); // Clear LED and Motor
			P2->OUT &= ~(BIT6);		   // Clear Buzzer output
			exceedThreshold = false;
		}
		__delay_cycles(10000);
	}
}
void TA0_0_IRQHandler(void)
{
	if (tic++ > 500) // If Timer exceeds 10 seconds
	{
		P2->OUT &= ~(BIT1);							 // Turn off Green LED
		tic = 0;									 // Restart Timer counter
		TIMER_A0->CCTL[0] &= ~(TIMER_A_CCTLN_CCIFG); // Clear Timer Flag
		TIMER_A0->CTL &= ~(TIMER_A_CTL_MC_1);		 // Turn off Timer
	}
	else
	{
		ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC; // ADC SC = Start Conversion, ADC ENC = Enable Conversion
	}
	TIMER_A0->CCTL[0] &= ~(TIMER_A_CCTLN_CCIFG); // Clear Timer Flag
}
void ADC14_IRQHandler(void)
{
	uint16_t rawData = ADC14->MEM[0]; // Store ADC reading
	if (rawData > THRESHOLD - 10)	  // If the reading is greater than the threshold
	{
		exceedThreshold = true; // Set statement as True
	}
}
void PORT5_IRQHandler(void)
{
	uint8_t result = P5->IFG; // If interrupt is press start timer
	if (result & BIT0)
	{

		P2->OUT |= BIT1;				   // Turn on Greed LED
		TIMER_A0->CTL |= TIMER_A_CTL_MC_1; // Timer starts
	}
	P5->IFG &= ~(result); // Clear interrupt flag
}
