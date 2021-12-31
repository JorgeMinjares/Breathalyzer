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
#define THRESHOLD 16384/2 //Threshold = 1.15V

bool exceedThreshold = 0;
uint8_t tic = 0;
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
    //LED
    P1->DIR |= BIT0 | BIT5;
    P1->OUT &= ~(BIT0 | BIT5);
    P1->SEL0 &= ~(BIT0 | BIT5);
    P1->SEL1 &= ~(BIT0 | BIT5);

    //ADC Channels
	P4->SEL0 |= BIT0;
	P4->SEL1 |= BIT0;
//------------------------------------------------------------------------
	//PWM Channels
	P2->DIR |= BIT4;
	P2->OUT |= BIT4;
	P2->SEL0 |= (BIT4);
	P2->SEL1 &= ~(BIT4);
//------------------------------------------------------------------------
	//Pull-down interrupt for restart
	P5->DIR &= ~(BIT0); // declare P5.0 as interrupt output
	P5->OUT &= ~(BIT0);//Clear P5.0
	P5->REN |= BIT0; // Enable internal resistance for interrupt
	P5->SEL0 &= ~(BIT0); //Clear Port from Special functions
	P5->SEL1 &= ~(BIT0);//Clear port from Special functions

	P5->IE |= BIT0; // Enable interrupt for P5.0
	P5->IES &= ~(BIT0);//Set interrupt as rising edge
	P5->IFG &= ~(BIT0);//Clear interrupt flag
//------------------------------------------------------------------------
	//SHP = Using signal from sampling timer
	//ON = Turn ADC On
	ADC14->CTL0 |= ADC14_CTL0_SHP | ADC14_CTL0_CONSEQ_0 | ADC14_CTL0_ON;
	ADC14->MCTL[0] = ADC14_MCTLN_INCH_13; // Enable P4.0 as ADC channel
	ADC14->CTL1 |= ADC14_CTL1_RES__14BIT; // Set ADC resolution as 14-bits
	ADC14->IER0 |= ADC14_IER0_IE0; // Enable ADC interrupt
//-------------------------------------------------------------------------
	//PWM and TIMER SET UP
	TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_0; // Tassel = SMCLK, MC = TIMER OFF
	TIMER_A0->CCR[0] |= 60000 - 1; // Set max Frequency for PWM, MAX = 50Hz
	TIMER_A0->CCR[1] |= 1 - 1;//Set up Frequency for P2.4
	TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_7;
	TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE;
//-------------------------------------------------------------------------
	NVIC->ISER[0] |= 1 << ((ADC14_IRQn) & 31);
	NVIC->ISER[1] |= 1 << ((PORT5_IRQn) & 31);
	NVIC->ISER[0] |= 1 << ((TA0_0_IRQn) & 31);


	__enable_irq();

	while(1){
	    if(exceedThreshold == true){
	        int i;
	        TIMER_A0->CCR[1] = 30000 - 1;
	        for(i = 0; i < 20; i++){
	            P1->OUT ^= BIT0 | BIT5;
	            __delay_cycles(3000000);
	        }
	        P1->OUT &= ~(BIT0 | BIT5);
	        TIMER_A0->CCR[1] = 1 - 1;
	        exceedThreshold = false;
	    }
	}
}
void TA0_IRQHandler(void){
    if(tic++ > 500){
        TIMER_A0->CCTL[0] &= ~(TIMER_A_CCTLN_CCIFG);
        TIMER_A0->CTL &= ~(TIMER_A_CTL_MC_1);
        tic = 0;
    }
    else{
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC; //ADC SC = Start Conversion, ADC ENC = Enable Conversion
    }
    TIMER_A0->CCTL[0] &= ~(TIMER_A_CCTLN_CCIFG);
}
void ADC14_IRQHandler(void){
    uint16_t rawData = ADC14->MEM[0];
    if(rawData > THRESHOLD){
        exceedThreshold = true;
    }
}
void PORT5_IRQHandler(void){
    uint8_t result = P5->IFG; // If interrupt is press restart system.
    if(result & BIT0){
        TIMER_A0->CTL |= TIMER_A_CTL_MC_1 ;
    }
    P5->IFG &= ~(result);
}
