/**
	\file
	\brief
		This is the starter file of FlexTimer.
		In this file the FlexTimer is configured in overflow mode.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	7/09/2014
	\todo
	    Add configuration structures.
 */



#include "FlexTimer.h"
#include "MK64F12.h"
/*
void FlexTimer_updateCHValue(sint16 channelValue)
{
	/**Assigns a new value for the duty cycle*/
	//FTM0->CONTROLS[0].CnV = channelValue;
//}


void FlexTimer_Init()
{
	/** Clock gating for the FlexTimer 0*/
		SIM->SCGC6 |= FLEX_TIMER_0_CLOCK_GATING;
		SIM->SCGC3 |= FLEX_TIMER_3_CLOCK_GATING;

		/**When write protection is enabled (WPDIS = 0), write protected bits cannot be written.
		* When write protection is disabled (WPDIS = 1), write protected bits can be written.*/
		FTM0->MODE |= FLEX_TIMER_WPDIS;
		/**Enables the writing over all registers*/
		FTM0->MODE &= ~FLEX_TIMER_FTMEN;
		/**Assigning a default value for modulo register*/
		FTM0->MOD = 975;
		/**Selects the Edge-Aligned PWM mode mode*/
		FTM0->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		/**Assign a duty cycle of 50%*/
		FTM0->CONTROLS[0].CnV = 48;

		FTM0->CONTROLS[2].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		/**Assign a duty cycle of 50%*/
		FTM0->CONTROLS[2].CnV = 48;

		FTM0->SC = FLEX_TIMER_CLKS_2|FLEX_TIMER_PS_1;

		FTM3->MODE |= FLEX_TIMER_WPDIS;
		/**Enables the writing over all registers*/
		FTM3->MODE &= ~FLEX_TIMER_FTMEN;
		/**Assigning a default value for modulo register*/
		FTM3->MOD = 975;
		/**Selects the Edge-Aligned PWM mode mode*/
		FTM3->CONTROLS[4].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		/**Assign a duty cycle of 50%*/
		FTM3->CONTROLS[4].CnV = 48;

		/**Selects the Edge-Aligned PWM mode mode*/
		FTM3->CONTROLS[5].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		/**Assign a dut cycle of 50%*/
		FTM3->CONTROLS[5].CnV = 48;

		/**Configure the times*/
		FTM3->SC = FLEX_TIMER_CLKS_2|FLEX_TIMER_PS_1;
}



