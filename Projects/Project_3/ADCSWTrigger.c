// ADCSWTrigger.c
// Runs on LM4F120/TM4C123
// Provide functions that initialize ADC0 SS3 to be triggered by
// software and trigger a conversion, wait for it to finish,
// and return the result.
// Daniel Valvano
// October 20, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#define ADC0_ACTSS_R            (*((volatile unsigned long *)0x40038000))
#define ADC0_RIS_R              (*((volatile unsigned long *)0x40038004))
#define ADC0_IM_R               (*((volatile unsigned long *)0x40038008))
#define ADC0_ISC_R              (*((volatile unsigned long *)0x4003800C))
#define ADC0_EMUX_R             (*((volatile unsigned long *)0x40038014))
#define ADC0_SSPRI_R            (*((volatile unsigned long *)0x40038020))
#define ADC0_CTL_R              (*((volatile unsigned long *)0x40038038))
#define ADC0_SSMUX2_R           (*((volatile unsigned long *)0x40038080))
#define ADC0_SSCTL2_R           (*((volatile unsigned long *)0x40038084))
#define ADC0_SSFIFO2_R          (*((volatile unsigned long *)0x40038088))
#define ADC0_PC_R               (*((volatile unsigned long *)0x40038FC4))
	
#define GPIO_PORTE_DATA_BITS_R  ((volatile unsigned long *)0x40024000)
#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_IS_R         (*((volatile unsigned long *)0x40024404))
#define GPIO_PORTE_IBE_R        (*((volatile unsigned long *)0x40024408))
#define GPIO_PORTE_IEV_R        (*((volatile unsigned long *)0x4002440C))
#define GPIO_PORTE_IM_R         (*((volatile unsigned long *)0x40024410))
#define GPIO_PORTE_RIS_R        (*((volatile unsigned long *)0x40024414))
#define GPIO_PORTE_MIS_R        (*((volatile unsigned long *)0x40024418))
#define GPIO_PORTE_ICR_R        (*((volatile unsigned long *)0x4002441C))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DR2R_R       (*((volatile unsigned long *)0x40024500))
#define GPIO_PORTE_DR4R_R       (*((volatile unsigned long *)0x40024504))
#define GPIO_PORTE_DR8R_R       (*((volatile unsigned long *)0x40024508))
#define GPIO_PORTE_ODR_R        (*((volatile unsigned long *)0x4002450C))
#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_PDR_R        (*((volatile unsigned long *)0x40024514))
#define GPIO_PORTE_SLR_R        (*((volatile unsigned long *)0x40024518))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_LOCK_R       (*((volatile unsigned long *)0x40024520))
#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define GPIO_PORTE_ADCCTL_R     (*((volatile unsigned long *)0x40024530))
#define GPIO_PORTE_DMACTL_R     (*((volatile unsigned long *)0x40024534))
	
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGCADC_R        (*((volatile unsigned long *)0x400FE638))
#define ADC0_PSSI_R             (*((volatile unsigned long *)0x40038028))



	
// This initialization function sets up the ADC according to the
// following parameters.  Any parameters not explicitly listed
// below are not modified:
// Max sample rate: <=125,000 samples/second
// Sequencer 0 priority: 1st (highest)
// Sequencer 1 priority: 2nd
// Sequencer 2 priority: 3rd
// Sequencer 3 priority: 4th (lowest)
// SS3 triggering event: software trigger
// SS3 1st sample source: Ain1 (PE2)
// SS3 interrupts: flag set on completion but no interrupt requested
void ADC0_InitSWTriggerSeq3_Ch1(void){ volatile unsigned long delay;
	//SYSCTL_RCGC0_R |= 0x00010000;   // 6) activate ADC0 (legacy code)
	SYSCTL_RCGCADC_R |= 0x00000001; // 1) activate ADC0
  SYSCTL_RCGC2_R |= 0x00000010;   // 1) activate clock for Port E
  delay = SYSCTL_RCGC2_R;         //    allow time for clock to stabilize
	delay = SYSCTL_RCGC2_R;
	delay = SYSCTL_RCGC2_R;
	delay = SYSCTL_RCGC2_R;
	
  GPIO_PORTE_DIR_R &= ~0x36;      // 2) make PE1, PE2, PE4, PE5 input
  GPIO_PORTE_AFSEL_R |= 0x36;     // 3) enable alternate function on PE1, PE2, PE4, PE5
  GPIO_PORTE_DEN_R &= ~0x36;      // 4) disable digital I/O on PE1, PE2, PE4, PE5
  GPIO_PORTE_AMSEL_R |= 0x36;     // 5) enable analog function on PE1, PE2, PE4, PE5
	ADC0_PC_R &= ~0xF;              // 8) clear max sample rate field
  ADC0_PC_R |= 0x1;               //    configure for 125K samples/sec
	
  ADC0_SSPRI_R = 0x3210;          // 8) Sequencer 3 is lowest priority
  ADC0_ACTSS_R &= ~0x0004;        // 9) disable sample sequencer 2
  ADC0_EMUX_R &= ~0x0F00;         // 10) seq2 is software trigger
	ADC0_SSMUX2_R = 0x8912;         // 12) set channels for SS2
  ADC0_SSCTL2_R = 0x0600;         // 12) no TS0 D0, yes IE0 END0
	
	ADC0_IM_R &= ~0x0004;						// disable SS2 interrupts
  ADC0_ACTSS_R |= 0x0004;         // 13) enable sample sequencer 2
}






//------------ADC0_InSeq3------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversion

void ADC_In298(unsigned long *ain2, unsigned long *ain1, unsigned long *ain9){
  ADC0_PSSI_R = 0x0004;            // 1) initiate SS2
  while((ADC0_RIS_R&0x04)==0){};   // 2) wait for conversion done
  *ain2 = ADC0_SSFIFO2_R&0xFFF;    // 3A) read first result
	*ain1 = ADC0_SSFIFO2_R&0xFFF;
  *ain9 = ADC0_SSFIFO2_R&0xFFF;    // 3B) read second result
//*ain8 = ADC0_SSFIFO2_R&0xFFF;    // 3C) read third result
  ADC0_ISC_R = 0x0004;             // 4) acknowledge completion
}


