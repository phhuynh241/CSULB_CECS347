// ADCTestMain.c
// Runs on LM4F120/TM4C123
// This program periodically samples ADC channel 1 and stores the
// result to a global variable that can be accessed with the JTAG
// debugger and viewed with the variable watch feature.
// Peter Huynh

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

// input signal connected to PE2/AIN1

#include "ADCSWTrigger.h"
#include "PLL.h"
#include "Nokia5110.h"

//Port C initialization
#define GPIO_PORTC_DATA_R				(*((volatile unsigned long *)0x400063FC))
#define GPIO_PORTC_DIR_R        (*((volatile unsigned long *)0x40006400))
#define GPIO_PORTC_AFSEL_R      (*((volatile unsigned long *)0x40006420))
#define GPIO_PORTC_DEN_R        (*((volatile unsigned long *)0x4000651C))
#define GPIO_PORTC_AMSEL_R      (*((volatile unsigned long *)0x40006528))
#define GPIO_PORTC_PCTL_R       (*((volatile unsigned long *)0x4000652C))
#define GPIO_PORTC_DR8R_R       (*((volatile unsigned long *)0x40006508))

//#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
	
//Port F initialization
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
	
//Port B initialization
#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_PUR_R        (*((volatile unsigned long *)0x40005510))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
	
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
	
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))
	
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

unsigned long H, L, In1, In2, In3, In4, In5, In6, In7, In8;
unsigned long lowPWM, highPWM;
unsigned char flag_start, flag_stop;

unsigned long percent;

double lcdvalue, frontDistance, rightDistance, leftDistance;

unsigned long ADCvalue,frontSensor, leftSensor, rightSensor;

void Motor_Init(void){
	unsigned volatile long delay;
  SYSCTL_RCGC2_R |= 0x00000004; // activate clock for port C
	H = L = 1000;                // 30%
	delay = SYSCTL_RCGC2_R;
  GPIO_PORTC_AMSEL_R &= ~0xF0;      // disable analog functionality on PA7-4
  GPIO_PORTC_PCTL_R &= ~0xFFFF0000; // configure PA7-4 as GPIO
  GPIO_PORTC_DIR_R |= 0xF0;     // make PA7-4 out
  GPIO_PORTC_DR8R_R |= 0xF0;    // enable 8 mA drive on PA7-4
  GPIO_PORTC_AFSEL_R &= ~0xF0;  // disable alt funct on PA7-4
  GPIO_PORTC_DEN_R |= 0xF0;     // enable digital I/O on PA7-4
  GPIO_PORTC_DATA_R &= ~0xF0;   // make PA7-4 low
	
  NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = L-1;       // reload value for 500us
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
  NVIC_ST_CTRL_R = 0x00000007;  // enable with core clock and interrupts
}

void SysTick_Handler(void){
  if(GPIO_PORTC_DATA_R&0xA0){   // toggle PA7 and PA5
    GPIO_PORTC_DATA_R &= ~0xA0; // make PA7 and PA5 low
    NVIC_ST_RELOAD_R = L-1;     // reload value for low phase
		lowPWM = 1;									//flag for low PWM phase
		
  } else{
    GPIO_PORTC_DATA_R |= 0xA0;  // make PA7 and PA5 high
    NVIC_ST_RELOAD_R = H-1;     // reload value for high phase
		highPWM = 1;								// flag for high PWM phase
  }
}

void Switch_Init(void){  unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;         // allow changes to PF4-0
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4-0 in (built-in button)
	GPIO_PORTF_DIR_R |= 0x0E; 		//make PF3-1 outputs
  GPIO_PORTF_AFSEL_R &= ~0x1F;  //     disable alt funct on PF4-0
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4-0
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; //  configure PF4-0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x1F;  //     disable analog functionality on PF4-0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4-0
	
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}


void GPIOPortF_Handler(void){ // called on touch of either SW1 or SW2
	if(GPIO_PORTF_RIS_R&0x01){  // SW2 touch
    GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
		flag_stop = 1;						//flag for SW2
  }
	else if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
		flag_start = 1;						//flag for SW1
	}
}

int main(void){
	unsigned long volatile delay;
  PLL_Init();           				 // bus clock at 80 MHz
	Nokia5110_Init();
	Nokia5110_Clear();
	Nokia5110_OutString("*********************SPD:***PWM%:----   ----- ");
	ADC0_InitSWTriggerSeq3_Ch1();	
  Motor_Init();        					 // output from PA5, SysTick interrupts
  Switch_Init();        				 // arm PF4, PF0 for falling edge interrupts	
	// New logic for potentiometer instead dipswitch
	
  while(1){		
		GPIO_PORTC_DATA_R |= 0x50; 						//H-bridge 1100|0000
		NVIC_ST_CTRL_R = 0x00000007; 					//Turns on SysTick timer		
		GPIO_PORTF_DATA_R = 0x0E; 						//sets LED white (GO)
								//PE1						//PE2					//PE4
		ADC_In298(&rightSensor, &ADCvalue, &leftSensor);
		
		if (lowPWM){
			lowPWM = 0;
			H = (80000 * ADCvalue/4095)-1; 			 //calculate value to be reloaded in the high phase
			if (ADCvalue <= 1228){
				GPIO_PORTF_DATA_R = 0x02; 				 //sets LED red (stop)
				H = 24000;
			}
			if (ADCvalue >= 3685){
				H = 72000;
			}
		}
		
		//Calculates distance and PWM/Speed value
		leftDistance  = ((double)((31327/leftSensor)));
		rightDistance = ((double)((32883/rightSensor)));
		frontDistance = ((double)((40950/frontSensor)));
		lcdvalue = ((double) H/80000)*100;
			
		
		Nokia5110_Clear();
		Nokia5110_OutString("SPEED%|DIST:----   -----");
		//Displays PWM/Speed 
		Nokia5110_SetCursor(0, 2);
		Nokia5110_OutUDec((int)lcdvalue);
		
		//Displays right sensor distance
		Nokia5110_SetCursor(0,3);
		Nokia5110_OutString("RS");
		Nokia5110_SetCursor(6,3);
		
		if ((int)rightDistance >= 10 && (int)rightDistance <= 80){
			Nokia5110_OutUDec((int)rightDistance);
			Nokia5110_OutString("cm");
		}
		else{
			Nokia5110_OutString(" OOR");
		}
		
		//Displays left sensor distance
		Nokia5110_SetCursor(0,4);
		Nokia5110_OutString("LS");
		Nokia5110_SetCursor(6,4);
		
		if ((int)leftDistance >= 10 && (int)leftDistance <= 80){
			Nokia5110_OutUDec((int)leftDistance);
			Nokia5110_OutString("cm");
		}
		else{
			Nokia5110_OutString(" OOR");
		}

		L = (80000 - H);
		for(delay=0; delay<100000; delay++){};	
  }//end while loop
}//end main loop

