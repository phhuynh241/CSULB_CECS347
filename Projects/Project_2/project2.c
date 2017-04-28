// DCMotor.c
// Runs on LM4F120 or TM4C123
// Use SysTick interrupts to implement a software PWM to drive
// a DC motor at a given duty cycle.  The built-in button SW1
// increases the speed, and SW2 decreases the speed.
// Daniel Valvano, Jonathan Valvano
// August 6, 2013

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
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

// PA5 connected to DC motor interface

#include "PLL.h"

//Port A initialization
#define GPIO_PORTA_DATA_R       (*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DR8R_R       (*((volatile unsigned long *)0x40004508))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
	
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


// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

unsigned long H, L, In1, In2, In3, In4, In5, In6, In7, In8;
unsigned long lowPWM, highPWM;
unsigned char flag_start, flag_stop;

void Motor_Init(void){
	unsigned volatile long delay;
  SYSCTL_RCGC2_R |= 0x00000001; // activate clock for port A
	H = L = 1000;                // 30%
	delay = SYSCTL_RCGC2_R;
  GPIO_PORTA_AMSEL_R &= ~0xF0;      // disable analog functionality on PA7-4
  GPIO_PORTA_PCTL_R &= ~0xFFFF0000; // configure PA7-4 as GPIO
  GPIO_PORTA_DIR_R |= 0xF0;     // make PA7-4 out
  GPIO_PORTA_DR8R_R |= 0xF0;    // enable 8 mA drive on PA7-4
  GPIO_PORTA_AFSEL_R &= ~0xF0;  // disable alt funct on PA7-4
  GPIO_PORTA_DEN_R |= 0xF0;     // enable digital I/O on PA7-4
  GPIO_PORTA_DATA_R &= ~0xF0;   // make PA7-4 low
	
  NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = L-1;       // reload value for 500us
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
  NVIC_ST_CTRL_R = 0x00000007;  // enable with core clock and interrupts
}

void SysTick_Handler(void){
  if(GPIO_PORTA_DATA_R&0xA0){   // toggle PA7 and PA5
    GPIO_PORTA_DATA_R &= ~0xA0; // make PA7 and PA5 low
    NVIC_ST_RELOAD_R = L-1;     // reload value for low phase
		lowPWM = 1;									//flag for low PWM phase
		
  } else{
    GPIO_PORTA_DATA_R |= 0xA0;  // make PA7 and PA5 high
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

void DipSwitch_Init(void){
	unsigned long volatile delay;
	SYSCTL_RCGC2_R |= 0x00000002; //activate clock for Port B
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTB_CR_R = 0xFF;		//allow changes to PB7-0
	GPIO_PORTB_AMSEL_R = 0x00;//disable analog functions
	GPIO_PORTB_PCTL_R = 0x00000000;
	GPIO_PORTB_DIR_R |= 0x00;
	GPIO_PORTB_AFSEL_R |= 0x00;
	GPIO_PORTB_PUR_R |= 0xFF;
	GPIO_PORTB_DEN_R |= 0xFF;
}

// L range: 8000,16000,24000,32000,40000,48000,56000,64000,72000
// power:   10%    20%  30%   40%   50%   60%   70%   80%   90%
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
  DisableInterrupts();  // disable interrupts while initializing
  PLL_Init();           // bus clock at 80 MHz
  Motor_Init();         // output from PA5, SysTick interrupts
  Switch_Init();        // arm PF4, PF0 for falling edge interrupts
  EnableInterrupts();   // enable after all initialization are done
	DipSwitch_Init();
	GPIO_PORTF_DATA_R = 0x02;
  while(1){		
		//In1-4 controls direction
		In1 = GPIO_PORTB_DATA_R&0x01;//forward
		In2 = GPIO_PORTB_DATA_R&0x02;//backward
		In3 = GPIO_PORTB_DATA_R&0x04;//right
		In4 = GPIO_PORTB_DATA_R&0x08;//left
		//In5-8 controls speed
		In5 = GPIO_PORTB_DATA_R&0x10;//30%
		In6 = GPIO_PORTB_DATA_R&0x20;//50%
		In7 = GPIO_PORTB_DATA_R&0x40;//70%
		In8 = GPIO_PORTB_DATA_R&0x80;//90%
			
		if (flag_start==1){
			flag_start = 0; 
			if (In1 && (In5 || In6 || In7 || In8)){ //FORWARD
				if(lowPWM)
				{
					lowPWM = 0;
					GPIO_PORTA_DATA_R |= 0x50;
					NVIC_ST_CTRL_R = 0x00000007;
					
					if(In5){
						L = 56000; // 30%
						GPIO_PORTF_DATA_R = 0x0E; //WHITE
					}else if(In6){ 
						L = 40000; // 50%
						GPIO_PORTF_DATA_R = 0x0E;
					}else if(In7){
						L = 24000; // 70%
						GPIO_PORTF_DATA_R = 0x0E;
					}else if(In8){
						L = 8000;  // 90%
						GPIO_PORTF_DATA_R = 0x0E;
					}else{
						GPIO_PORTF_DATA_R = 0x02;
						NVIC_ST_CTRL_R = 0;
					}
				}
			}else if (In2 && (In5 || In6 || In7 || In8)){ //REVERSE
				if(highPWM)
				{
					highPWM = 0;
					GPIO_PORTA_DATA_R &= ~0x50;
					NVIC_ST_CTRL_R = 0x00000007;
					
					if(In5){
						L = 24000; // 30%
						GPIO_PORTF_DATA_R = 0x04; //BLUE
					}else if(In6){ 
						L = 40000; // 50%
						GPIO_PORTF_DATA_R = 0x04;
					}else if(In7){
						L = 56000; // 70%
						GPIO_PORTF_DATA_R = 0x04;
					}else if(In8){
						L = 72000;  // 90%
						GPIO_PORTF_DATA_R = 0x04;
					}else {
						NVIC_ST_CTRL_R = 0;
						GPIO_PORTF_DATA_R = 0x02;
					}
				}
			}else if (In3 && (In5 || In6 || In7 || In8)){ //RIGHT
				if(lowPWM)
				{
					lowPWM = 0;
					GPIO_PORTA_DATA_R &= ~0x10;
					GPIO_PORTA_DATA_R |= 0x40;
					NVIC_ST_CTRL_R = 0x00000007;
					
					if(In5){
						L = 24000; // 30%
						GPIO_PORTF_DATA_R = 0x08; //GREEN
					}else if(In6){ 
						L = 40000; // 50%
						GPIO_PORTF_DATA_R = 0x08;
					}else if(In7){
						L = 56000; // 70%
						GPIO_PORTF_DATA_R = 0x08;
					}else if(In8){
						L = 72000;  // 90%
						GPIO_PORTF_DATA_R = 0x08;
					}else {
						GPIO_PORTF_DATA_R = 0x02;
						NVIC_ST_CTRL_R = 0;
					}
				}
			}else if (In4 && (In5 || In6 || In7 || In8)){ //LEFT
				if (lowPWM){
					lowPWM = 0;
					GPIO_PORTA_DATA_R &= ~0x40;
					GPIO_PORTA_DATA_R |= 0x10;
					NVIC_ST_CTRL_R = 0x00000007;
					
					if(In5){
						L = 56000; // 30%
						GPIO_PORTF_DATA_R = 0x06; //PINK
					}else if(In6){ 
						L = 40000; // 50%
						GPIO_PORTF_DATA_R = 0x06;
					}else if(In7){
						L = 24000; // 70%
						GPIO_PORTF_DATA_R = 0x06;
					}else if(In8){
						L = 8000;  // 90%
						GPIO_PORTF_DATA_R = 0x06;
					}else {
						GPIO_PORTF_DATA_R = 0x02;
						NVIC_ST_CTRL_R = 0;
					}
				}
			}else{
				GPIO_PORTF_DATA_R = 0x02;
				NVIC_ST_CTRL_R = 0;
			}
		}
		
		if (flag_stop==1){
			flag_stop = 0;
			GPIO_PORTA_DATA_R = 0xF0;
			GPIO_PORTF_DATA_R = 0x02; //red
			NVIC_ST_CTRL_R = 0;
		}
		H = 80000 - L; //calculate value to be reloaded in the high phase
  }//end while loop
}//end main loop
