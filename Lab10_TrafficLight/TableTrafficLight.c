// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

#define NVIC_ST_CTRL_R      		(*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    		(*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   		(*((volatile unsigned long *)0xE000E018))
	
#define StreetLight							(*((volatile unsigned long *)0x400050FC))
#define WalkLight								(*((volatile unsigned long *)0x40025028))
#define	Signal									(*((volatile unsigned long *)0x4002401C))
//state set up
typedef struct State{
	unsigned long Time;			//time delay
	unsigned long Traffic;	//Traffic Signal
	unsigned long Walk;			//Walk Signal
	unsigned long Next[8];
}TrafficLight;

	unsigned long cState; //initialize the current state

//More Port Initializations
void PortB_Init(void){ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000002;      // 1)Activate B clock
  delay = SYSCTL_RCGC2_R;            // delay to allow clock to stabilize 
  GPIO_PORTB_LOCK_R = 0x4C4F434B;   // unlock port
  GPIO_PORTB_CR_R = 0x3F;           // allow changes to PB5-0
	GPIO_PORTB_PCTL_R = 0x00000000;   // clear PCTL
  GPIO_PORTB_AMSEL_R &= ~0x3F;      // disable analog on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F;      // disable alt funct on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;         // enable digital I/O on PB5-0
	GPIO_PORTB_DIR_R |= 0x3F;         // PB5-0 outputs	
}

void PortE_Init(void){ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000010;      // 1)Activate E clock
  delay = SYSCTL_RCGC2_R;            // delay to allow clock to stabilize 
  GPIO_PORTE_LOCK_R = 0x4C4F434B;   // unlock port
  GPIO_PORTE_CR_R = 0x07;           // allow changes to PE2-0
	GPIO_PORTE_PCTL_R = 0x00000000;   // clear PCTL
  GPIO_PORTE_AMSEL_R &= ~0x07;      // disable analog on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07;      // disable alt funct on PE2-0
  GPIO_PORTE_PUR_R &= ~0x07;        // disableb pull-up on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;         // enable digital I/O on PE2-0
	GPIO_PORTE_DIR_R &= ~0x07;        // PE2-0 inputs
}

void PortF_Init(void){ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;      // 1)Activate F clock
  delay = SYSCTL_RCGC2_R;            // delay to allow clock to stabilize 
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock port
  GPIO_PORTF_CR_R = 0x0A;           // allow changes to PF1 & PF3
	GPIO_PORTF_PCTL_R = 0x00000000;   // clear PCTL
  GPIO_PORTF_AMSEL_R &= ~0x0A;      // disable analog on PF1 & PF3
  GPIO_PORTF_AFSEL_R &= ~0x0A;      // disable alt funct on PF1 & PF3
  GPIO_PORTF_DEN_R |= 0x0A;         // enable digital I/O on PF1 & PF3
	GPIO_PORTF_DIR_R |= 0x0A;         // PF1 & PF3 outputs
}
//Systick Functions
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 800000*12.5ns equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****
	TrafficLight FSM[11] = {
		{200, 0x21, 0x02, {0, 0, 1, 1, 1, 1, 1, 1}},
		{300, 0x22, 0x02, {2, 2, 2, 2, 4, 4, 2, 2}},
		{200, 0x0C, 0x02, {2, 3, 2, 3, 3, 3, 3, 3}},
		{300, 0x14, 0x02, {0, 0, 0, 0, 4, 0, 4, 4}},
		{200, 0x24, 0x08, {4, 5, 5, 5, 4, 5, 5, 5}},
		{50,  0x24, 0x00, {6, 6, 6, 6, 6, 6, 6, 6}},
		{50,  0x24, 0x02, {7, 7, 7, 7, 7, 7, 7, 7}},
		{50,  0x24, 0x00, {8, 8, 8, 8, 8, 8, 8, 8}},
		{50,  0x24, 0x02, {9, 9, 9, 9, 9, 9, 9, 9}},
		{50,  0x24, 0x00, {10, 10, 10, 10, 10, 10, 10, 10}},
		{50,  0x24, 0x02, {0, 0, 2, 0, 4, 0, 2, 0}}
	};


int main(void){ 
	TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	PortB_Init();
  PortE_Init();
	PortF_Init();
	SysTick_Init();
  
	//Traffic Light FSM
	cState = 0;

	EnableInterrupts();
  while(1){
		//Time Traffic Walk Next
		StreetLight = FSM[cState].Traffic;		//Traffic Light Color = The state the traffic bits are
		WalkLight = FSM[cState].Walk;					//Walk Light Color = The state the walk bits are
		SysTick_Wait10ms(FSM[cState].Time);		//Wait = initialize the systick wait with the amount set in the FSM
		cState = FSM[cState].Next[Signal];		//This sets cState by taking the value from the signal and using that to decide the next state
		
  }
}