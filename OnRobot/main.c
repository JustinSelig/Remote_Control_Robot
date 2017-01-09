#include "MK64F12.h"

/*UART Helper Function*/
uint8_t uart_getchar ()
{
/* Wait until character has been received */
while (!(UART3_S1 & UART_S1_RDRF_MASK));
/* Return the 8-bit data from the receiver */
return UART3_D;
}

static int increase_speed;
static int decrease_speed;
static int bear_left;
static int bear_right;
static int forward;
static int backward;

int main(void){
	
	/*UART Init*/
	uint16_t ubd;					/*Variable to save the baud rate*/
	uint8_t temp;
	uint8_t ch;
	SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;      /*Enable the UART clock*/
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;		/*Enable the PORTB clock*/
	PORTC_PCR16 |= PORT_PCR_MUX(3);		//PORT C PIN 16
	PORTC_PCR17 |= PORT_PCR_MUX(3);   //PORT C PIN 17
	UART3_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );  /*Disable Tx and Rx*/
	UART3_C1 = 0; 		/*Dafault settings of the register*/
	ubd = (uint16_t)((21000*1000)/(9600 * 16));  /* Calculate baud settings -- baud rate=9600*/
	temp = UART3_BDH & ~(UART_BDH_SBR(0x1F));   /*Save the value of UART3_BDH except SBR*/
	UART3_BDH = temp | (((ubd & 0x1F00) >> 8));
	UART3_BDL = (uint8_t)(ubd & UART_BDL_SBR_MASK);
	UART3_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );    /* Enable receiver and transmitter */
	
  SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; /*ENABLE PORTD CLOCK: PTD5*/
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //added for ptc4?
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK; /*ENABLE FTM0 CLOCK*/
	PORTD_PCR5 = PORT_PCR_MUX(4);	/*MUX = ALT 4*/
	NVIC_EnableIRQ(FTM0_IRQn); /*enable the ftm interrupt*/
	FTM0_SC |= 0x004F; /*Status and control: Setting TOIE = 1, CLKS = 01, PS = 111*/
	FTM0_MOD = 32000; /*Setting modulo register = 32000 */
	FTM0_C5SC |= 0x0028; /*Channel 5 Status and control: Setting MSB=1, ELSnB=1 -- edge aligned pwm*/
	FTM0_C5V = 500; /*Value of the channel 5*/

	PORTD_PCR7 = PORT_PCR_MUX(4);	/*MUX = ALT 4 -- ftm0_ch7*/
	FTM0_C7SC |= 0x0028; /*Setting MSB=1, ELSnB=1*///ftm3ch5?
	FTM0_C7V = 500; /*Value of the channel*/

	/*GPIO Setup: port d, bit 0,1,2*/
	PORTD_PCR0 = PORT_PCR_MUX(1); //=0x100; multiplexer alternative 1
	PORTD_PCR1 = PORT_PCR_MUX(1); //PTD1
	PORTD_PCR2 = PORT_PCR_MUX(1); //PTD2
	PORTD_PCR3 = PORT_PCR_MUX(1); //PTD3
	PORTC_PCR4 = PORT_PCR_MUX(1); //PTC4

	GPIOD_PDDR |= 1; //(1 << 0);		/*Setting the bit 0 of the port D as Output*/
	GPIOD_PDDR |= (1 << 1);		/*Setting the bit 1 of the port D as Output*/
	GPIOD_PDDR |= (1 << 2);		/*Setting the bit 2 of the port D as Output*/
	GPIOD_PDDR |= (1 << 3);		/*Setting the bit 3 of the port D as Output*/
	GPIOC_PDDR |= (1 << 4);		/*Setting the bit 4 of the port C as Output*/
	
	//Standby: PTD0
	GPIOD_PDOR |= 1; //(1 << 0); //high - most of the time
	//BIN1: PTD2
	GPIOD_PDOR &= ~(1 << 2); //low
	//BIN2: PTD1
	GPIOD_PDOR |= (1 << 1); //high
	//AIN1: PTD3
	GPIOD_PDOR &= ~(1 << 3); //low
	//AIN2: PTC4
	GPIOC_PDOR |= (1 << 4); //high

	while(1){

		ch = uart_getchar();
		if (ch == 'l'){	//left
			//AIN1: PTD3
			GPIOD_PDOR &= ~(1 << 3); //low
			//AIN2: PTC4
			GPIOC_PDOR |= (1 << 4); //high
			//BIN1: PTD2
			GPIOD_PDOR |= (1 << 2); //high
			//BIN2: PTD1
			GPIOD_PDOR &= ~(1 << 1); //low
		}
		else if (ch == 'r'){	//right
			//AIN1: PTD3
			GPIOD_PDOR |= (1 << 3); //high
			//AIN2: PTC4
			GPIOC_PDOR &= ~(1 << 4); //low
			//BIN1: PTD2
			GPIOD_PDOR &= ~(1 << 2); //low
			//BIN2: PTD1
			GPIOD_PDOR |= (1 << 1); //high
		}
		else if (ch == 'f'){	//forwards
			forward = 1;
			//AIN1: PTD3
			GPIOD_PDOR |= (1 << 3); //low
			//AIN2: PTC4
			GPIOC_PDOR &= ~(1 << 4); //high
			//BIN1: PTD2
			GPIOD_PDOR |= (1 << 2); //high
			//BIN2: PTD1
			GPIOD_PDOR &= ~(1 << 1); //low
		}
		else if (ch == 'b'){	//backwards
			backward = 1;
			//AIN1: PTD3
			GPIOD_PDOR &= ~(1 << 3); //low
			//AIN2: PTC4
			GPIOC_PDOR |= (1 << 4); //high
			//BIN1: PTD2
			GPIOD_PDOR &= ~(1 << 2); //low
			//BIN2: PTD1
			GPIOD_PDOR |= (1 << 1); //high
		}
		else if (ch == 'z'){ //bear left
			bear_left = 1;
		}
		else if (ch == 'e'){	//bear right
			bear_right = 1;
		}
		else if (ch == 'i'){	//increase speed
			increase_speed = 1;
		}
		else if (ch == 'd'){	//decrease speed
			decrease_speed = 1;
		}
	//	else { //standby
	//		GPIOD_PDOR &= ~(1 << 0); 
	//	}
		
	}
}

void FTM0_IRQHandler(void){
	unsigned long Channel5Value = FTM0_C5V; //Left Motor: Take the value of the channel to compare it
	unsigned long Channel7Value = FTM0_C7V; //Right motor: Take the value of the channel to compare it
	(void)FTM0_SC;
	
	if ((Channel5Value < 32000) && (Channel7Value < 32000)){
		if (increase_speed){
			FTM0_C5V += 500;
			FTM0_C7V += 500;
			increase_speed = 0;
		}
		if (decrease_speed){
			FTM0_C5V -= 500;
			FTM0_C7V -= 500;
			decrease_speed = 0;
		}
		if (bear_left){
			FTM0_C5V = 2*FTM0_C7V;
			bear_left = 0;
		}
		else if (bear_right){
			FTM0_C7V = 2*FTM0_C5V;
			bear_right = 0;
		}
		
		if (forward || backward){
			FTM0_C7V = FTM0_C5V;
			forward = 0;
			backward = 0;
		}
	}
//	else {	//values too high
//		FTM0_C5V = 0;
//		FTM0_C7V = 0;
//	}
	
	//this works, but doesn't allow more of this interrupt:
	//FTM0_SC = FTM_SC_TOIE_MASK; //| FTM_SC_CLKS(FTM_SC_CLKS_FIXED_FREQUENCY_CLOCK) | FTM_SC_PS(FTM_SC_PS_DIVIDE_BY_128);
	
	//clock divide by 128, system clock, up count, enable timer overflow interrupts (toie), indicate timer overflowed -- ...01001111
	FTM0_SC = 0x004F;
}
