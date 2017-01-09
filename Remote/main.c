#include "MK64F12.h"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

uint8_t uart_getchar ();
void uart_putchar (char ch);

int main(){
	/*UART Init*/
	uint16_t ubd;					/*Variable to save the baud rate*/
	uint8_t temp;
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
	
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD_PCR0 = 0x90100; 		/*PORTC_PCR6: ISF=0,IRQC=9,MUX=1 */
	PORTD_PCR1 = 0x90100;
	PORTD_PCR2 = 0x90100;
	PORTD_PCR3 = 0x90100;
	
	GPIOD_PDDR &= ~(1 << 0);		/*Setting the bit 0 of the port D as input*/
	GPIOD_PDDR &= ~(1 << 1);		/*Setting the bit 1 of the port D as input*/
	GPIOD_PDDR &= ~(1 << 2);		/*Setting the bit 2 of the port D as input*/
	GPIOD_PDDR &= ~(1 << 3);		/*Setting the bit 3 of the port D as input*/
	
	PORTD_ISFR = PORT_ISFR_ISF(0x40); 	  /* Clear interrupt status flag */
	NVIC_EnableIRQ(PORTD_IRQn);			/*Enable the PORTD interrupt*/
	
	while(1);
}

void PORTD_IRQHandler(void)
{
	if ((CHECK_BIT(GPIOD_PDIR, 0) && CHECK_BIT(GPIOD_PDIR, 1)) ||
		(CHECK_BIT(GPIOD_PDIR, 2) && CHECK_BIT(GPIOD_PDIR, 3))){
		uart_putchar('e');
		uart_putchar('\r');
		uart_putchar('\n');
	}
	else if ((CHECK_BIT(GPIOD_PDIR, 0) && CHECK_BIT(GPIOD_PDIR, 2)) ||
		(CHECK_BIT(GPIOD_PDIR, 1) && CHECK_BIT(GPIOD_PDIR, 3))){
		uart_putchar('z');
		uart_putchar('\r');
		uart_putchar('\n');
	}
	else if (CHECK_BIT(GPIOD_PDIR, 1) && CHECK_BIT(GPIOD_PDIR, 2)){
		uart_putchar('i');
		uart_putchar('\r');
		uart_putchar('\n');
	}
	else if (CHECK_BIT(GPIOD_PDIR, 0) && CHECK_BIT(GPIOD_PDIR, 3)){
		uart_putchar('d');
		uart_putchar('\r');
		uart_putchar('\n');
	}
	else if (CHECK_BIT(GPIOD_PDIR, 0)){ //zeroth bit is set-button pressed
		uart_putchar('f');
		uart_putchar('\r');
		uart_putchar('\n');
	}
	else if (CHECK_BIT(GPIOD_PDIR, 1)){
		uart_putchar('r');
		uart_putchar('\r');
		uart_putchar('\n');
	}
	else if (CHECK_BIT(GPIOD_PDIR, 2)){
		uart_putchar('l');
		uart_putchar('\r');
		uart_putchar('\n');
	}
	else if (CHECK_BIT(GPIOD_PDIR, 3)){
		uart_putchar('b');
		uart_putchar('\r');
		uart_putchar('\n');
	}

	PORTD_ISFR = PORT_ISFR_ISF(0x40);		/* Clear interrupt status flag */
}

/*UART Helper Functions*/
void uart_putchar (char ch)
{
/* Wait until space is available in the FIFO */
while(!(UART3_S1 & UART_S1_TDRE_MASK));
/* Send the character */
UART3_D = (uint8_t)ch;
}