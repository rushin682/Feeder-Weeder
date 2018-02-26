#define F_CPU 7372800UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

unsigned char ReceivedByte;
int triggered=0, update_received=0;
int global_array[36];

//UART0 initialisation
// desired baud rate: 9600
// actual: baud rate:9600 (0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	
	UCSRB = (1 << RXEN) | (1 << TXEN);//|(1 << TXCIE);   // Turn on the transmission and reception circuitry
	UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes
	UBRRL = 0x2F;
	UBRRH =0x00;
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) );
	/* Put data into buffer, sends the data */
	UDR = data;
}

unsigned char usart_rx()
{
	while(!(UCSRA & (1<<RXC)));              //wait until reception is completed
	
	return UDR;                                     //receive first character by polling
}

ISR(USART_RXC_vect)
{
	ReceivedByte = UDR; // Fetch the received byte value into the variable "ByteReceived"
	if ((ReceivedByte=='A')||(ReceivedByte=='C')){
		triggered = 1;			
	}
	if (triggered == 1 && ReceivedByte != 'E' && ReceivedByte != 'A' && ReceivedByte != 'C' && ReceivedByte != 'F'){
		int ascii = ReceivedByte;
		global_array[(ascii % 100)] = (ascii / 100);
	}
	
}