#define F_CPU 7372800UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

unsigned char ReceivedByte;
/*void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}*/

//Function to initialize ports
/*void port_init()
{
 buzzer_pin_config();
}*/

//UART0 initialisation
// desired baud rate: 9600
// actual: baud rate:9600 (0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 
 UCSRB = (1 << RXEN) | (1 << TXEN) | (1<<RXCIE);   // Turn on the transmission and reception circuitry
 UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes
 
 UBRRL = 0x2F;
 UBRRH =0x00;
 }
void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) )
	;
	/* Put data into buffer, sends the data */
	UDR = data;
}


/*void buzzer_on (void)
{
	PORTC = PORTC | 0b00001000;
}
void buzzer_off (void)
{
	PORTC = PORTC & 0b11110111;
}
*/
/*void init_devices (void)
{
 cli();         //Clears the global interrupts
 port_init();
 uart0_init();
 sei();         //Enables the global interrupts
}*/
unsigned char usart_rx()
{
	while(!(UCSRA & (1<<RXC)));              //wait until reception is completed
	
	return UDR;                                     //receive first character by polling
}

ISR(USART_RXC_vect) 
{
  ReceivedByte = UDR; // Fetch the received byte value into the variable "ByteReceived"
  if (ReceivedByte=='a')
 {
	 buzzer_on();
	 
 }
 else if (ReceivedByte=='b')
 {
	 
	 buzzer_off();
 }
}


//Main Function
/*int main()
{
	init_devices();
	unsigned char temp=usart_rx();
	if (temp=='a')
	{
		buzzer_on();
		
	}
	else if (temp=='b')
	{
		
		buzzer_off();
	}
	temp=usart_rx();
	if (ReceivedByte=='a')
	{
		buzzer_on();
		
	}
	
	else if (ReceivedByte=='b')
	{
		
		buzzer_off();
	}
	while(1){
		// USART_Transmit('a');
		 // _delay_ms(1000);
	}
}
*/
