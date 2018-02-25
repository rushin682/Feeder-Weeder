/*
 * Initial_Navigation.c
 *
 * Created: 2/20/2018 10:03:49 PM
 *  Author: Eastways Travel
 */ 

#define F_CPU 7372800UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function

//#include "buzzer.h"
#include "lcd.c"
#include "zigbee_comms.c"

#include "Line_Follow.c"
#include "POS.c"
#include "IR_Embed.c"

#define STOPS 7 

int device_id = 1;

int vertical = 0;
int horizontal = 0;
char direction, face;
int x_D = 0, y_D = 0;



/*unsigned char ADC_Value;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;*/

int avg[3]={60,60,60}; //Used for thresholds
int th[3]={0,0,0}; //Used for digital values of line eg 010 when center is on line
int error=0,target=2,p=0,kp=40; //Constants for Proportional in p
int highcount=0,highcurrent=0; //Constants for Proportional in p
int baseLine=140; //Constants for Proportional in p

void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

/*void buzzer_on (void)
{
	PORTC = PORTC | 0b00001000;
}
void buzzer_off (void)
{
	PORTC = PORTC & 0b11110111;
}*/

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRB = DDRB | 0x0F;    //set direction of the PORTB3 to PORTB0 pins as output
	PORTB = PORTB & 0xF0;  //set initial value of the PORTB3 to PORTB0 pins to logic 0
	DDRD = DDRD | 0x30;    //Setting PD5 and PD4 pins as output for PWM generation
	PORTD = PORTD | 0x30;  //PD5 and PD4 pins are for velocity control using PWM
}

void port_init()
{
	buzzer_pin_config();
	//lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortBRestore = 0;

	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortBRestore = PORTB; 			// reading the PORTB's original status
	PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
	PORTB = PortBRestore; 			// setting the command to the port
}


void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}


//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM, int sign, char purpose)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0, pendingShaftCount = 0;
	int plant_trigger=0;
	ReqdShaftCount = DistanceInMM / 12.92; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{		
		readSensor();
		proportional();	
		
		if(sign > 0){forward();}
		else if(sign < 0){ back();}	
			
		velocity(baseLine+p,baseLine-p);		
		//plant_trigger = plantDetect();
		/*if(plant_trigger == 1){
			stop();
			pendingShaftCount = ReqdShaftCountInt - ShaftCountRight;
			if(purpose == 'S'){ 				
				colourDetect();
				timer1_init();
			}
			else if(purpose == 'W'){
				stop();
				buzzer_on();
				_delay_ms(3000);
				//servoAction();				
			} 
		}*/
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break; 
		}
	}
	stop(); //Stop robot
}

void readSensor()
{
	highcurrent=0;
	highcount=0;
	sensor(0,th[0],avg[0]);
	sensor(1,th[1],avg[1]);
	sensor(2,th[2],avg[2]);
	
	if(th[0] == 1){PORTC |= 0b00000001;}
	else if(th[0] == 0){PORTC &= 0b11111110;}
	if(th[1]==1){PORTC |= 0b00000010;}	
	else if(th[1]==0){PORTC &= 0b11111101;}
	if(th[2] == 1){PORTC |= 0b00000100;}
	else if(th[2] == 1){PORTC &= 0b11111011;}
	
}
void sensor(int n,int sn,int avgn)
{
	
	sn=ADC_Conversion(n+3);
	if(sn>=avgn)
	{
		th[n]=1;
		highcount++;
		highcurrent+=(n+1);
	}
	else if(sn<avgn)
	{
		th[n]=0;
	}
	//lcd_print(1, 3-n, th[n], 1);
}
void proportional()
{
	error = ((double)highcurrent/highcount)-target;
	p=(kp*error);
	if(th[0]==0 && th[1]==0 && th[2]==0)
	{
		p=0;
	}
}

void forward_mm(unsigned int DistanceInMM, char purpose)
{
	
	linear_distance_mm(DistanceInMM, 1, purpose);
}

void back_mm(unsigned int DistanceInMM, char purpose)
{
	linear_distance_mm(DistanceInMM, -1, purpose);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}


void manoeuvre(int x_F, int y_F,char dir_F){
	int x_origin = 4;
	int y_origin = 'D';
	
	
	
	
	int x_middle = x_F - x_origin;
	int y_middle = y_F - y_origin;
	
	//For determining the final destination x-coordinate
	if( (x_middle) > 0 ){ x_D = 7; }
	else { x_D = 1; }
	int x_intermediate = x_D - x_origin;
	
	//For determining the final destination y-coordinate	
	if((y_middle) > 0){ y_D = 'F'; }	
	else if((y_middle) == 0){ y_D = 'D'; }
	else { y_D = 'B'; } 
	int y_intermediate = y_D - y_origin;	
	
	//How much to manoeuvre
	vertical = 	x_middle - x_intermediate;
	horizontal = y_middle - y_intermediate;
	direction = dir_F;	
}

/*void print_value()
{
	lcd_set_4bit();
	lcd_cursor(1, 3);
	lcd_string("Vert");	
	lcd_print(1,8, vertical, 3);
	lcd_cursor(2, 1);
	lcd_string("horiz");
	lcd_print(2,8, horizontal, 3);
}*/

void rotate(char face){
	switch(direction){
		case 'N':{
			if(face == 'S'){right_degrees(180);}
			else if(face == 'E'){right_degrees(90);}
			else if(face == 'W'){left_degrees(90);}
				break;
		}
		case 'W':{
			if(face == 'S'){left_degrees(90);}
			else if(face == 'E'){right_degrees(180);}
			else if(face == 'N'){right_degrees(90);}
				break;
		}
		case 'S':{
			if(face == 'N'){right_degrees(180);}
			else if(face == 'W'){right_degrees(90);}
			else if(face == 'E'){left_degrees(90);}
				break;
		}
		case 'E':{
			if(face == 'W'){right_degrees(180);}
			else if(face == 'S'){right_degrees(90);}
			else if(face == 'N'){left_degrees(90);}
				break;
		}		
	}
}

void vertical_motion(){
	
	if(vertical < 0){ 
		face = 'S';
	}
	else{
		face = 'N';
	}
	rotate(face);
	direction = face;	
	vertical = abs(vertical);
	//lcd_print(1,3, vertical, 3);
	DDRC|= 0b00000111;
	
	
	forward_mm(vertical * 340, 'S');
	
}
void horizontal_motion(){
	
	if(horizontal < 0){ 
		face = 'E';
	}
	else{
		face = 'W';
	}
	rotate(face);
	direction = face;
	horizontal = abs(horizontal);
	forward_mm(horizontal * 340, 'S');
}


void scan(int nodes){
	for(int i=0;i<2;i++){	
		if(x_D == 1){
			face = 'S';
			x_D = 7; 
		}
		else if(x_D == 7){
			face = 'N';
			x_D = 1;
		}
		rotate(face);
		direction = face;
		_delay_ms(1000);
		forward_mm(((nodes-1) * 340), 'S');
		_delay_ms(1000);
		
		
		//right_degrees(180);
	}
	
	
	/*buzzer_on();
	_delay_ms(1000);
	buzzer_off();*/
			
}

transmit(int grid_number){
	char v = (device_id*100)+grid_number;
	USART_Transmit(v);
}

void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer1_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	
	uart0_init();
	sei();   //Enables the global interrupts
}


int main(void)
{	
	
	//lcd_port_config();
	init_devices();
	//lcd_set_4bit();	
	//lcd_init();	
	
	
	/*PORTC|=0b00000111;
	_delay_ms(2000);
	PORTC&=0b11111000;
	forward();*/
	
	
	//Enter the co-ordinates of the initial positions of all the three bots.
	//Fx,Fy for Firebird | Sbx,Sby for Spark blue | Srx,Sry for Spark Red
		int x_F = 5;
		int y_F = 'D';
		char dir_F = 'N';
	
	
		
	
		/*buzzer_on();
		_delay_ms(1000);		//delay
		buzzer_off();*/
		
		
		//This function will set the nodes to travel in the vertical & horizontal direction for the bot to reach its standard spot
		manoeuvre(x_F, y_F, dir_F);	
		
		//print_value();
		//_delay_ms(5000);		
		//lcd_wr_command(0x01);
		//_delay_ms(1000);
		
		//int grid_number = (vertical-1)*7) + (horizontal-64);
		//transmit(grid_number);
		
		
		
		/*while(1){
			readSensor();			
			proportional();
			velocity(baseLine+p,baseLine-p);
		}*/			
				
		vertical_motion();
		stop();
		_delay_ms(1000);
		horizontal_motion();
		_delay_ms(1000);
		
		//right_degrees(180);
		//_delay_ms(500);
		
		
		scan(STOPS);	
		stop();
		
		
}