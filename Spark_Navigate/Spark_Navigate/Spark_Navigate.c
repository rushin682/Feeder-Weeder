/*
 * Initial_Navigation.c
 *
 * Created: 2/20/2018 10:03:49 PM
 *  Author: Eastways Travel
 */ 

//For Red Spark

#define F_CPU 7372800UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function

//#include "initializations.c"
//#include "movements_config.c"
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


int avg[3]={60,60,60}; //Used for thresholds
int th[3]={0,0,0}; //Used for digital values of line eg 010 when center is on line
int error=0,target=2,p=0,kp=40; //Constants for Proportional in p
int highcount=0,highcurrent=0; //Constants for Proportional in p
int baseLine=140; //Constants for Proportional in p

int plant_number = 3;
int toSend[3];
//Buzzer Configurations
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

//Buzzer Ends


//Line Follower 

void readSensor()
{
	highcurrent=0;
	highcount=0;
	sensor(0,th[0],avg[0]);
	sensor(1,th[1],avg[1]);
	sensor(2,th[2],avg[2]);
	
/*if(th[0] == 1){PORTC |= 0b00000001;}
	else if(th[0] == 0){PORTC &= 0b11111110;}
	if(th[1]==1){PORTC |= 0b00000010;}	
	else if(th[1]==0){PORTC &= 0b11111101;}
	if(th[2] == 1){PORTC |= 0b00000100;}
	else if(th[2] == 1){PORTC &= 0b11111011;}*/
	
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


//Line Follower Ends


void port_init()
{
	buzzer_pin_config();
	//lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
}


//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRB = DDRB | 0x0F;    //set direction of the PORTB3 to PORTB0 pins as output
	PORTB = PORTB & 0xF0;  //set initial value of the PORTB3 to PORTB0 pins to logic 0
	DDRD = DDRD | 0x30;    //Setting PD5 and PD4 pins as output for PWM generation
	PORTD = PORTD | 0x30;  //PD5 and PD4 pins are for velocity control using PWM
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

void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer1_init();
	uart0_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	
	uart0_init();
	sei();   //Enables the global interrupts
}



//Movement Configs
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

void forward_mm(unsigned int DistanceInMM, char purpose)
{
	
	linear_distance_mm(DistanceInMM, 1, purpose);
}

void back_mm(unsigned int DistanceInMM, char purpose)
{
	linear_distance_mm(DistanceInMM, -1, purpose);
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
		plant_trigger = plantDetect(0);
		if(plant_trigger == 1){
			velocity(0, 0);
			pendingShaftCount = ReqdShaftCountInt - ShaftCountRight;
			if(purpose == 'S'){ 	
				buzzer_on();	
				_delay_ms(1000);
				buzzer_off();
				forward();
				_delay_ms(2000);		
				//colourDetect();
				int color = 1, current_X, current_Y;
				if(color != 1){
					current_X = x_D - ((ShaftCountRight * 12.92)/340);
					current_Y = y_D;
					update(color, current_X, current_Y, face);
				}				
				//timer1_init();
			}
			else if(purpose == 'W'){
				buzzer_on();
				_delay_ms(3000);
				//servoAction();				
			} 
		}
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break; 
		}
	}
	stop(); //Stop robot
}


//Movement Ends

//Rotation Configs


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



//Rotation Ends



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
	
	forward_mm(vertical * 340, 'R');
	
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
	forward_mm(horizontal * 340, 'R');
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
		if(ReceivedByte == 'E'){
		transmit_findings(toSend);
		}		
	}
			
}

transmit(int grid_number){
	//char v = (device_id*100)+grid_number;
	USART_Transmit(grid_number);
}


void transmit_findings(int array[]){
	transmit(65);
	for (int i=0;i<sizeof(array);i++){
		transmit(array[i]);
	}
	transmit(69);
}



void update(int color, int current_X, char current_Y, char current_Direction){
	int r = 0, g = 1, b = 2;
	int block;
	if(current_Direction = 'N'){
		current_X--;
	}
	block = ((current_X - 1)*6)+((int)current_Y - 64);
	global_array[block - 1] = color;
	if(plant_number>0){		
		toSend[--plant_number] = (color * 100) + block;
	}
}

int main(void)
{	
	
	init_devices();		
	
	//Enter the co-ordinates of the initial positions of all the three bots.
	//Fx,Fy for Firebird | Sbx,Sby for Spark blue | Srx,Sry for Spark Red
		int x_F = 3;
		int y_F = 'C';
		char dir_F = 'E';
		
		//This function will set the nodes to travel in the vertical & horizontal direction for the bot to reach its standard spot
		manoeuvre(x_F, y_F, dir_F);	
		
		//int grid_number = (vertical-1)*7) + (horizontal-64);
		//transmit(grid_number);
		
		vertical_motion();
		stop();
		_delay_ms(1000);
		horizontal_motion();
		_delay_ms(1000);
		
		scan(STOPS);	
		stop();
		
		
		
		return 0;
}