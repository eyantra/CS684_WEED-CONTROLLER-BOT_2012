#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include "lcd.c"

unsigned char ADC_Value, ADC_Valuep;

volatile int shaftcountright = 0;
volatile int shaftcountleft = 0;

unsigned char flag = 0;

unsigned char Left_white_line ;
unsigned char Center_white_line ;
unsigned char Right_white_line ;

unsigned char data, data1;
int trou;

ISR(INT5_vect)
{
    shaftcountright++;
}

ISR(INT4_vect)
{
    shaftcountleft++;
}

void init_l(void)
{
    DDRE = DDRE & 0xEF;
    PORTE = PORTE | 0x10;
}

void init_r(void)
{
    DDRE = DDRE & 0xDF;
    PORTE = PORTE | 0x20;
}

void left_position_encoder_interrupt_init(void)
{
    cli();
    EICRB = EICRB | 0x02;
    EIMSK = EIMSK | 0x10;
    sei();
}

void right_position_encoder_interrupt_init(void)
{
    cli();
    EICRB = EICRB | 0x08;
    EIMSK = EIMSK | 0x20;
    sei();
}

void velocity(unsigned char left_motor, unsigned char right_motor)
{
    OCR5AL = (unsigned char) left_motor;
    OCR5BL = (unsigned char) right_motor;
}

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
    UCSR0B = 0x00; //disable while setting baud rate
    UCSR0A = 0x00;
    UCSR0C = 0x06;
//    UBRR0L = 0x47; //11059200 Hz
    UBRR0L = 0x5F; // 14745600 Hz set baud rate low
    UBRR0H = 0x00; //set baud rate hi
    UCSR0B = 0x98;
}

void motion_init(void)
{
    DDRA = 0x0F;
    PORTA = 0x00;
//    PORTA = PORTA & 0xF0;
    DDRL = 0x18;
    PORTL = 0x18;
}

void stop(void)
{
    PORTA = 0x00;
}

void forward(void)
{
    PORTA = 0x06;
}

void backward(void)
{
    PORTA = 0x09;
}

void right(void)
{
    PORTA = 0x0A;
}

void left(void)
{
    PORTA = 0x05;
}

void distance(unsigned int dis)
{
    float reqshaftcount = 0;
    unsigned long int reqshaftcountint = 0;
    reqshaftcount = dis / 5.338;
    reqshaftcountint = (unsigned long int) reqshaftcount;
    shaftcountright = 0;
    
    while(1)
    {
        if(shaftcountright > reqshaftcountint)
        {
            break;
        }
    }
    
    stop();
}

void angle_rotate(unsigned int Degrees) 
{ 
    float ReqdShaftCount = 0; 
    unsigned long int ReqdShaftCountInt = 0; 
    ReqdShaftCount = (float) Degrees / 4.090; // division by resolution to get shaft count 
    ReqdShaftCountInt = (unsigned int) ReqdShaftCount; 
    shaftcountright = 0;  
    shaftcountleft = 0;  
    
    while (1) 
    { 
        if((shaftcountright >= ReqdShaftCountInt) | (shaftcountleft >= ReqdShaftCountInt))
            break; 
    } 
    
    stop(); //Stop robot 
}

void lcd_port_config(void)
{
    DDRC = DDRC | 0xF7;
    PORTC = PORTC & 0X80;
}

void adc_pin_config (void) 
{
    DDRF = 0x00; //set PORTF direction as input 
    PORTF = 0x00; //set PORTF pins floating 
    DDRK = 0x00; //set PORTK direction as input 
    PORTK = 0x00; //set PORTK pins floating 
}

// Function to Initialize ADC 
void adc_init() 
{ 
    ADCSRA = 0x00; 
    ADCSRB = 0x00;  //MUX5 = 0 
    ADMUX = 0x20;   //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000 
    ACSR = 0x80; 
    ADCSRA = 0x86;  //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0 
}

void init_devices (void) 
{ 
    cli(); //Clears the global interrupts 
    adc_init(); 
    sei(); //Enables the global interrupts 
} 

//This Function accepts the Channel Number and returns the corresponding Analog Value  
unsigned char ADC_Conversion(unsigned char Ch) 
{ 
    unsigned char a; 

    if(Ch>7) 
    { 
        ADCSRB = 0x08; 
    }

    Ch = Ch & 0x07;      
    ADMUX = 0x20 | Ch;        
    ADCSRA = ADCSRA | 0x40;  //Set start conversion bit 

    while((ADCSRA & 0x10) == 0);  //Wait for ADC conversion to complete 

    a = ADCH;
    ADCSRA = ADCSRA|0x10;   //clear ADIF (ADC Interrupt Flag) by writing 1 to it 
    ADCSRB = 0x00; 

    return a; 
}

// This function moves the bot on white line by given distance(in mm)
void linear_distance_white(unsigned int distance)
{
	
	float reqshaftcount = 0;
	unsigned long int reqshaftcountint = 0;

	reqshaftcount = distance / 5.338; // division by resolution to get shaft count
	reqshaftcountint = (unsigned long int) reqshaftcount;

    shaftcountright = 0;
	
	while(1)
	{
		if(shaftcountright > reqshaftcountint)
  		{
  			break;
  		}

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		if(Center_white_line < 0x28)
		{
			forward();
			velocity(150, 150);		// Reduce velocity to avoid jerks.
			//velocity(110, 110);
		}

		if(Left_white_line > 0x28)
		{
			forward();
			velocity(120, 50);
		}

		if(Right_white_line > 0x28)
		{
			forward();
			velocity(50, 120);
		}

		if(Center_white_line > 0x28 && Left_white_line > 0x28 && Right_white_line > 0x28)
		{
			forward();
			velocity(0, 0);
		}
	}

	stop();		
}


// This function moves the bot on black line by given distance(in mm)
void linear_distance_black(unsigned int distance)
{
	
	float reqshaftcount = 0;
	unsigned long int reqshaftcountint = 0;

	reqshaftcount = distance / 5.338; // division by resolution to get shaft count
	reqshaftcountint = (unsigned long int) reqshaftcount;

    shaftcountright = 0;
	lcd_print(2,7,40,2);
	
	while(1)
	{
		if(shaftcountright > reqshaftcountint)
  		{
  			break;
  		}

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

	

		if (Center_white_line > 0x28 && Left_white_line < 0x28 && Right_white_line < 0x28)
		{
			forward();
			velocity(130, 130);
		}

		if(Center_white_line < 0x28 && Left_white_line < 0x28 && Right_white_line > 0x28)
		{
			forward();
			velocity(120, 50);
		}

		if (Center_white_line < 0x28 && Left_white_line > 0x28 && Right_white_line < 0x28)
		{
			forward();
			velocity(50, 120);
		}

		if(Center_white_line > 0x28 && Left_white_line > 0x28 && Right_white_line > 0x28)
		{
			flag = flag + 1;
			break;
		}
	}

	//stop();		// stop() is commented to avoid jerks while movement of bot
}

// This function moves bot to the given trough number
void go_to_trou(int n)
{
    switch(n)
    {
        case 1:
            break;
        case 2:
            while(flag == 0)
            {
                forward();
                linear_distance_black(3);
            }
            forward();
            linear_distance_black(70);
            stop();
            break;
        case 3:
            break;
        case 4:
            go_to_trou(2);
            break;
    }
}

// This function returns distance of object from sensor on the basis of adc value
unsigned int Sharp_dis_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int) (10.00 * (2799.6 * (1.00 / (pow(adc_reading, 1.1546)))));
	distanceInt = (int) distance;
	if(distanceInt > 800)
	{
		distanceInt = 800;
	}
	return distanceInt;
}

void print_sensor(char row, char column, unsigned char channel)
{
    ADC_Value = ADC_Conversion(channel);
    lcd_print(row, column, ADC_Value, 3);
}


void timer5_init()
{
    TCCR5B=0x00;
    TCNT5H=0xFF;
    TCNT5L=0x00;
    OCR5AH=0x00;
    OCR5BH=0x00;
    OCR5BL=0xFF;
    OCR5CH=0x00;
    OCR5CL=0xFF;
    TCCR5A=0xA9;
    TCCR5B=0x0B;
}

void init()
{
    cli();
    motion_init();
    init_r();
    init_l();
    left_position_encoder_interrupt_init();
    right_position_encoder_interrupt_init();
    adc_init();
    adc_pin_config();
    uart0_init();
    sei();
}

void servo1_pin_config (void)
{
 	DDRB  = DDRB | 0x20;  		//making PORTB 5 pin output
 	PORTB = PORTB | 0x20; 		//setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 	DDRB  = DDRB | 0x40;  		//making PORTB 6 pin output
 	PORTB = PORTB | 0x40; 		//setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 	DDRB  = DDRB | 0x80;  		//making PORTB 7 pin output
 	PORTB = PORTB | 0x80; 		//setting PORTB 7 pin to logic 1
}


void port_init_servo(void)
{ 
	servo1_pin_config();
	servo2_pin_config();
	servo3_pin_config();
}

void timer1_init(void)
{
 	TCCR1B = 0x00; 				//stop
 	TCNT1H = 0xFC; 				//Counter high value to which OCR1xH value is to be compared with
 	TCNT1L = 0x01;				//Counter low value to which OCR1xH value is to be compared with
 	OCR1AH = 0x03;				//Output compare eegister high value for servo 1
 	OCR1AL = 0xFF;				//Output Compare Register low Value For servo 1
 	OCR1BH = 0x03;				//Output compare eegister high value for servo 2
 	OCR1BL = 0xFF;				//Output Compare Register low Value For servo 2
 	OCR1CH = 0x03;				//Output compare eegister high value for servo 3
 	OCR1CL = 0xFF;				//Output Compare Register low Value For servo 3
 	ICR1H  = 0x03;	
 	ICR1L  = 0xFF;
 	TCCR1A = 0xAB; 				/*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 									For Overriding normal port functionalit to OCRnA outputs.
				  				{WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 	TCCR1C = 0x00;
 	TCCR1B = 0x0C; 				//WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

void init_devices_servo(void)
{
 	cli(); 						//disable all interrupts
 	port_init_servo();
 	timer1_init();
 	sei(); 						//re-enable interrupts 
}

//Function to rotate Servo 1 by a specified angle in the multiples of 2.25 degrees
void servo_1(unsigned char degrees)  
{
 	float PositionPanServo = 0;
 	PositionPanServo = ((float)degrees / 2.25) + 21.0;
 	OCR1AH = 0x00;
 	OCR1AL = (unsigned char) PositionPanServo;
}

//Function to rotate Servo 2 by a specified angle in the multiples of 2.25 degrees
void servo_2(unsigned char degrees)
{
 	float PositionTiltServo = 0;
 	PositionTiltServo = ((float)degrees / 2.25) + 21.0;
 	OCR1BH = 0x00;
 	OCR1BL = (unsigned char) PositionTiltServo;
}


//Function to rotate Servo 3 by a specified angle in the multiples of 2.25 degrees
void servo_3(unsigned char degrees)
{
 	float PositionTiltServo = 0;
 	PositionTiltServo = ((float)degrees / 2.25) + 21.0;
 	OCR1CH = 0x00;
 	OCR1CL = (unsigned char) PositionTiltServo;
}

void servo_1_free (void) 	//makes servo 1 free rotating
{
 	OCR1AH = 0x03; 
 	OCR1AL = 0xFF; 			//Servo 1 off
}

void servo_2_free (void) 	//makes servo 2 free rotating
{
 	OCR1BH = 0x03;
 	OCR1BL = 0xFF; 			//Servo 2 off
}

void servo_3_free (void) 	//makes servo 3 free rotating
{
 	OCR1CH = 0x03;
 	OCR1CL = 0xFF; 			//Servo 3 off
} 



void start_weed() // for troughs on the left side of bot
{
    unsigned int value;   // value will store the calculated distance from weed
		
    servo_3(80);
    
    _delay_ms(1000);

    flag=0;
    
    while(1)
    {
        if(flag > 0)          // code for moving the bot correctly with the help of checkpoints 
        {
            stop();
            left();                       
            angle_rotate(180);
            flag = 0;
            go_to_trou(2);
            forward();
            linear_distance_black(80);
            left();
            angle_rotate(180);
            break;
        }
        forward();
        linear_distance_black(3);
        lcd_print(1,7,12,2);

        ADC_Value = ADC_Conversion(44);
        value = Sharp_dis_estimation(ADC_Value);  // calculating the distance form weeds

        lcd_print(1,1,ADC_Value,3);
        lcd_print(2,1,value,3);
        lcd_print(1,10,14,2);

        if(value < 400 && value > 150)  // weed is identified
        {
        	forward();
		    linear_distance_black(30);
		    velocity(150, 150);
		    left();                    // moving left as trough is on the left side
		    angle_rotate(80);
    		
    		while(1)   // code for rotating left through 7 degree and sensing trough front sensor
	    	{
	        	ADC_Value=ADC_Conversion(45);
	        	lcd_print(1,1,ADC_Value,3);
	        	
	        	if (ADC_Value < 40)
	        	{
	            	left();
	            	angle_rotate(7);
	            	_delay_ms(1000);
	        	}
	        	else
	        	{
	            	left();
	            	angle_rotate(7);
	            	_delay_ms(1000);
	            	break;
	        	}
	    	}

	    	servo_2(100);
	    	_delay_ms(1000);
	    	servo_2_free();

	    	servo_1(0);
            _delay_ms(1000);
            servo_1_free();

    		forward();
    		distance(40);   //for 18cm

            servo_1(180);
            _delay_ms(1000);

            servo_3(45);
            _delay_ms(1000);

    		backward();
    		distance(80);

            servo_3(80);
            _delay_ms(1000);

        	servo_1(0);
            _delay_ms(2000);
            servo_1_free();

        	servo_2(180);
    		_delay_ms(1000);
    		servo_2_free();
            _delay_ms(1000);

    		right();
	    	angle_rotate(90);
	    }
    }
}	

void start_weed1() // code for troughs on right side of bot
{
    unsigned int value;
    while(1)
    {
        if(flag > 0)
        {
            stop();
            left();
            angle_rotate(180);
            flag = 0;
            go_to_trou(2);
            forward();
            linear_distance_black(80);
            left();
            angle_rotate(180);
            break;
        }
        forward();
        linear_distance_black(3);
        lcd_print(1,7,12,2);

        ADC_Value = ADC_Conversion(11);
        value = Sharp_dis_estimation(ADC_Value);
        
        lcd_print(1,1,ADC_Value,3);
        lcd_print(2,1,value,3);
        lcd_print(1,10,14,2);
        
        if(value < 400 && value > 150) // weed detected
        {
            forward();
            linear_distance_black(30);
    		velocity(150, 150);
	    	right();
	    	angle_rotate(80);
	    
	    	while(1) // code for rotating right through 7 degree and sensing trough front sensor
		    {
		        ADC_Value = ADC_Conversion(45);

        		lcd_print(1,1,ADC_Value,3);

        		if (ADC_Value < 40)
        		{
            		right();
            		angle_rotate(7);
            		_delay_ms(1000);
        		}
        		else
        		{
            		right();
            		angle_rotate(7);
            		_delay_ms(1000);
            		break;
        		}
    		}

    		servo_2(100);
    		_delay_ms(1000);
    		servo_2_free();

    		servo_1(0);
		_delay_ms(1000);
            	servo_1_free();

    		forward();
    		distance(40);   //for 18cm
		
	        servo_1(180);
        	_delay_ms(1000);

	        servo_3(45);
        	_delay_ms(1000);

    		backward();
    		distance(70);
		
	        servo_3(90);
        	_delay_ms(1000);
        
        	servo_1(0);
	        _delay_ms(2000);
        	servo_1_free();
        
        	servo_2(180);
    		_delay_ms(1000);
    		servo_2_free();
	        _delay_ms(1000);

    		left();
    		angle_rotate(90);
    	}
    }
}	


char fcall[5][5];
int i = 0, j = 0;
int botId;

// Pass the message to be passed as a string
void send_status(char msg[])
{
	int i;
	UDR0 = 0x28;
	
	for(i = 0; msg[i] != 0; i++)
	{
		while ( !( UCSR0A & (1 << UDRE0)) );
		UDR0 = msg[i];
	}
	
	while ( !( UCSR0A & (1 << UDRE0)) );
	UDR0 = 0x29;
}

void forward123(int n)
{
	char msg[] = "Forward Half Done";

	PORTA = 0x06; //forward
	_delay_ms(n*50);
	send_status(msg);
	_delay_ms(n*50);
	PORTA = 0x00; //stop
}

void starter(int par)
{
    go_to_trou(par);
    if(par > 2)
  	    start_weed1();
    else
      	start_weed();
    send_status("DONE");
}     

void function_caller()
{
	int val, par;
	val = atoi(fcall[1]);
	par = atoi(fcall[2]);
	switch(val)
	{
		case 1 : forward123(par); break;
		case 2 : starter(par);break;
		default : UDR0 = 0x26;
	}
}

//FORMAT "botId$funCode$par1$par2$par3#"
SIGNAL (SIG_USART0_RECV)
{
	cli();
	data = UDR0;

	if(data == 0x23) // #
	{
		if(atoi(fcall[0]) == botId)
		{
			if(j != 0)
			{
				fcall[i][j] = 0;
				sei();
				function_caller();
				cli();
			}
			UDR0 = data;
		}
		i = 0;
		j = 0;
	}
	else if(data == 0x24) // $
	{
		fcall[i][j] = 0;
		i++;
		j = 0;
	}
	else
	{	
		fcall[i][j] = data;
		j++;
	}
	sei();
}

//////////////////////////////////////////////////////////

int main(void)
{
    botId = 5;

    init();
    timer5_init();
    lcd_port_config();
    lcd_init();
    init_devices_servo();
    
    while(1);
}
