

#ifndef F_CPU
#define F_CPU 1000000UL // 16 MHz clock speed
#endif
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define DHT11_PIN 6

uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;

#define LCD_Data_Dir DDRB		/* Define LCD data port direction */
#define LCD_Command_Dir DDRC	/* Define LCD command port direction register */
#define LCD_Data_Port PORTB		/* Define LCD data port */
#define LCD_Command_Port PORTC	/* Define LCD data port */
#define RS PC3					/* Define Register Select (data/command reg.)pin */
#define RW PC4					/* Define Read/Write signal pin */
#define EN PC5					/* Define Enable signal pin */


int display=0;
void LCD_Command(unsigned char cmnd)
{
	LCD_Data_Port= cmnd;
	LCD_Command_Port &= ~(1<<RS);	/* RS=0 command reg. */
	LCD_Command_Port &= ~(1<<RW);	/* RW=0 Write operation */
	LCD_Command_Port |= (1<<EN);	/* Enable pulse */
	_delay_us(1);
	LCD_Command_Port &= ~(1<<EN);
	_delay_ms(3);
}

void LCD_Char (unsigned char char_data)	/* LCD data write function */
{
	LCD_Data_Port= char_data;
	LCD_Command_Port |= (1<<RS);	/* RS=1 Data reg. */
	LCD_Command_Port &= ~(1<<RW);	/* RW=0 write operation */
	LCD_Command_Port |= (1<<EN);	/* Enable Pulse */
	_delay_us(1);
	LCD_Command_Port &= ~(1<<EN);
	_delay_ms(1);
}

void LCD_Init (void)			/* LCD Initialize function */
{
	LCD_Command_Dir |= (1<<PORTC5) | (1<<PORTC4) | (1<<PORTC3);		/* Make LCD command port direction as o/p */
	LCD_Data_Dir = 0xFF;		/* Make LCD data port direction as o/p */
	_delay_ms(20);			/* LCD Power ON delay always >15ms */
	
	LCD_Command (0x38);		/* Initialization of 16X2 LCD in 8bit mode */
	LCD_Command (0x0C);		/* Display ON Cursor OFF */
	LCD_Command (0x06);		/* Auto Increment cursor */
	LCD_Command (0x01);		/* Clear display */
	LCD_Command (0x80);		/* Cursor at home position */
}

void LCD_String (char *str)		/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)		/* Send each char of string till the NULL */
	{
		LCD_Char (str[i]);
	}
}
void LCD_String_Move (char *str)		/* Send string to LCD function */
{
	int i;
	// do the string shuffle
	c = str[15]; // take copy of the left most character
	// shuffle all the others one place to the left
	LCD_String (str);
	for (i = 15; i >0; i--) {
		str[i] = str[i - 1];
		
	}
	// put the saved character on the end
	str[0] = c;
	// just in case it's disturbed
	str[16] = 0; // end of string marker
	

}
void LCD_String_xy (char row, char pos, char *str)/* Send string to LCD with xy position */
{
	if (row == 0 && pos<16)
	LCD_Command((pos & 0x0F)|0x80);	/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	LCD_Command((pos & 0x0F)|0xC0);	/* Command of first row and required position<16 */
	LCD_String(str);		/* Call LCD string function */
}

void LCD_Clear()
{
	LCD_Command (0x01);		/* clear display */
	LCD_Command (0x80);		/* cursor at home position */
}

void Request()				/* Microcontroller send start pulse/request */
{
	DDRD |= (1<<DHT11_PIN);
	PORTD &= ~(1<<DHT11_PIN);	/* set to low pin */
	_delay_ms(18);			/* wait for 18ms */
	PORTD |= (1<<DHT11_PIN);	/* set to high pin */
}

void Response()				/* receive response from DHT11 */
{
	DDRD &= ~(1<<DHT11_PIN);
	while(PIND & (1<<DHT11_PIN));
	while((PIND & (1<<DHT11_PIN))==0);
	while(PIND & (1<<DHT11_PIN));
}

uint8_t Receive_data()			/* receive data */
{
	
	for (int q=0; q<8; q++)
	{
		
		while((PIND & (1<<DHT11_PIN)) == 0);  /* wait for logic 0 */
		{
		}
		_delay_us(30);
		//PORTB = 0x02;
		if(PIND & (1<<DHT11_PIN))/* if high pulse is greater than 30ms */
		c = (c<<1)|(0x01);	/* then its logic HIGH */
		else			/* otherwise its logic LOW */
		c = (c<<1);
		//PORTB = 0x04;
		while(PIND & (1<<DHT11_PIN));
	}
	return c;
}

void ADC_Init()
{
	ADMUX = (1<<REFS0);     //select AVCC as reference
	ADCSRA = (1<<ADEN)|(1<<ADPS0)|(1<<ADPS1);  //enable and prescale = 128 (16MHz/128 = 125kHz)
}

//int ADC_Read(char channel)
int ADC_Read(int ADCchannel)
{
	//channel &= 0b0000111;
	//ADMUX |= (1<<MUX0);
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);		//select input and ref
	ADCSRA |= (1<<ADSC);                 //start the conversion
	while (ADCSRA & (1<<ADSC));          //wait for end of conversion
	return ADCW;
}

ISR (INT0_vect)
{
	if(display==0){
		display=1;
		}else{
		display=0;
	}
}

int main(void)
{
	float soil_sensor;
	char data[5];
	int emo=0;
	LCD_Init();                 /* initialize 16x2 LCD*/
	LCD_Clear();
	ADC_Init();                 /* initialize ADC*/

	DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
	// PD2 (PCINT0 pin) is now an input

	PORTD |= (1 << PORTD2);    // turn On the Pull-up
	// PD2 is now an input with pull-up enabled



	EICRA |= (1 << ISC01);    // set INT0 to trigger on ANY logic change
	EIMSK |= (1 << INT0);     // Turns on INT0
	LCD_Clear();
	LCD_String("   Welcome!!!   ");	/* write string on 1st line of LCD*/
	_delay_ms(2000);
	sei();
	while(1)
	{
		soil_sensor = ADC_Read(0);
		soil_sensor = 100-(soil_sensor*100)/1023;
		//light_sensor = ADC_Read(1);
		Request();		/* send start pulse */

		Response();		/* receive response */
		
	    I_RH=Receive_data();	/* store first eight bit in I_RH */
		D_RH=Receive_data();	/* store next eight bit in D_RH */
		I_Temp=Receive_data();	/* store next eight bit in I_Temp */
		D_Temp=Receive_data();	/* store next eight bit in D_Temp */
		CheckSum=Receive_data();/* store next eight bit in CheckSum */
		
		if(display==0){
			LCD_Clear();
			LCD_String("H:");	
			itoa(I_RH,data,10);
			LCD_String(data);
			LCD_String(".");
			itoa(D_RH,data,10);
			LCD_String(data);
			LCD_String("%    ");
			LCD_String("S:");	
			itoa(soil_sensor,data,10);
			LCD_String(data);
			LCD_String("%");
			LCD_Command(0xC0);		
			LCD_String("T:");	
			itoa(I_Temp,data,10);
			LCD_String(data);
			LCD_String(".");
			itoa(D_Temp,data,10);
			LCD_String(data);
			LCD_String("\xDF");
			itoa(0xdf,data,10);
			LCD_String("C   ");
		}else{
			LCD_Clear();
			if((I_Temp>4 && I_Temp<30)&&(soil_sensor >=10 && soil_sensor<=20)&&(I_RH<80)){
				LCD_String_Move("Very happy <3   ");
				if(emo==1){
					LCD_String_xy(1,0,"     ( >o< )    ");
					emo=0;
				}else{
					LCD_String_xy(1,0,"     ( >.< )    ");
					emo=1;
				}
			}else if(soil_sensor > 30){
			LCD_String_Move("Too much water!");
				if(emo==1){
					LCD_String_xy(1,0,"     ( Y.Y')    ");
					emo=0;
					}else{
					LCD_String_xy(1,0,"     ( YoY )    ");
					emo=1;
				}
			}else if(soil_sensor <10){
			LCD_String_Move("Need more water!");
			if(emo==1){
				LCD_String_xy(1,0,"     ( U.U')    ");
				emo=0;
				}else{
				LCD_String_xy(1,0,"     ( U_U )!    ");
				emo=1;
				}
			}else if(I_Temp <4) {
				LCD_String_Move("Too cold..!     ");
				if(emo==1){
					LCD_String_xy(1,0,"     ( >.< )    ");
					emo=0;
					}else{
					LCD_String_xy(1,0,"     ( O.O )    ");
					emo=1;
				}		
			}else if(I_Temp >40) {
			LCD_String_Move("Too hot..!      ");
			if(emo==1){
				LCD_String_xy(1,0,"     ( -w- )!!    ");
				emo=0;
				}else{
				LCD_String_xy(1,0,"     ( -.- )!    ");
				emo=1;
				}
			}else if(I_RH>80) {
			LCD_String_Move("What happen..!  ");
			if(emo==1){
				LCD_String_xy(1,0,"     ( O.o )?    ");
				emo=0;
				}else{
				LCD_String_xy(1,0,"     ( o.O )    ");
				emo=1;
			}
		}else{
			LCD_String_Move("I'm fine :)     ");
			if(emo==1){
				LCD_String_xy(1,0,"     ( ^_^ )    ");
				emo=0;
				}else{
				LCD_String_xy(1,0,"     ( ^.^ )    ");
				emo=1;
			}			
		}
		}
		_delay_ms(1000);
	}
}

