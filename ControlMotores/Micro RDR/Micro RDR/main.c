/*
 * Micro RDR.c
 *
 * Created: 24/08/2021 01:07:33 p. m.
 * Author : AlejandroDaniel
 */ 

#define F_CPU 16000000UL

#include "util/delay.h"
#include <avr/interrupt.h>
#include <avr/io.h>

int getIDMov(void);
void enviarMov(int);

// Pines para leer movimiento enviado por la raspberry
uint8_t pinID_0 = 2;
uint8_t pinID_1 = 3;
uint8_t pinID_2 = 4;

// Pines para enviar movimiento a microIMA y microPLM
uint8_t pin_out0 = 3;
uint8_t pin_out1 = 4;
uint8_t pin_out2 = 5;

// Pin para interrupción microIMA
uint8_t pin_leer = 2;

// Variables para lectura de movimiento
uint8_t lectura[3] = {0, 0 ,0};
uint8_t moda = 0;


int main(void)
{
 
	cli();   
	_delay_ms(200);
	
	DDRC = 0x00;   // 0000 0000 (PC2, PC3, PC4 como entradas de control)
	DDRB = 0x38;   // 0011 1000 (PB3, PB4, PB5 como salidas hacia microPLM y microIMA)

	PORTC |= ((1<<2) | (1<<3) | (1<<4));  // Activar resistencia pull-up para PC2, PC3 y PC4
	PORTB &= ~((1<<3) | (1<<4) | (1<<5));  // Poner en bajo PB3, PB4, PB5
	
	sei();
	
    while (1) 
    {
		
		_delay_ms(250);
		lectura[0] = getIDMov();
		lectura[1] = getIDMov();
		lectura[2] = getIDMov();


		if((lectura[0] == lectura[1]) && (lectura[0] == lectura[2]))
		{
			moda = lectura[0];
		}
		else if((lectura[0] == lectura[1]) && (lectura[0] != lectura[2]))
		{
			moda = lectura[0];
		}
		else if((lectura[0] != lectura[1]) && (lectura[0] == lectura[2]))
		{
			moda = lectura[0];
		}
		else if((lectura[1] == lectura[2]) && (lectura[0] != lectura[1]))
		{
			moda = lectura[1];
		}
		else
		{
			moda = lectura[1];
		}

		enviarMov(moda);

		_delay_ms(300);
		
    }
}



int getIDMov(void)
{
	uint8_t clave_mov[3] = {0, 0 ,0};
	uint8_t id_mov = 0;
	
	// Obtiene los estados de los pines de control
	if (((PINC & (1 << PINC2)) >> PINC2) == 1)
	{
		clave_mov[0] = 1;
	}
	else
	{
		clave_mov[0] = 0;
	}
	
	if (((PINC & (1 << PINC3)) >> PINC3) == 1)
	{
		clave_mov[1] = 2;
	}
	else
	{
		clave_mov[1] = 0;
	}
	
	if (((PINC & (1 << PINC4)) >> PINC4) == 1)
	{
		clave_mov[2] = 4;
	}
	else
	{
		clave_mov[2] = 0;
	}
	
	id_mov = clave_mov[0] + clave_mov[1] + clave_mov[2];

	return(id_mov);
}


void enviarMov(int mode)
{

	if(mode == 0)
	{
		PORTB &= ~((1<<pin_out2) | (1<<pin_out1) | (1<<pin_out0));
	}
	else if(mode == 1)
	{
		PORTB &= ~((1<<pin_out2) | (1<<pin_out1));
		PORTB |= (1<<pin_out0);
	}
	else if(mode == 2)
	{
		PORTB &= ~((1<<pin_out2) | (1<<pin_out0));
		PORTB |= (1<<pin_out1);
	}
	else if(mode == 3)
	{
		PORTB &= ~(1<<pin_out2);
		PORTB |= ((1<<pin_out1) | (1<<pin_out0));
	}
	else if(mode == 4)
	{
		PORTB &= ~((1<<pin_out1) | (1<<pin_out0));
		PORTB |= (1<<pin_out2);
	}
	else if(mode == 5)
	{
		PORTB &= ~(1<<pin_out1);
		PORTB |= ((1<<pin_out2) | (1<<pin_out0));
	}
	else if(mode == 6)
	{
		PORTB &= ~(1<<pin_out0);
		PORTB |= ((1<<pin_out2) | (1<<pin_out1));
	}
	else if(mode == 7)
	{
		PORTB |= ((1<<pin_out2) | (1<<pin_out1) | (1<<pin_out0));
	}

}

