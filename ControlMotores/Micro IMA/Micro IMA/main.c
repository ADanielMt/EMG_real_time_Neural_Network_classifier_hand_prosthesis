/*
 * Micro IMA.c
 *
 * Created: 02/06/2021 11:56:55 p. m.
 * Author : AlejandroDaniel
 */ 

#define F_CPU 16000000UL

#define BAUD 9600
#include <util/setbaud.h>
#include <stdio.h>
#include <stdlib.h>
#include "util/delay.h"
#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "UART/UART.h"


void configPWM(void);
void ActivarPWM_Ind(uint8_t PWM, uint8_t encendido);
void ActivarPWM_Med(uint8_t PWM, uint8_t encendido);
void ActivarPWM_Anu(uint8_t PWM, uint8_t encendido);
void configADC(void);
int ADC_GetData(int canalADC);
void calibrar(void);
void configInterrupt(void);
void getSerialParams(void);
//void getIDMov(void);
int getIDMov(void);
void getPulsosMov(void);
//void pulgarCero(void);


// Variables para comunicación serial
uint8_t numChars = 32;
char RxBuffer[32];
volatile unsigned char RxContador = 0;
uint8_t serialCode = 0;
int serialPulse = 0;
int newData = 0;
int serial_ind = 0;
int serial_med = 0;
int serial_anu = 0;
uint8_t recibiendo = 0;
uint8_t j = 0;

// Variables para movimientos y comunicación con Micro ima
uint8_t count_event = 0;	// Cuenta las interrupciones (estado alto) de pulgar listo de micro plm para máquina de estados
uint8_t iniciar_sis = 0;	// Si 0, indica que el sistema debe calibrarse, si 1, indica que está calibrado para iniciar
uint8_t enc_calib = 0;		// 1: Los encoders están calibrados, 0: lo contrario.
uint8_t pin_pul_listo = 3;  // Entrada: Pin del puerto C asignado al pulgar listo.
uint8_t pulgar_listo = 0;	// Auxiliar: Indica que el pulgar está en posición de inicio y calibrado a 0
uint8_t ima_listo = 0;		// Entrada: Indica que el índice, medio y anular están en posición de inicio y en 0
uint8_t plm_listo = 0;
uint8_t pin_ima_listo = 4;  // Salida: Pin del puerto C asignado a señal de índice, medio y anular listos.
uint8_t clave_mov[3] = {0, 0 ,0};
uint8_t id_mov = 0;			// Entrada: ID del movimiento enviado por la raspberry
uint8_t id_mov_ant = 0;
uint8_t new_mov = 0;		// Salida: Indica a la raspberry que ya puede enviar un nuevo movimiento
int mov[8] = {0, 0, 0, 0, 0, 0, 0 , 0}; // Entrada: Lee movimiento detectado por la Raspberry

// Variables para motores
uint8_t onoff_ind = 0;
uint8_t onoff_med = 0;
uint8_t onoff_anu = 0;
uint8_t pwm_ind = 0;
uint8_t pwm_med = 0;
uint8_t pwm_anu = 0;
uint8_t dir_ind = 0;
uint8_t dir_med = 0;
uint8_t dir_anu = 0;
uint8_t dirpin_ind = 7;
uint8_t dirpin_med = 0;
uint8_t dirpin_anu = 2;
uint8_t ind_ready = 0;
uint8_t med_ready = 0;
uint8_t anu_ready = 0;
uint8_t PWM_mot=0;

// Variables para encoders
int count_enc_ind = 0;
int count_enc_med = 0;
int count_enc_anu = 0;
int abs_indice = 0;
int abs_medio = 0;
int abs_anular = 0;
int rel_indice = 0;
int rel_medio = 0;
int rel_anular = 0;
int signo_ind = 0;
int signo_med = 0;
int signo_anu = 0;
int limite_ind = 800;
int limite_med = 800;
int limite_anu = 800;

// Variables para Interrupciones PCINT
volatile uint8_t portdhistory = 0xFF;     // Por default en alto por la resistencia pull-up
volatile uint8_t portchistory = 0xFF;     // Por default en alto por la resistencia pull-up

// Pines ADC: FSR - IR, O: FC
uint8_t FSRpin_ind = 0;
uint8_t FSRpin_med = 1;
uint8_t FSRpin_anu = 2;
//uint8_t FC_pin = 3;

// Pin de comunicación con raspberry para indicar que clasifique nuevo movimiento
uint8_t raspB_pin = 5;

// Variables para almacenar valores de FSR, IR y FC
int fsr_ind = 0;
int fsr_med = 0;
int fsr_anu = 0;
//int fc_lat = 0;

// Variables para guardar posiciones de cada dedo
//                  (0)      (1)        (2)     (3)    (4)     (5)       (6)           (7)          (8)
//               descanso, cilindro, lateral, pinza, gancho, cuernos, esférico, índice extendido, calibrar
int indice[9] =  {  700,	400,	    0,    450,      0,	   900,      400,		  800,            0 };
int medio[9]  =  {  650,	300,	    0,      0,      0,	     0,      500,			0,	          0 };
int anular[9] =  {  750,	400,	    0,      0,      0,	     0,      600,		    0,	          0 };


// Variables de apoyo
int pulsos_ind = 0;
int pulsos_med = 0;
int pulsos_anu = 0;
uint8_t aux = 0;
uint8_t aux2 = 0;

// Variables para verificación de lectura de movimiento
uint8_t lectura[3] = {0, 0 ,0};
uint8_t moda = 0;


int main(void)
{
	cli();
	
	DDRC = 0x30;   // 0011 0000 (PC0-FSR_ind, PC1-FSR_med, PC2-FSR_anu),(PC3-pulgar listo), (PC4-ima_listo), (PC5-nuevo mov a raspb)
	DDRD = 0xE2;   // 1110 0010 (PD0-PD1-UART), (PD2-Enc_ind, PD3-Enc_med, PD4-Enc_anu), (PD5-PWM_ind, PD6-PWM_med), (PD7-Dir_ind)
	DDRB = 0x07;   // 0000 0111 (PB0-Dir_med, PB2-Dir_anu), (PB1-PWM_anu), (PB3:5-Control), (PB7-PB7-Clock)
	
	PORTC |= (1<<3);	// Activar resistencia pull-up para PC3
	PORTB |= ((1<<3) | (1<<4) | (1<<5));	// Activar resistencia pull-up para PB3, PB4 y PB5
	PORTC &= ~(1<<4);	// Poner en bajo PC4
	PORTC &= ~(1<<5);	// Poner en bajo PC5
	PORTD |= ((1<<2) | (1<<3) | (1<<4));	// Activar resistencia pull-up para PD2, PD3 y PD4
	PORTD &= ~((1<<5) | (1<<6) | (1<<7));	// Poner en bajo PD5, PD6, PD7
	PORTB &= ~((1<<0) | (1<<1) | (1<<2));	// Poner en bajo PB0, PB1, PB2
	
	configADC();   // Configura ADC
	configPWM();   // Configura PWM
	configInterrupt();   // Configura interrupciones de encoders y pulgar_listo
	UART_init();
	

	
	sei();
	
	
	while (1)
	{
		_delay_ms(250);
		_delay_ms(250);
		getSerialParams();
		//printf("Sis: %d \n", iniciar_sis);
		
		// Paso 1: Calibración inicial
		
		if(iniciar_sis == 0)		// (iniciar_sis == 0 && pulgar_listo == 1)
		{
			/*for (int t=0; t<20; t++)
			{
				_delay_ms(250);			// Espera 2 segundos para iniciar
			}*/
			
			printf("Hola usuario N \n ");
			_delay_ms(250);
			printf("Vamos a calibrar \n");
			_delay_ms(250);
			
			while (count_event != 1)
			{
				if ( (PINC & (1 << pin_pul_listo)) != 0 ) // (PINC & (1 << PINC3)) == (1 << PINC3)
				{
					count_event = 1;
					//printf("cat 0 \n");
				}
			}
			printf("cat 0 \n");
			calibrar();			
			iniciar_sis = 1;
			id_mov_ant = 8;
			
			while (count_event != 2)
			{
				if ( ((PINC & (1 << pin_pul_listo)) == 0) && ((PINC & (1 << pin_ima_listo)) == 0) )
				{
					PORTC |= (1 << raspB_pin);	// Poner pin C5 en alto, enviar indicación de nuevo movimiento a Raspberry
					PORTC |= (1<<pin_ima_listo);       // Poner pin en alto
					count_event = 2;
					printf("cat \n");
				}
			}
			
		}
		
		// Fin de paso 1
		
		// Paso 2: Detección de movimientos
		// if(count_event == 2)
		while(count_event == 2)
		{
			printf("In 2 \n");
			_delay_ms(5);
			//getIDMov();			// Obtener el código del movimiento deseado, leer señales de control
			
			///////////////// Sección en prueba //////////////////////////////////////
			
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
			
			id_mov = moda;
			
			///////////////// Fin de sección de prueba ///////////////////////////////
			
			
			if(id_mov == id_mov_ant)
			{
				// No hacer nada y salir del if principal para volver a leer señales de control
				PORTC |= (1 << raspB_pin);	// Para asegurarse que el pin C5 esté en alto
			}
			else if (((id_mov_ant == 1 || id_mov_ant == 3 || id_mov_ant == 4) && (id_mov != 0)) || ((id_mov == 1 || id_mov == 3 || id_mov == 4) && (id_mov_ant != 0)) )
			{
				// Si la posición anterior es de agarre, no hacer nada y salir del if principal
				// para esperar a que el usuario indique posición de descanso para liberar objeto.
				// Ó si la posición actual es agarre pero la posición anterior no es descanso,
				// salir del if principal para esperar a que el usuario indicque descanso y luego agarre.
				PORTC |= (1 << raspB_pin);	// Para asegurarse que el pin C5 esté en alto
			}
			else
			{
				printf("In m \n");
				_delay_ms(5);
				id_mov_ant = id_mov;
				
				PORTC &= ~(1 << raspB_pin);	// Poner C5 en bajo, indica a raspberry que el movimiento ha sido leído
				
				ActivarPWM_Ind(0,0);
				ActivarPWM_Med(0,0);
				ActivarPWM_Anu(0,0);
				
				getPulsosMov();		// Obtener el número de pulsos para alcanzar la posición
				
				while (count_event != 3)
				{
					if ( ((PINC & (1 << pin_pul_listo)) != 0) && ((PINC & (1 << pin_ima_listo)) != 0) )
					{
						PORTC &= ~(1<<pin_ima_listo);       // Poner pin en bajo
						count_event = 3;
					}
				}
				
				printf("If \n");
				_delay_ms(5);
				/////////////////////////////////////////////////////////////////////////////////////////
				if (id_mov == 0)			// Rutina para movimiento de descanso
				{
					printf("Mov 0 \n");
					_delay_ms(5);
					if (pulsos_ind != 0)
					{
						ActivarPWM_Ind(pwm_ind, onoff_ind);
						while(ind_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						abs_indice = abs_indice + (signo_ind * rel_indice);
						printf("Ind en pos: %d \n", abs_indice);
					}
					
					if (pulsos_med != 0)
					{
						ActivarPWM_Med(pwm_med, onoff_med);
						while(med_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						abs_medio = abs_medio + (signo_med * rel_medio);
						printf("Med en pos: %d \n", abs_medio);
					}
					
					if (pulsos_anu != 0)
					{
						ActivarPWM_Anu(pwm_anu, onoff_anu);
						while(anu_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						abs_anular = abs_anular + (signo_anu * rel_anular);
						printf("Anu en pos: %d \n", abs_anular);
					}
					
					printf("IMA listo \n");
					//PORTC |= (1<<pin_ima_listo);	// Avisar a Micro PLM que Micro IMA está en posición deseada
					//ima_listo = 1;					// Bandera: Indica que Micro IMA está en posición deseada
				}
				////////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 1)			// Rutina para agarre cilíndrico (de fuerza) ///////////////
				{
					if (pulsos_ind != 0)				// Si la posición actual es distinta de la deseada
					{
						ActivarPWM_Ind(pwm_ind, onoff_ind);
						while(ind_ready != 1)		// Mientras no alcance la posición guardada (límite)
						{
							if ((ADC_GetData(FSRpin_ind)) > 100)  // Evaluar FSR, si FSR en contacto con objeto:
							{									  // apagar motor e indicar que está en posición
								ActivarPWM_Ind(0,0);
								count_enc_ind = 0;
								onoff_ind = 0;
								pwm_ind = 0;
								ind_ready = 1;
								printf("Ind come \n");
							}
						}
						
						abs_indice = abs_indice + (signo_ind * rel_indice);
						printf("Ind en pos: %d \n", abs_indice);
					}
					
					if (pulsos_med != 0)
					{
						ActivarPWM_Med(pwm_med, onoff_med);
						while(med_ready != 1)		// Mientras no alcance la posición guardada (límite)
						{
							if ((ADC_GetData(FSRpin_med)) > 100)  // Evaluar FSR, si FSR en contacto con objeto:
							{									  // apagar motor e indicar que está en posición
								ActivarPWM_Med(0,0);
								count_enc_med = 0;
								onoff_med = 0;
								pwm_med = 0;
								med_ready = 1;
								printf("Med come \n");
							}
						}
						
						abs_medio = abs_medio + (signo_med * rel_medio);
						printf("Med en pos: %d \n", abs_medio);
					}
					
					if (pulsos_anu != 0)
					{
						ActivarPWM_Anu(pwm_anu, onoff_anu);
						while(anu_ready != 1)		// Mientras no alcance la posición guardada (límite)
						{
							if ((ADC_GetData(FSRpin_anu)) > 100)  // Evaluar FSR, si FSR en contacto con objeto:
							{									  // apagar motor e indicar que está en posición
								ActivarPWM_Anu(0,0);
								count_enc_anu = 0;
								onoff_anu = 0;
								pwm_anu = 0;
								anu_ready = 1;
								printf("Anu come \n");
							}
						}
						
						abs_anular = abs_anular + (signo_anu * rel_anular);
						printf("Anu en pos: %d \n", abs_anular);
					}
					
					printf("IMA listo \n");
					//PORTC |= (1<<pin_ima_listo);	// Avisar a Micro PLM que Micro IMA está en posición deseada
					//ima_listo = 1;					// Bandera: Indica que Micro IMA está en posición deseada
				}
				
				////////////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 2)			// Rutina para puño/agarre lateral ///////////////
				{
					if (pulsos_ind != 0)				// Si la posición actual es distinta de la deseada
					{
						pulsos_ind = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Ind(pwm_ind, onoff_ind);
						while(ADC_GetData(FSRpin_ind) < 40)		// Mientras los sensores no toquen la palma
						{
						}
						ActivarPWM_Ind(0,0);
						count_enc_ind = 0;
						onoff_ind = 0;
						pwm_ind = 0;
						ind_ready = 1;
						printf("Ind come \n");
						
						abs_indice = 0; // abs_menique + (signo_men * rel_menique);
						printf("Ind en pos: %d \n", abs_indice);
					}
					
					if (pulsos_med != 0)		// Si la posición actual es distinta de la deseada
					{
						pulsos_med = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Med(pwm_med, onoff_med);
						while(ADC_GetData(FSRpin_med) < 40)		// Mientras los sensores no toquen la palma
						{
						}
						ActivarPWM_Med(0,0);
						count_enc_med = 0;
						onoff_med = 0;
						pwm_med = 0;
						med_ready = 1;
						printf("Med come \n");
						
						abs_medio = 0; // abs_menique + (signo_men * rel_menique);
						printf("Med en pos: %d \n", abs_medio);
					}
					
					if (pulsos_anu != 0)		// Si la posición actual es distinta de la deseada
					{
						pulsos_anu = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Anu(pwm_anu, onoff_anu);
						while(ADC_GetData(FSRpin_anu) < 50)		// Mientras los sensores no toquen la palma
						{
						}
						ActivarPWM_Anu(0,0);
						count_enc_anu = 0;
						onoff_anu = 0;
						pwm_anu = 0;
						anu_ready = 1;
						printf("Anu come \n");
						
						abs_anular = 0; // abs_menique + (signo_men * rel_menique);
						printf("Anu en pos: %d \n", abs_anular);
					}
					
					printf("IMA listo \n");
					//PORTC |= (1<<pin_ima_listo);	// Avisar a Micro PLM que Micro IMA está en posición deseada
					//ima_listo = 1;					// Bandera: Indica que Micro IMA está en posición deseada
				}
				
				////////////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 3)			// Rutina para pinza ///////////////
				{
					
					if (pulsos_ind != 0)				// Si la posición actual es distinta de la deseada
					{
						ActivarPWM_Ind(pwm_ind, onoff_ind);
						while(ind_ready != 1)		// Mientras no alcance la posición guardada (límite)
						{
							if ((ADC_GetData(FSRpin_ind)) > 40)  // Evaluar FSR, si FSR en contacto con objeto:
							{									  // apagar motor e indicar que está en posición
								ActivarPWM_Ind(0,0);
								count_enc_ind = 0;
								onoff_ind = 0;
								pwm_ind = 0;
								ind_ready = 1;
								printf("Ind come \n");
							}
						}
						
						abs_indice = abs_indice + (signo_ind * rel_indice);
						printf("Ind en pos: %d \n", abs_indice);
					}
					
					if (pulsos_med != 0)		// Si la posición actual es distinta de la deseada
					{
						pulsos_med = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Med(pwm_med, onoff_med);
						while(ADC_GetData(FSRpin_med) < 40)		// Mientras los sensores no toquen la palma
						{
						}
						ActivarPWM_Med(0,0);
						count_enc_med = 0;
						onoff_med = 0;
						pwm_med = 0;
						med_ready = 1;
						printf("Med come \n");
						
						abs_medio = 0; // abs_menique + (signo_men * rel_menique);
						printf("Med en pos: %d \n", abs_medio);
					}
					
					if (pulsos_anu != 0)		// Si la posición actual es distinta de la deseada
					{
						pulsos_anu = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Anu(pwm_anu, onoff_anu);
						while(ADC_GetData(FSRpin_anu) < 50)		// Mientras los sensores no toquen la palma
						{
						}
						ActivarPWM_Anu(0,0);
						count_enc_anu = 0;
						onoff_anu = 0;
						pwm_anu = 0;
						anu_ready = 1;
						printf("Anu come \n");
						
						abs_anular = 0; // abs_menique + (signo_men * rel_menique);
						printf("Anu en pos: %d \n", abs_anular);
					}
					
					printf("IMA listo \n");
					//PORTC |= (1<<pin_ima_listo);	// Avisar a Micro PLM que Micro IMA está en posición deseada
					//ima_listo = 1;					// Bandera: Indica que Micro IMA está en posición deseada
					
				}
				
				////////////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 4)			// Rutina para like ///////////////
				{
					if (pulsos_ind != 0)				// Si la posición actual es distinta de la deseada
					{
						//pulsos_ind = 1000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Ind(pwm_ind, onoff_ind);
						while(ind_ready != 1)		// Mientras no esté en posición límite
						{
							if (ADC_GetData(FSRpin_ind) > 40)
							{
								ActivarPWM_Ind(0,0);
								count_enc_ind = 0;
								onoff_ind = 0;
								pwm_ind = 0;
								ind_ready = 1;
								printf("Ind come \n");
							}
						}
						
						abs_indice = abs_indice + (signo_ind * rel_indice);
						printf("Ind en pos: %d \n", abs_indice);
					}
					
					if (pulsos_med != 0)		// Si la posición actual es distinta de la deseada
					{
						//pulsos_med = 1000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Med(pwm_med, onoff_med);
						while(med_ready != 1)		// Mientras no esté en posición límite
						{
							if (ADC_GetData(FSRpin_med) > 40)
							{
								ActivarPWM_Med(0,0);
								count_enc_med = 0;
								onoff_med = 0;
								pwm_med = 0;
								med_ready = 1;
								printf("Med come \n");
							}
						}
						
						abs_medio = abs_medio + (signo_med * rel_medio);
						printf("Med en pos: %d \n", abs_medio);
					}
					
					if (pulsos_anu != 0)		// Si la posición actual es distinta de la deseada
					{
						//pulsos_anu = 1000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Anu(pwm_anu, onoff_anu);
						while(anu_ready != 1)		// Mientras no esté en posición límite
						{
							if (ADC_GetData(FSRpin_anu) > 50)
							{
								ActivarPWM_Anu(0,0);
								count_enc_anu = 0;
								onoff_anu = 0;
								pwm_anu = 0;
								anu_ready = 1;
								printf("Anu come \n");
							}
						}
						
						abs_anular = abs_anular + (signo_anu * rel_anular);
						printf("Anu en pos: %d \n", abs_anular);
					}
					
					printf("IMA listo \n");
					//PORTC |= (1<<pin_ima_listo);	// Avisar a Micro PLM que Micro IMA está en posición deseada
					//ima_listo = 1;					// Bandera: Indica que Micro IMA está en posición deseada
				}
				
				/////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 5)			// Rutina para movimiento de cuernos
				{
					if (pulsos_ind != 0)
					{
						ActivarPWM_Ind(pwm_ind, onoff_ind);
						while(ind_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						abs_indice = abs_indice + (signo_ind * rel_indice);
						printf("Ind en pos: %d \n", abs_indice);
					}
					
					if (pulsos_med != 0)		// Si la posición actual es distinta de la deseada
					{
						pulsos_med = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Med(pwm_med, onoff_med);
						while(ADC_GetData(FSRpin_med) < 40)		// Mientras los sensores no toquen la palma
						{
						}
						ActivarPWM_Med(0,0);
						count_enc_med = 0;
						onoff_med = 0;
						pwm_med = 0;
						med_ready = 1;
						printf("Med come \n");
						
						abs_medio = 0; // abs_menique + (signo_men * rel_menique);
						printf("Med en pos: %d \n", abs_medio);
					}
					
					if (pulsos_anu != 0)		// Si la posición actual es distinta de la deseada
					{
						pulsos_anu = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Anu(pwm_anu, onoff_anu);
						while(ADC_GetData(FSRpin_anu) < 50)		// Mientras los sensores no toquen la palma
						{
						}
						ActivarPWM_Anu(0,0);
						count_enc_anu = 0;
						onoff_anu = 0;
						pwm_anu = 0;
						anu_ready = 1;
						printf("Anu come \n");
						
						abs_anular = 0; // abs_menique + (signo_men * rel_menique);
						printf("Anu en pos: %d \n", abs_anular);
					}
					
					printf("IMA listo \n");
					//PORTC |= (1<<pin_ima_listo);	// Avisar a Micro PLM que Micro IMA está en posición deseada
					//ima_listo = 1;					// Bandera: Indica que Micro IMA está en posición deseada
				}
				
				/////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 6)			// Rutina para movimiento de esfera (palma fuera)
				{
					printf("In 6 \n");
					_delay_ms(5);
					if (pulsos_ind != 0)
					{
						printf("If ind \n");
						ActivarPWM_Ind(pwm_ind, onoff_ind);
						while(ind_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							//printf("I: %d \n", ind_ready);
							_delay_ms(20);
						}
						abs_indice = abs_indice + (signo_ind * rel_indice);
						printf("Ind en pos: %d \n", abs_indice);
						_delay_ms(5);
					}
					
					if (pulsos_med != 0)
					{
						printf("If med \n");
						ActivarPWM_Med(pwm_med, onoff_med);
						while(med_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							//printf("M: %d \n", med_ready);
							_delay_ms(20);
						}
						abs_medio = abs_medio + (signo_med * rel_medio);
						printf("Med en pos: %d \n", abs_medio);
						_delay_ms(5);
					}
					
					if (pulsos_anu != 0)
					{
						printf("If anu \n");
						ActivarPWM_Anu(pwm_anu, onoff_anu);
						while(anu_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							//printf("A: %d \n", anu_ready);
							_delay_ms(20);
						}
						abs_anular = abs_anular + (signo_anu * rel_anular);
						printf("Anu en pos: %d \n", abs_anular);
						_delay_ms(5);
					}
					
					printf("IMA listo \n");
					//_delay_ms(5);
					//PORTC |= (1<<pin_ima_listo);	// Avisar a Micro PLM que Micro IMA está en posición deseada
					//_delay_ms(100);
					//ima_listo = 1;					// Bandera: Indica que Micro IMA está en posición deseada
				}
				
				/////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 7)			// Rutina para movimiento de pistola
				{
					if (pulsos_ind != 0)
					{
						ActivarPWM_Ind(pwm_ind, onoff_ind);
						while(ind_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						abs_indice = abs_indice + (signo_ind * rel_indice);
						printf("Ind en pos: %d \n", abs_indice);
					}
					
					if (pulsos_med != 0)		// Si la posición actual es distinta de la deseada
					{
						pulsos_med = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Med(pwm_med, onoff_med);
						while(ADC_GetData(FSRpin_med) < 40)		// Mientras los sensores no toquen la palma
						{
						}
						ActivarPWM_Med(0,0);
						count_enc_med = 0;
						onoff_med = 0;
						pwm_med = 0;
						med_ready = 1;
						printf("Med come \n");
						
						abs_medio = 0; // abs_menique + (signo_men * rel_menique);
						printf("Med en pos: %d \n", abs_medio);
					}
					
					if (pulsos_anu != 0)		// Si la posición actual es distinta de la deseada
					{
						pulsos_anu = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Anu(pwm_anu, onoff_anu);
						while(ADC_GetData(FSRpin_anu) < 50)		// Mientras los sensores no toquen la palma
						{
						}
						ActivarPWM_Anu(0,0);
						count_enc_anu = 0;
						onoff_anu = 0;
						pwm_anu = 0;
						anu_ready = 1;
						printf("Anu come \n");
						
						abs_anular = 0; // abs_menique + (signo_men * rel_menique);
						printf("Anu en pos: %d \n", abs_anular);
					}
					
					printf("IMA listo \n");
					//PORTC |= (1<<pin_ima_listo);	// Avisar a Micro PLM que Micro IMA está en posición deseada
					//ima_listo = 1;					// Bandera: Indica que Micro IMA está en posición deseada
				}
				
				
				//////////////////// Checar que IMA y PLM estén en posición deseada ///////////////////////////////
				//_delay_ms(200);					// Esperar
				//PORTC &= ~(1<<pin_ima_listo);  // Poner en bajo señal enviada
				pulsos_anu = 0;
				pulsos_med = 0;
				pulsos_ind = 0;
				ima_listo = 1;
				
				//////////////////// Sección en prueba ///////////////////////////////
				////////// Reporte de posición de motores y fuerza aplicada //////////
				
				printf("Indice: %d \n", abs_indice);
				printf("Medio: %d \n", abs_medio);
				printf("Anular: %d \n", abs_anular);
				
				printf("Fuerza Ind: %d \n", ADC_GetData(FSRpin_ind));
				printf("Fuerza Med: %d \n", ADC_GetData(FSRpin_med));
				printf("Fuerza Anu: %d \n", ADC_GetData(FSRpin_anu));
				
				////////////////////// Fin de reporte ////////////////////////////////
				//////////////////////////////////////////////////////////////////////
				
				
				while (count_event != 4)		// Mientras PLM no esté en posición, no hacer nada
				{
					if ( ((PINC & (1 << pin_pul_listo)) == 0) && ((PINC & (1 << pin_ima_listo)) == 0) )
					{
						PORTC |= (1<<pin_ima_listo);
						count_event = 4;
					}
				}
				
				plm_listo = 1;					// Cuando PLM esté en posición (count_event = 4), poner bandera en 1
				printf("Listos \n");
				_delay_ms(5);
				
				if (plm_listo == 1 && ima_listo == 1)	// Si IMA y PLM están en posición, hacer count_event=1
				{										// para que pueda volver a leer señales de control
					PORTC |= (1 << raspB_pin);	// Poner pin C5 en alto, enviar indicación de nuevo movimiento a Raspberry
					_delay_ms(5);
					count_event = 2;
					anu_ready = 0;
					med_ready = 0;
					ind_ready = 0;
					plm_listo = 0;
					ima_listo = 0;
				}
				
			}
			
			printf("Wait new \n");
		}
		
		// Fin de paso 2
		
		
		if(serial_ind == 1)
		{
			if (serialCode == 0)
			{
				pwm_ind = 100;
				onoff_ind = 1;
				//dir_ind = 1;
				PORTD |= (1<<dirpin_ind);
				//PORTC &= ~(1 << 2);
				printf("Abrir indice \n");
				serial_ind = 0;
				ActivarPWM_Ind(pwm_ind, onoff_ind);
				_delay_ms(200);
			}
			else if (serialCode == 1)
			{
				pwm_ind = 100;
				onoff_ind = 1;
				//dir_ind = 1;
				//PORTD |= (1<<pwmpin_ind);
				PORTD &= ~(1 << dirpin_ind);
				printf("Cerrar indice \n");
				serial_ind = 0;
				ActivarPWM_Ind(pwm_ind, onoff_ind);
				_delay_ms(200);
			}
			
		}

		if(serial_med == 1)
		{
			if (serialCode == 2)
			{
				pwm_med = 120;
				onoff_med = 1;
				//dir_med = 1;
				PORTB |= (1<<dirpin_med);
				printf("Abrir med \n");
				serial_med = 0;
				ActivarPWM_Med(pwm_med, onoff_med);
				_delay_ms(200);
			}
			else if (serialCode == 3)
			{
				pwm_med = 100;
				onoff_med = 1;
				//dir_med = 0;
				//PORTB |= (1<<pwmpin_med);
				PORTB &= ~(1 << dirpin_med);
				printf("Cerrar med \n");
				serial_med = 0;
				ActivarPWM_Med(pwm_med, onoff_med);
				_delay_ms(200);
			}
		}
		
		if(serial_anu == 1)
		{
			if (serialCode == 4)
			{
				pwm_anu = 100;
				onoff_anu = 1;
				//dir_anu = 1;
				PORTB |= (1<<dirpin_anu);
				printf("Abrir anul \n");
				serial_anu = 0;
				ActivarPWM_Anu(pwm_anu, onoff_anu);
				_delay_ms(200);
			}
			else if (serialCode == 5)
			{
				pwm_anu = 100;
				onoff_anu = 1;
				//dir_anu = 0;
				//PORTB |= (1<<pwmpin_anu);
				PORTB &= ~(1 << dirpin_anu);
				printf("Cerrar anul \n");
				serial_anu = 0;
				ActivarPWM_Anu(pwm_anu, onoff_anu);
				_delay_ms(200);
			}
		}
		
		//printf("\n N pulsos: %d \n", serialPulse);
		//printf("Code: %d \n", serialCode);
		//printf("Count: %d \n", RxContador);
		_delay_ms(240);

	}
	
}



ISR(USART_RX_vect)
{	
	char dato = UDR0;
	
	if(recibiendo>0)
	{
		if (dato != '>' && j == 0)
		{
			serialCode = dato - '0';
			j ++;
		}
		else if(dato == '>')
		{
			
			RxBuffer[RxContador] = '\0';
			RxContador = 0;
			recibiendo = 0;
			j = 0;
			serialPulse = atol(RxBuffer);
			newData ++;
			
		}
		else
		{
			RxBuffer[RxContador++] = dato;
		}
	}
	else if(dato=='<')
	{
		recibiendo++;
	}
	
	
}


void getSerialParams(void)
{
	if (newData == 1)
	{
		newData = 0;

		if(serialCode == 0)
		{
			dir_ind = 1;
			serial_ind = 1;
			pulsos_ind = serialPulse;
			printf("Avanzar %d pulsos \n", pulsos_ind);
			printf("Abriendo indice \n");
		}

		if(serialCode == 1)
		{
			dir_ind = 0;
			serial_ind = 1;
			pulsos_ind = serialPulse;
			printf("Cerrando indice \n");
		}

		if(serialCode == 2)
		{
			dir_med = 1;
			serial_med = 1;
			pulsos_med = serialPulse;
			printf("Abriendo med \n");
		}

		if(serialCode == 3)
		{
			dir_med = 0;
			serial_med = 1;
			pulsos_med = serialPulse;
			printf("Cerrando med \n");
		}
		
		if(serialCode == 4)
		{
			dir_anu = 1;
			serial_anu = 1;
			pulsos_anu = serialPulse;
			printf("Abriendo anul \n");
		}

		if(serialCode == 5)
		{
			dir_anu = 0;
			serial_anu = 1;
			pulsos_anu = serialPulse;
			printf("Cerrando anul \n");
		}
		
		if(serialCode == 6)
		{
			fsr_ind = ADC_GetData(FSRpin_ind);
			printf("FSR ind: %d \n", fsr_ind);
			_delay_ms(30);
		}
		
		if(serialCode == 7)
		{
			fsr_med = ADC_GetData(FSRpin_med);
			printf("FSR med: %d \n", fsr_med);
			_delay_ms(30);
		}
		
		if(serialCode == 8)
		{
			fsr_anu = ADC_GetData(FSRpin_anu);
			printf("FSR anu: %d \n", fsr_anu);
			_delay_ms(30);
		}
		
		
		if(serialCode == 9)
		{
			//iniciar_sis = 0;
			//printf("Inicio %d \n", iniciar_sis);
			printf("Pulgar %d \n", pulgar_listo);
			_delay_ms(30);
		}
		
	}
}



void configPWM(void)
{
	//Config PWM0
	TCCR0A=0x03;	// 0000 0011 --- Operación normal, salidas PWM OC0A y OC0B desactivadas, PWM rápido
	TCCR0B=0x02;    // 0000 0010 --- Escalador/8
	
	//Config PWM1
	TCCR1A=0x01;    // 1010 0001 --- Operación normal, salida PWM OC1A desactivada, PWM rápido
	TCCR1B=0X0A;    // 0000 1010 --- Escalador/8
}

void ActivarPWM_Med(uint8_t PWM, uint8_t encendido)
{
	OCR0A=PWM;				// Definir ancho de pulso
	
	if (encendido==1)		// Si el motor debe encenderse o continuar encendido:
	{
		rel_medio = 0;
		TCCR0A |= (1<<7);   // TCCR0A = 0b10x00011 ----  10x0 0011  Modo no invertido, salida OC0A activada, PWM rápido
		printf("Med on \n");
	}
	else                    // Si no (el motor debe apagarse):
	{
		TCCR0A &=~ (1<<7); // TCCR0A = 0b00x00011 ----  00x0 0011  Operación normal, salida PWM OC0A desactivada, PWM rápido
		rel_medio = count_enc_med;
		printf("Med off \n");
	}
}

void ActivarPWM_Ind(uint8_t PWM, uint8_t encendido)
{
	OCR0B=PWM;				// Definir ancho de pulso
	
	if (encendido==1)		// Si el motor debe encenderse o continuar encendido:
	{
		rel_indice = 0;
		TCCR0A |= (1<<5);   // TCCR0A = 0b10x00011 ----  10x0 0011  Modo no invertido, salida OC0B activada, PWM rápido
		printf("Ind on \n");
	}
	else                    // Si no (el motor debe apagarse):
	{
		TCCR0A &=~ (1<<5); // TCCR0A = 0b00x00011 ----  00x0 0011  Operación normal, salida PWM OC0B desactivada, PWM rápido
		rel_indice = count_enc_ind;
		printf("Ind off \n");
	}
}

void ActivarPWM_Anu(uint8_t PWM, uint8_t encendido)
{
	OCR1A=PWM;				// Definir ancho de pulso
	
	if (encendido==1)		// Si el motor debe encenderse o continuar encendido:
	{
		rel_anular = 0;
		TCCR1A |= (1<<7);   // TCCR0A = 0b10x00011 ----  10x0 0011  Modo no invertido, salida OC1A activada, PWM rápido
		printf("Anu on \n");
	}
	else                    // Si no (el motor debe apagarse):
	{
		TCCR1A &=~ (1<<7); // TCCR0A = 0b00x00011 ----  00x0 0011  Operación normal, salida PWM OC1A desactivada, PWM rápido
		rel_anular = count_enc_anu;
		printf("Anu off \n");
	}
}


void configADC(void)
{
	ADMUX = 0b01000000;		// AVcc
	ADCSRA = 0b0000111;		// ADC apagado, preescalador=128 --> 16MHz/64=125KHz
	ADCSRB = 0;				// Modo de carrera libre
	//DIDR0 |= ((1<<3)|(1<<4));		// Entrada digital (PC3/ADC3) y (PC4/ADC4) deshabilitada
}

int ADC_GetData(int canalADC)
{
	ADMUX &=~  0x0F;		// Limpiar selección de puertos ADC
	ADMUX |=  canalADC;		// Selección de puerto ADC
	ADCSRA |= (1<<ADEN);	// Habilita ADC
	//	_delay_us(10);			// Tiempo de espera
	ADCSRA |= (1<<ADSC);	// Inicia conversión AD
	while (ADCSRA&(1<<ADSC));	// Esperar mientras se realiza la conversión
	ADCSRA &=~ (1<<ADEN);	// Desactiva el ADC
	return ADC;				// Retorna lectura del ADC
}



void configInterrupt(void)
{
	EICRA |= (1<<ISC00);    // INT0 configurado = Flanco de subida (11) // Cualquier flanco (01) // ((1<<ISC01)|(1<<ISC00))
	EICRA &=~ (1<<ISC01);
	EIMSK |= (1<<INT0);		// INT0 activado
	
	EICRA |= (1<<ISC10);    // INT1 configurado = Flanco de subida (11) // Cualquier flanco (01) // ((1<<ISC11)|(1<<ISC10))
	EICRA &=~ (1<<ISC11);
	EIMSK |= (1<<INT1);		// INT1 activado
	
	PCICR |= (1 << PCIE2);		 // Activa PCIE2 para activar grupo PCMSK2
	PCMSK2 |= ((1 << PCINT20));  // Configura PCINT20 (Encoder meñique) para activar interrupciones PCINTx
	
	//PCICR |= (1 << PCIE1);		 // Activa PCIE1 para activar grupo PCMSK1
	//PCMSK1 |= ((1 << PCINT11));  // Configura PCINT11 (pulgar_listo) para activar interrupciones PCINTx
	
}

void calibrar(void)
{
	printf("Iniciar calib \n");
	fsr_ind = ADC_GetData(FSRpin_ind);
	pulsos_ind = 3000; // Para evitar que interrupción de encoder apague pwm
	if (fsr_ind <= 40)
	{
		dir_ind = 0;
		pwm_ind = 120;
		onoff_ind = 1;
		PORTD &= ~(1<<dirpin_ind);
		_delay_ms(10);
		ActivarPWM_Ind(pwm_ind, onoff_ind);
		_delay_ms(10);
		while((ADC_GetData(FSRpin_ind)) <= 40 )
		{

		}
		ActivarPWM_Ind(0, 0);
		pwm_ind = 0;
		onoff_ind = 0;
		count_enc_ind = 0;
		pulsos_ind = 0;         // Para evitar que interrupción de encoder apague pwm - resetear a 0
		printf("Indice listo \n");
		
	}
	
	fsr_med = ADC_GetData(FSRpin_med);
	pulsos_med = 3000; // Para evitar que interrupción apague pwm
	if (fsr_med <= 40)
	{
		dir_med = 0;
		pwm_med = 130;
		onoff_med = 1;
		PORTB &= ~(1 << dirpin_med);
		ActivarPWM_Med(pwm_med, onoff_med);
		
		while((ADC_GetData(FSRpin_med)) <= 40 )
		{

		}
		ActivarPWM_Med(0, 0);
		pwm_med = 0;
		onoff_med = 0;
		count_enc_med = 0;
		pulsos_med = 0;         // Para evitar que interrupción de encoder apague pwm - resetear a 0
		printf("Medio listo \n");
	
	}
	
	fsr_anu = ADC_GetData(FSRpin_anu);
	pulsos_anu = 3000;         // Para evitar que interrupción de encoder apague pwm
	if (fsr_anu <= 50)
	{
		dir_anu = 0;
		pwm_anu = 120;
		onoff_anu = 1;
		PORTB &= ~(1 << dirpin_anu);
		ActivarPWM_Anu(pwm_anu, onoff_anu);
		
		while(ADC_GetData(FSRpin_anu) <= 50)
		{
			
		}
		
		ActivarPWM_Anu(0, 0);
		pwm_anu = 0;
		onoff_anu = 0;
		count_enc_anu = 0;
		pulsos_anu = 0;         // Para evitar que interrupción de encoder apague pwm - resetear a 0
		printf("Anular listo \n");
		_delay_ms(5);
		printf("IMA listo \n");
		_delay_ms(50);
	}
	
	abs_indice = 0;
	abs_medio = 0;
	abs_anular = 0;
	
	ind_ready = 0;
	med_ready = 0;
	anu_ready = 0;
}


/*
void getIDMov(void)
{
	// Obtiene los estados de los pines de control
	if (((PINB & (1 << PINB3)) >> PINB3) == 1)
	{
		clave_mov[0] = 1;
	}
	else
	{
		clave_mov[0] = 0;
	}
	
	if (((PINB & (1 << PINB4)) >> PINB4) == 1)
	{
		clave_mov[1] = 2;
	}
	else
	{
		clave_mov[1] = 0;
	}
	
	if (((PINB & (1 << PINB5)) >> PINB5) == 1)
	{
		clave_mov[2] = 4;
	}
	else
	{
		clave_mov[2] = 0;
	}
	
	id_mov = clave_mov[0] + clave_mov[1] + clave_mov[2];
	
	printf("ID Mov: %d \n", id_mov);
}
*/

int getIDMov(void)
{
	uint8_t clave_mov[3] = {0, 0 ,0};
	uint8_t id_move = 0;
	
	// Obtiene los estados de los pines de control
	if (((PINB & (1 << PINB3)) >> PINB3) == 1)
	{
		clave_mov[0] = 1;
	}
	else
	{
		clave_mov[0] = 0;
	}
	
	if (((PINB & (1 << PINB4)) >> PINB4) == 1)
	{
		clave_mov[1] = 2;
	}
	else
	{
		clave_mov[1] = 0;
	}
	
	if (((PINB & (1 << PINB5)) >> PINB5) == 1)
	{
		clave_mov[2] = 4;
	}
	else
	{
		clave_mov[2] = 0;
	}
	
	id_move = clave_mov[0] + clave_mov[1] + clave_mov[2];
	printf("ID Mov: %d \n", id_move);

	return(id_move);
}


void getPulsosMov(void)
{
	
	if (abs_indice < indice[id_mov])
	{
		pulsos_ind = indice[id_mov] - abs_indice;
		dir_ind = 1;
		signo_ind = 1;
		pwm_ind = 130;
		onoff_ind = 1;
		PORTD |= (1<<dirpin_ind);	// Dirección del movimiento
	}
	else if (abs_indice > indice[id_mov])
	{
		pulsos_ind = abs(indice[id_mov] - abs_indice);
		dir_ind = 0;
		signo_ind = -1;
		pwm_ind = 140;
		onoff_ind = 1;
		PORTD &= ~(1<<dirpin_ind);		// Dirección del movimiento
	}
	else if (abs_indice == indice[id_mov])
	{
		pulsos_ind = 0;
		dir_ind = 0;
		pwm_ind = 0;
		onoff_ind = 0;
		PORTD &= ~(1<<dirpin_ind);		// Dirección del movimiento
	}
	
	
	if (abs_medio < medio[id_mov])
	{
		pulsos_med = medio[id_mov] - abs_medio;
		dir_med = 1;
		signo_med = 1;
		pwm_med = 140;
		onoff_med = 1;
		PORTB |= (1<<dirpin_med);	// Dirección del movimiento
	}
	else if (abs_medio > medio[id_mov])
	{
		pulsos_med = abs(medio[id_mov] - abs_medio);
		dir_med = 0;
		signo_med = -1;
		pwm_med = 150;
		onoff_med = 1;
		PORTB &= ~(1<<dirpin_med);		// Dirección del movimiento
	}
	else if (abs_medio == medio[id_mov])
	{
		pulsos_med = 0;
		dir_med = 0;
		pwm_med = 0;
		onoff_med = 0;
		PORTB &= ~(1<<dirpin_med);		// Dirección del movimiento
	}
	
	
	if (abs_anular < anular[id_mov])
	{
		pulsos_anu = anular[id_mov] - abs_anular;
		dir_anu = 1;
		signo_anu = 1;
		pwm_anu = 140;
		onoff_anu = 1;
		PORTB |= (1<<dirpin_anu);	// Dirección del movimiento
	}
	else if (abs_anular > anular[id_mov])
	{
		pulsos_anu = abs(anular[id_mov] - abs_anular);
		dir_anu = 0;
		signo_anu = -1;
		pwm_anu = 120;
		onoff_anu = 1;
		PORTB &= ~(1<<dirpin_anu);		// Dirección del movimiento
	}
	else if (abs_anular == anular[id_mov])
	{
		pulsos_anu = 0;
		dir_anu = 0;
		pwm_anu = 0;
		onoff_anu = 0;
		PORTB &= ~(1<<dirpin_anu);		// Dirección del movimiento
	}
	
	printf("PI: %d \n", pulsos_ind);
	_delay_ms(5);
	printf("PM: %d \n", pulsos_med);
	_delay_ms(5);
	printf("PA: %d \n", pulsos_anu);
	_delay_ms(5);
	
	ind_ready = 0;
	med_ready = 0;
	anu_ready = 0;
}


ISR(INT0_vect)
{
	if (count_enc_ind <= pulsos_ind)
	{
		count_enc_ind ++;
	}
	else
	{
		ActivarPWM_Ind(0, 0);
		count_enc_ind = 0;
		onoff_ind = 0;
		pwm_ind = 0;
		ind_ready = 1;
		printf("Ind come: %d \n", ind_ready);
	}
	
}

ISR(INT1_vect)
{
	if (count_enc_med <= pulsos_med)
	{
		count_enc_med ++;
	}
	else
	{
		ActivarPWM_Med(0, 0);
		count_enc_med = 0;
		onoff_med = 0;
		pwm_med = 0;
		med_ready = 1;
		printf("Med come \n");
		
	}
}



ISR (PCINT2_vect)
{
	uint8_t changedbits;

	changedbits = PIND ^ portdhistory;
	portdhistory = PIND;

	if(changedbits & (1 << PIND4))
	{
		if (count_enc_anu <= pulsos_anu)
		{
			count_enc_anu ++;
		}
		else
		{
			ActivarPWM_Anu(0, 0);
			count_enc_anu = 0;
			onoff_anu = 0;
			pwm_anu = 0;
			anu_ready = 1;
			printf("Anu come \n");
			
		}
	}
	
}


/*ISR (PCINT1_vect)
{
	uint8_t changedbits;

	changedbits = PINC ^ portchistory;
	portchistory = PINC;

	if(changedbits & (1 << PINC3))
	{
		if ( (PINC & (1 << PINC3)) == (1 << PINC3) )
		{
			//pulgar_listo = 1;
			count_event ++;
			printf("Evento: %d \n", count_event);
		}
		else
		{
			printf("Pulgar 0 \n");
		}
	}
	
}
*/


