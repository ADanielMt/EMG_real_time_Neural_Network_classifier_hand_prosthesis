/*
 * Micro PLM.c
 *
 * Created: 21/05/2021 06:42:43 p. m.
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
void ActivarPWM_Pul(uint8_t PWM, uint8_t encendido);
void ActivarPWM_Lat(uint8_t PWM, uint8_t encendido);
void ActivarPWM_Men(uint8_t PWM, uint8_t encendido);
void configADC(void);
int ADC_GetData(int canalADC);
void calibrar(void);
void configInterrupt(void);
void getSerialParams(void);
//void getIDMov(void);
int getIDMov(void);
void getPulsosMov(void);
void pulgarCero(void);


// Variables para comunicación serial
uint8_t numChars = 32;
char RxBuffer[32];
volatile unsigned char RxContador = 0;
uint8_t serialCode = 0;
int serialPulse = 0;
int newData = 0;
int serial_pul = 0;
int serial_lat = 0;
int serial_men = 0;
uint8_t recibiendo = 0;
uint8_t j = 0;

// Variables para movimientos y comunicación con Micro ima
uint8_t iniciar_sis = 0;	// Si 0, indica que el sistema debe calibrarse, si 1, indica que está calibrado para iniciar
uint8_t count_event = 0;
uint8_t enc_calib = 0;		// 1: Los encoders están calibrados, 0: lo contrario.
uint8_t pin_pul_listo = 4;  // Pin del puerto C asignado al pulgar listo.   
uint8_t pulgar_listo = 0;	// Salida: Indica que el pulgar está en posición de inicio y calibrado a 0
uint8_t ima_listo = 0;		// Entrada: Indica que el indice, medio y anular están en posición deseada
uint8_t pin_ima_listo = 5;
uint8_t plm_listo = 0;		// Entrada: Indica que el pulgar, lateral y meñique están en posición deseada
uint8_t clave_mov[3] = {0, 0 ,0};
uint8_t id_mov = 0;			// Entrada: ID del movimiento enviado por la raspberry
uint8_t id_mov_ant = 0;
uint8_t new_mov = 0;		// Salida: Indica a la raspberry que ya puede enviar un nuevo movimiento
int mov[8] = {0, 0, 0, 0, 0, 0, 0 , 0}; // Entrada: Lee movimiento detectado por la Raspberry

// Variables para motores
uint8_t onoff_lat = 0;
uint8_t onoff_pul = 0;
uint8_t onoff_men = 0;
uint8_t pwm_lat = 0;
uint8_t pwm_pul = 0;
uint8_t pwm_men = 0;
uint8_t dir_lat = 0;
uint8_t dir_pul = 0;
uint8_t dir_men = 0;
uint8_t dirpin_lat = 0;
uint8_t dirpin_pul = 7;
uint8_t dirpin_men = 2;
uint8_t pul_ready = 0;
uint8_t lat_ready = 0;
uint8_t men_ready = 0;
uint8_t PWM_mot=0;

// Variables para encoders
int count_enc_lat = 0;
int count_enc_pul = 0;
int count_enc_men = 0;
int abs_lateral = 0;
int abs_pulgar = 0;
int abs_menique = 0;
int rel_lateral = 0;
int rel_pulgar = 0;
int rel_menique = 0;
int signo_lat = 0;
int signo_pul = 0;
int signo_men = 0;
int limite_lat = 3000;
int limite_pul = 300;
int limite_men = 800;

// Variables para Interrupciones PCINT
volatile uint8_t portdhistory = 0xFF;     // Por default en alto por la resistencia pull-up
volatile uint8_t portchistory = 0xFF;     // Por default en alto por la resistencia pull-up

// Pines ADC: FSR - IR, O: FC
uint8_t FSRpin_pul = 0;
uint8_t IRpin = 1;
uint8_t FSRpin_men = 2;
uint8_t FC_pin = 3;

// Variables para almacenar valores de FSR, IR y FC
int fsr_pul = 0;
int fsr_men = 0;
int ir_pul = 0;
int fc_lat = 0;

// Variables para guardar posiciones de cada dedo
//                  (0)      (1)        (2)     (3)    (4)     (5)       (6)           (7)          (8)
//               descanso, cilindro, lateral, pinza, gancho, cuernos, esférico, índice extendido, calibrar
int pulgar[9] =  {  120,	150,	  470,     435,    0,	   520,     300,          0,		     0  };  // 200 225
int lateral[9] = { 1500,   2500,	    0,    1600,    0,	  2500,    2500,	      0,	         0  };
int menique[9] = {  700,	400,	    0,       0,    0,	   900,     400,	      0,		     0  };

 
// Variables de apoyo
int pulsos_pul = 0;
int pulsos_lat = 0;
int pulsos_men = 0;

// Variables para verificación de lectura de movimiento
uint8_t lectura[3] = {0, 0 ,0};
uint8_t moda = 0;


int main(void)
{
    cli();
	
	DDRC = 0x10;   // 0001 0000 (PC0-FSR_pul, PC1-IR_pul, PC2-FSR_men),(PC3-FC_lat), (PC4-pulgar_list), (PC5-ima_listo)
	DDRD = 0xE2;   // 1110 0010 (PD0-PD1-UART), (PD2-Enc_pul, PD3-Enc_lat, PD4-Enc_men), (PD5-PWM_pul, PD6-PWM_lat), (PD7-Dir_pul)
	DDRB = 0x07;   // 0000 0111 (PB0-Dir_lat, PB2-Dir_men), (PB1-PWM_men), (PB3:5-Control), (PB7-PB7-Clock)
	
	PORTC |= (1<<3);	// Activar resistencia pull-up para PC3
	PORTC |= (1<<5);	// Activar resistencia pull-up para PC5
	PORTB |= ((1<<3) | (1<<4) | (1<<5));	// Activar resistencia pull-up para PB3, PB4 y PB5
	PORTC &= ~(1<<4);	// Poner en bajo PC4
	PORTD |= ((1<<2) | (1<<3) | (1<<4));	// Activar resistencia pull-up para PD2, PD3 y PD4
	PORTD &= ~((1<<5) | (1<<6) | (1<<7));	// Poner en bajo PD5, PD6, PD7
	PORTB &= ~((1<<0) | (1<<1) | (1<<2));	// Poner en bajo PB0, PB1, PB2
	
	configADC();   // Configura ADC
	configPWM();   // Configura PWM
	configInterrupt();   // Configura interrupciones de encoders y FC
    UART_init();
	

	
    sei();
    
	
    while (1)
    {
		_delay_ms(250);
		_delay_ms(250);
		getSerialParams();
		_delay_ms(250);
		
		// Paso 1: Calibración inicial
		
		if(iniciar_sis == 0 && count_event == 0)
		{
			for (int t=0; t<20; t++)
			{
				_delay_ms(250);			// Espera 2 segundos para iniciar
			}
			
			//PORTC |= (1<<pin_pul_listo);	// Enviar señal para inicializar interrupciones del puerto C
			for (int t=0; t<4; t++)
			{
				_delay_ms(250);				// Espera 
			}
			//PORTC &= ~ (1<<pin_pul_listo);  // Poner en bajo señal enviada 
			
			printf("Hola usuario \n ");
			_delay_ms(250);
			printf("Vamos a calibrar \n");
			_delay_ms(250);
			calibrar();
			
			iniciar_sis = 1;
			id_mov_ant = 8;
			_delay_ms(250);
			
			printf("W 1 \n");
			PORTC &= ~(1<<pin_pul_listo);       // Poner pin en cero 
			while (count_event != 1)
			{
				if ( ((PINC & (1 << pin_pul_listo)) == 0) && ((PINC & (1 << pin_ima_listo)) != 0) )
				{
					count_event = 1;
				}
			}
			
			// Si evento == 1, pasar a la siguiente sección
		} 
		
	//	printf("E1 \n");
	// Fin de paso 1
	
	// Paso 2: Detección de movimientos
	// if(count_event == 1)
		while(count_event == 1)    // While en prueba
		{
			printf("In 1 \n");
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
			}
			else if (((id_mov_ant == 1 || id_mov_ant == 3 || id_mov_ant == 4) && (id_mov != 0)) || ((id_mov == 1 || id_mov == 3 || id_mov == 4) && (id_mov_ant != 0)) )
			{
				// Si la posición anterior es de agarre, no hacer nada y salir del if principal 
				// para esperar a que el usuario indique posición de descanso para liberar objeto.
				// Ó si la posición acutual es agarre pero la posición anterior no es descanso,
				// salir del if principal para esperar a que el usuario indicque descanso y luego agarre.
			}
			else
			{
				printf("In m \n");
				_delay_ms(5);
				id_mov_ant = id_mov;
				
				ActivarPWM_Pul(0,0);
				ActivarPWM_Lat(0,0);
				ActivarPWM_Men(0,0);
				pulgarCero();		// Mover pulgar a cero para evitar que choque con los demás dedos
				getPulsosMov();		// Obtener el número de pulsos para alcanzar la posición
				
				PORTC |= (1<<pin_pul_listo);	// Poner pin en alto

				while (count_event != 2) // While en prueba
				{
					if ( ((PINC & (1 << pin_pul_listo)) != 0) && ((PINC & (1 << pin_ima_listo)) == 0) )
					{
						count_event = 2;
					}
				}
				
				printf("If \n");
				_delay_ms(5);
				
				/////////////////////////////////////////////////////////////////////////////////////////
				if (id_mov == 0)			// Rutina para movimiento de descanso
				{
					printf("Mov 0 \n");
					_delay_ms(10);
					if (pulsos_men != 0)
					{
						ActivarPWM_Men(pwm_men, onoff_men);
						while(men_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						abs_menique = abs_menique + (signo_men * rel_menique);
						printf("Men en pos: %d \n", abs_menique);
					}
					
					if (pulsos_lat != 0)
					{
						ActivarPWM_Lat(pwm_lat, onoff_lat);
						while(lat_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						abs_lateral = abs_lateral + (signo_lat * rel_lateral);
						printf("Lat en pos: %d \n", abs_lateral);
					}
					
					if (pulsos_pul != 0)
					{
						ActivarPWM_Pul(pwm_pul, onoff_pul);
						while(pul_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						abs_pulgar = abs_pulgar + (signo_pul * rel_pulgar);
						printf("Pul en pos: %d \n", abs_pulgar);
					}
					
					printf("PLM listo \n");
					//PORTC |= (1<<pin_pul_listo);	// Avisar a Micro IMA que Micro PLM está en posición deseada
					//plm_listo = 1;					// Bandera: Indica que Micro PLM está en posición deseada
				}
				////////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 1)			// Rutina para agarre cilíndrico (de fuerza) ///////////////
				{		
					if (pulsos_men != 0)				// Si la posición actual es distinta de la deseada
					{
						ActivarPWM_Men(pwm_men, onoff_men);
						while(men_ready != 1)		// Mientras no alcance la posición guardada (límite)
						{
							if ((ADC_GetData(FSRpin_men)) > 100)  // Evaluar FSR, si FSR en contacto con objeto:
							{									 // apagar motor e indicar que está en posición
								ActivarPWM_Men(0,0);
								count_enc_men = 0;
								onoff_men = 0;
								pwm_men = 0;
								men_ready = 1;	
								printf("Men come \n");
							}
						}
						
						abs_menique = abs_menique + (signo_men * rel_menique);
						printf("Men en pos: %d \n", abs_menique);
					}
					
					if (pulsos_lat != 0)
					{
						ActivarPWM_Lat(pwm_lat, onoff_lat);
						while(lat_ready != 1)		// Mientras no alcance la posición guardada (límite)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						
						abs_lateral = abs_lateral + (signo_lat * rel_lateral);
						printf("Lat en pos: %d \n", abs_lateral);
					}
					
					if (pulsos_pul != 0)
					{
						ActivarPWM_Pul(pwm_pul, onoff_pul);
						while(pul_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						
						abs_pulgar = abs_pulgar + (signo_pul * rel_pulgar);
						printf("Pul en pos: %d \n", abs_pulgar); 
					}
					
					printf("PLM listo \n");
					//PORTC |= (1<<pin_pul_listo);	// Avisar a Micro IMA que Micro PLM está en posición deseada
					//plm_listo = 1;					// Bandera: Indica que Micro PLM está en posición deseada
				}
				
				////////////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 2)			// Rutina para puño/agarre lateral ///////////////
				{
					if (pulsos_men != 0)				// Si la posición actual es distinta de la deseada
					{
						pulsos_men = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0 
						ActivarPWM_Men(pwm_men, onoff_men);
						while(ADC_GetData(FSRpin_men) < 35)		// Mientras los sensores no toquen la palma
						{
						}
						ActivarPWM_Men(0,0);
						count_enc_men = 0;
						onoff_men = 0;
						pwm_men = 0;
						men_ready = 1;
						printf("Men come \n");
						
						abs_menique = 0; // abs_menique + (signo_men * rel_menique);
						printf("Men en pos: %d \n", abs_menique);
					}
			
					
					if (pulsos_lat != 0)		// Si la posición actual es distinta de la deseada
					{
						pulsos_lat = 5000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Lat(pwm_lat, onoff_lat);
						while(lat_ready != 1)		// Mientras no alcance la posición guardada (límite)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						
						abs_lateral = 0; // abs_lateral + (signo_lat * rel_lateral);
						printf("Lat en pos: %d \n", abs_lateral);
					}
					
					
					if (pulsos_pul != 0)		// Si la posición actual es distinta de la deseada
					{
						ActivarPWM_Pul(pwm_pul, onoff_pul);
						while(pul_ready != 1)
						{
							if (ADC_GetData(FSRpin_pul) > 70)
							{
								ActivarPWM_Pul(0,0);
								count_enc_pul = 0;
								onoff_pul = 0;
								pwm_pul = 0;
								pul_ready = 1;
								printf("Pul come \n");
							}
						}
						
						abs_pulgar = abs_pulgar + (signo_pul * rel_pulgar);
						printf("Pul en pos: %d \n", abs_pulgar);
					}
					
					
					printf("PLM listo \n");
					//PORTC |= (1<<pin_pul_listo);	// Avisar a Micro IMA que Micro PLM está en posición deseada
					//plm_listo = 1;					// Bandera: Indica que Micro PLM está en posición deseada
				}
				
				////////////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 3)			// Rutina para pinza ///////////////
				{
					if (pulsos_men != 0)				// Si la posición actual es distinta de la deseada
					{
						pulsos_men = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0 
						ActivarPWM_Men(pwm_men, onoff_men);
						while(ADC_GetData(FSRpin_men) < 35)		// Mientras no toque la palma
						{
						}
						ActivarPWM_Men(0,0);
						count_enc_men = 0;
						onoff_men = 0;
						pwm_men = 0;
						men_ready = 1;
						printf("Men come \n");
						
						abs_menique = 0; // abs_menique + (signo_men * rel_menique);
						printf("Men en pos: %d \n", abs_menique);
					}
					
					if (pulsos_lat != 0)		// Si la posición actual es distinta de la deseada
					{
						ActivarPWM_Lat(pwm_lat, onoff_lat);
						while(lat_ready != 1)		// Mientras no alcance la posición guardada (límite)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						
						abs_lateral = abs_lateral + (signo_lat * rel_lateral);
						printf("Lat en pos: %d \n", abs_lateral);
					}
					
					if (pulsos_pul != 0)		// Si la posición actual es distinta de la deseada
					{
						ActivarPWM_Pul(pwm_pul, onoff_pul);
						while(pul_ready != 1)
						{		
							if (ADC_GetData(FSRpin_pul) > 70)
							{
								ActivarPWM_Pul(0,0);
								count_enc_pul = 0;
								onoff_pul = 0;
								pwm_pul = 0;
								pul_ready = 1;
								printf("Pul come \n");
							}
						}
						
						abs_pulgar = abs_pulgar + (signo_pul * rel_pulgar);
						printf("Pul en pos: %d \n", abs_pulgar);
					}
					
					printf("PLM listo \n");
					//PORTC |= (1<<pin_pul_listo);	// Avisar a Micro IMA que Micro PLM está en posición deseada
					//plm_listo = 1;					// Bandera: Indica que Micro PLM está en posición deseada
				}
				
				////////////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 4)			// Rutina para like ///////////////
				{
					if (pulsos_men != 0)				// Si la posición actual es distinta de la deseada
					{
						//pulsos_men = 1000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Men(pwm_men, onoff_men);
						while(men_ready != 1)		// Mientras no esté en posición límite
						{
							if (ADC_GetData(FSRpin_men) > 30)
							{
								ActivarPWM_Men(0,0);
								count_enc_men = 0;
								onoff_men = 0;
								pwm_men = 0;
								men_ready = 1;
								printf("Men come \n");
							}
						}
						
						abs_menique = abs_menique + (signo_men * rel_menique);
						printf("Men en pos: %d \n", abs_menique);
					}
					
					if (pulsos_lat != 0)		// Si la posición actual es distinta de la deseada
					{
						pulsos_lat = 5000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Lat(pwm_lat, onoff_lat);
						while(lat_ready != 1)		// Mientras no alcance la posición guardada (límite) || (PINC & (1 << PINC3)) == 0
						{		// Espera indicación de motor listo para salir del loop
							_delay_us(30);
						}
						
						abs_lateral = 0; // abs_lateral + (signo_lat * rel_lateral);
						printf("Lat en pos: %d \n", abs_lateral);
					}
					
					if (pulsos_pul != 0)		// Si la posición actual es distinta de la deseada
					{
						pulsos_pul = 2000; 
						ActivarPWM_Pul(pwm_pul, onoff_pul);
						
						while(ADC_GetData(IRpin) < 245)
						{		// Espera que lo detecte el sensor IR para salir de loop
						}
						
						ActivarPWM_Pul(0,0);
						count_enc_pul = 0;
						onoff_pul = 0;
						pwm_pul = 0;
						pul_ready = 1;
						printf("Pul come \n");
						
						abs_pulgar = 0;  //abs_pulgar + (signo_pul * rel_pulgar);
						printf("Pul en pos: %d \n", abs_pulgar);
					}
					
					printf("PLM listo \n");
					//PORTC |= (1<<pin_pul_listo);	// Avisar a Micro IMA que Micro PLM está en posición deseada
					//plm_listo = 1;					// Bandera: Indica que Micro PLM está en posición deseada
				}
				
				/////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 5)			// Rutina para movimiento de cuernos
				{
					if (pulsos_men != 0)
					{
						ActivarPWM_Men(pwm_men, onoff_men);
						while(men_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						abs_menique = abs_menique + (signo_men * rel_menique);
						printf("Men en pos: %d \n", abs_menique);
					}
					
					if (pulsos_lat != 0)
					{
						ActivarPWM_Lat(pwm_lat, onoff_lat);
						while(lat_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						abs_lateral = abs_lateral + (signo_lat * rel_lateral);
						printf("Lat en pos: %d \n", abs_lateral);
					}
					
					if (pulsos_pul != 0)
					{
						ActivarPWM_Pul(pwm_pul, onoff_pul);
						while(pul_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						abs_pulgar = abs_pulgar + (signo_pul * rel_pulgar);
						printf("Pul en pos: %d \n", abs_pulgar);
					}
					
					printf("PLM listo \n");
					//PORTC |= (1<<pin_pul_listo);	// Avisar a Micro IMA que Micro PLM está en posición deseada
					//plm_listo = 1;					// Bandera: Indica que Micro PLM está en posición deseada
				}
				
				/////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 6)			// Rutina para movimiento de esfera (palma fuera)
				{
					printf("In 6 \n");
					_delay_ms(5);
					if (pulsos_men != 0)
					{
						printf("In men \n");
						ActivarPWM_Men(pwm_men, onoff_men);
						while(men_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							//printf("M: %d \n", men_ready);
							_delay_ms(20);
						}
						abs_menique = abs_menique + (signo_men * rel_menique);
						printf("Men en pos: %d \n", abs_menique);
						_delay_ms(5);
					}
					
					if (pulsos_lat != 0)
					{
						printf("In lat \n");
						ActivarPWM_Lat(pwm_lat, onoff_lat);
						while(lat_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							//printf("L: %d \n", lat_ready);
							_delay_ms(20);
						}
						abs_lateral = abs_lateral + (signo_lat * rel_lateral);
						printf("Lat en pos: %d \n", abs_lateral);
						_delay_ms(5);
					}
					
					if (pulsos_pul != 0)
					{
						printf("In pul \n");
						ActivarPWM_Pul(pwm_pul, onoff_pul);
						while(pul_ready != 1)
						{		// Espera indicación de motor listo para salir del loop
							//printf("P: %d \n", pul_ready);
							_delay_ms(20);
						}
						abs_pulgar = abs_pulgar + (signo_pul * rel_pulgar);
						printf("Pul en pos: %d \n", abs_pulgar);
						_delay_ms(5);
					}
					
					printf("PLM listo \n");
					//_delay_ms(5);
					//PORTC |= (1<<pin_pul_listo);	// Avisar a Micro IMA que Micro PLM está en posición deseada
					//_delay_ms(200);
					//plm_listo = 1;					// Bandera: Indica que Micro PLM está en posición deseada
				}
				
				/////////////////////////////////////////////////////////////////////////////////////////
				else if (id_mov == 7)			// Rutina para movimiento de pistola
				{
					if (pulsos_men != 0)
					{
						pulsos_men = 2000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Men(pwm_men, onoff_men);
						while(ADC_GetData(FSRpin_men)<30)		// Mientras no toque la palma
						{
						}
						ActivarPWM_Men(0,0);
						count_enc_men = 0;
						onoff_men = 0;
						pwm_men = 0;
						men_ready = 1;
						printf("Men come \n");
						
						abs_menique = 0; // abs_menique + (signo_men * rel_menique);
						printf("Men en pos: %d \n", abs_menique);
					}
					
					if (pulsos_lat != 0)
					{
						pulsos_lat = 5000;	// Para evitar que interrupción de encoder apague motor antes de llegar a 0
						ActivarPWM_Lat(pwm_lat, onoff_lat);
						while(lat_ready != 1)		// Mientras no alcance la posición guardada (límite)
						{		// Espera indicación de motor listo para salir del loop
							_delay_ms(20);
						}
						
						abs_lateral = 0; // abs_lateral + (signo_lat * rel_lateral);
						printf("Lat en pos: %d \n", abs_lateral);
					}
					
					if (pulsos_pul != 0)
					{
						pulsos_pul = 2000;
						ActivarPWM_Pul(pwm_pul, onoff_pul);
						
						while(ADC_GetData(IRpin) < 245)
						{		// Espera que lo detecte el sensor IR para salir de loop
						}
						
						ActivarPWM_Pul(0,0);
						count_enc_pul = 0;
						onoff_pul = 0;
						pwm_pul = 0;
						pul_ready = 1;
						printf("Pul come \n");
						
						abs_pulgar = 0;  //abs_pulgar + (signo_pul * rel_pulgar);
						printf("Pul en pos: %d \n", abs_pulgar);
					}
					
					printf("PLM listo \n");
					//PORTC |= (1<<pin_pul_listo);	// Avisar a Micro IMA que Micro PLM está en posición deseada
					//plm_listo = 1;					// Bandera: Indica que Micro PLM está en posición deseada
				}
				
						
				//////////////////// Checar que IMA y PLM estén en posición deseada ///////////////////////////////
				//_delay_ms(200);					// Esperar
				//PORTC &= ~ (1<<pin_pul_listo);  // Poner en bajo señal enviada
				pulsos_pul = 0;
				pulsos_lat = 0;
				pulsos_men = 0;
				plm_listo = 1;
				
				//////////////////// Sección en prueba ///////////////////////////////
				////////// Reporte de posición de motores y fuerza aplicada //////////
				
				printf("Pulgar: %d \n", abs_pulgar);
				printf("Lateral: %d \n", abs_lateral);
				printf("Menique: %d \n", abs_menique);
				
				printf("Fuerza Pul: %d \n", ADC_GetData(FSRpin_pul));
				printf("Fuerza Men: %d \n", ADC_GetData(FSRpin_men));
				
				////////////////////// Fin de reporte ////////////////////////////////
				//////////////////////////////////////////////////////////////////////
				
				PORTC &= ~ (1<<pin_pul_listo);  // Poner pin en bajo
				
				while (count_event != 3)		// Mientras IMA no esté en posición, no hacer nada
				{
					if ( ((PINC & (1 << pin_pul_listo)) == 0) && ((PINC & (1 << pin_ima_listo)) != 0) )
					{
						count_event = 3;
					}
				}
				
				ima_listo = 1;					// Cuando IMA esté en posición (count_event = 2), poner bandera en 1
				printf("Listos \n");
				_delay_ms(5);
				
				if (plm_listo == 1 && ima_listo == 1)	// Si IMA y PLM están en posición, hacer count_event=1
				{										// para que pueda volver a leer señales de control
					count_event = 1;
					pul_ready = 0;
					lat_ready = 0;
					men_ready = 0;
					plm_listo = 0;
					ima_listo = 0;
				}
				
			}
			
			printf("Wait new \n");
			getSerialParams();   // En prueba
		}
		
		// Fin de paso 2
		
		
		if(serial_pul == 1)
		{
			if (serialCode == 0)
			{
				pwm_pul = 100;
				onoff_pul = 1;
				//dir_pul = 1;
				PORTD |= (1<<dirpin_pul);
				//PORTC &= ~(1 << 2);
				printf("Abrir pulgar \n");
				serial_pul = 0;
				ActivarPWM_Pul(pwm_pul, onoff_pul);
				_delay_ms(200);
			}
			else if (serialCode == 1)
			{
				pwm_pul = 100;
				onoff_pul = 1;
				//dir_pul = 1;
				//PORTD |= (1<<pwmpin_pul);
				PORTD &= ~(1 << dirpin_pul);
				printf("Cerrar pulgar \n");
				serial_pul = 0;
				ActivarPWM_Pul(pwm_pul, onoff_pul);
				_delay_ms(200);
			}
			
		}

		if(serial_lat == 1)
		{
			if (serialCode == 2)
			{
				pwm_lat = 100;
				onoff_lat = 1;
				//dir_lat = 1;
				PORTB |= (1<<dirpin_lat);
				printf("Abrir lat \n");
				serial_lat = 0;
				ActivarPWM_Lat(pwm_lat, onoff_lat);
				_delay_ms(200);
			}
			else if (serialCode == 3)
			{
				pwm_lat = 100;
				onoff_lat = 1;
				//dir_lat = 1;
				//PORTB |= (1<<pwmpin_lat);
				PORTB &= ~(1 << dirpin_lat);
				printf("Cerrar lat \n");
				serial_lat = 0;
				ActivarPWM_Lat(pwm_lat, onoff_lat);
				_delay_ms(200);
			}
		}
		
		if(serial_men == 1)
		{
			if (serialCode == 4)
			{
				pwm_men = 100;
				onoff_men = 1;
				//dir_men = 1;
				PORTB |= (1<<dirpin_men);
				printf("Abrir men \n");
				serial_men = 0;
				ActivarPWM_Men(pwm_men, onoff_men);
				_delay_ms(200);
			}
			else if (serialCode == 5)
			{
				pwm_men = 100;
				onoff_men = 1;
				//dir_men = 1;
				//PORTB |= (1<<pwmpin_men);
				PORTB &= ~(1 << dirpin_men);
				printf("Cerrar men \n");
				serial_men = 0;
				ActivarPWM_Men(pwm_men, onoff_men);
				_delay_ms(200);
			}
		}
		
		//printf("\n N pulsos: %d \n", serialPulse);
		//printf("Code: %d \n", serialCode);
		//printf("Count: %d \n", RxContador);
		_delay_ms(240);

    }
	
}



ISR(USART_RX_vect){
	
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
			dir_pul = 1;
			serial_pul = 1;
			pulsos_pul = serialPulse;
			printf("Avanzar %d pulsos \n", pulsos_pul);
			printf("Abriendo pulgar \n");
		}

		if(serialCode == 1)
		{
			dir_pul = 0;
			serial_pul = 1;
			pulsos_pul = serialPulse;
			printf("Cerrando pulgar \n");
		}

		if(serialCode == 2)
		{
			dir_lat = 1;
			serial_lat = 1;
			pulsos_lat = serialPulse;
			printf("Abriendo lat \n");
		}

		if(serialCode == 3)
		{
			dir_lat = 0;
			serial_lat = 1;
			pulsos_lat = serialPulse;
			printf("Cerrando lat \n");
		}
		
		if(serialCode == 4)
		{
			dir_men = 1;
			serial_men = 1;
			pulsos_men = serialPulse;
			printf("Abriendo men \n");
		}

		if(serialCode == 5)
		{
			dir_men = 0;
			serial_men = 1;
			pulsos_men = serialPulse;
			printf("Cerrando men \n");
		}
		
		if(serialCode == 6)
		{
			fsr_pul = ADC_GetData(FSRpin_pul);
			printf("FSR pul: %d \n", fsr_pul);
			_delay_ms(30);
		}
		
		if(serialCode == 7)
		{
			fsr_men = ADC_GetData(FSRpin_men);
			printf("FSR men: %d \n", fsr_men);
			_delay_ms(30);
		}
		
		if(serialCode == 8)
		{
			ir_pul = ADC_GetData(IRpin);
			printf("IR pul: %d \n", ir_pul);
			_delay_ms(30);
		}
		
		if(serialCode == 9)
		{
			iniciar_sis = 0;
			printf("Inicio %d \n", iniciar_sis);
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

void ActivarPWM_Lat(uint8_t PWM, uint8_t encendido)
{
	OCR0A=PWM;				// Definir ancho de pulso
	
	if (encendido==1)		// Si el motor debe encenderse o continuar encendido:
	{
		rel_lateral = 0; 
		TCCR0A |= (1<<7);   // TCCR0A = 0b10x00011 ----  10x0 0011  Modo no invertido, salida OC0A activada, PWM rápido
	    printf("Lat on \n");
	}
	else                    // Si no (el motor debe apagarse):
	{
		TCCR0A &=~ (1<<7); // TCCR0A = 0b00x00011 ----  00x0 0011  Operación normal, salida PWM OC0A desactivada, PWM rápido
		rel_lateral = count_enc_lat;
		printf("Lat off \n");
	}
}

void ActivarPWM_Pul(uint8_t PWM, uint8_t encendido)
{
	OCR0B=PWM;				// Definir ancho de pulso
	
	if (encendido==1)		// Si el motor debe encenderse o continuar encendido:
	{
		rel_pulgar = 0;
		TCCR0A |= (1<<5);   // TCCR0A = 0b10x00011 ----  10x0 0011  Modo no invertido, salida OC0B activada, PWM rápido
		printf("Pul on \n");
	}
	else                    // Si no (el motor debe apagarse):
	{
		TCCR0A &=~ (1<<5); // TCCR0A = 0b00x00011 ----  00x0 0011  Operación normal, salida PWM OC0B desactivada, PWM rápido
		rel_pulgar = count_enc_pul;
		printf("Pul off \n");
	}
}

void ActivarPWM_Men(uint8_t PWM, uint8_t encendido)
{
	OCR1A=PWM;				// Definir ancho de pulso
	
	if (encendido==1)		// Si el motor debe encenderse o continuar encendido:
	{
		rel_menique = 0;
		TCCR1A |= (1<<7);   // TCCR0A = 0b10x00011 ----  10x0 0011  Modo no invertido, salida OC1A activada, PWM rápido
	    printf("Men on \n");
	}
	else                    // Si no (el motor debe apagarse):
	{
		TCCR1A &=~ (1<<7); // TCCR0A = 0b00x00011 ----  00x0 0011  Operación normal, salida PWM OC1A desactivada, PWM rápido
		rel_menique = count_enc_men;
		printf("Men off \n");
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
	
	PCICR |= (1 << PCIE1);		 // Activa PCIE1 para activar grupo PCMSK1
	PCMSK1 |= ((1 << PCINT11));  // Configura PCINT11 (FC) para activar interrupciones PCINTx
	//PCMSK1 |= ((1 << PCINT13));  // Configura PCINT11 (IMA listo) para activar interrupciones PCINTx
}

void calibrar(void)
{
	//PORTC &= ~ (1<<pin_pul_listo);  // Poner en bajo señal de pulgar listo
	_delay_ms(250);
	printf("Iniciar calib \n");
	_delay_ms(250);
	ir_pul = ADC_GetData(IRpin);
	_delay_ms(250);
	pulsos_pul = 3000;				// Para evitar que interrupción de encoder apague pwm
	_delay_ms(250);
	if (ir_pul < 245)   // 480 - valor original
	{
		dir_pul = 0;
		pwm_pul = 100;
		onoff_pul = 1;
		PORTD &= ~(1<<dirpin_pul);
		_delay_ms(10);
		ActivarPWM_Pul(pwm_pul, onoff_pul);
		_delay_ms(10);
		while((ADC_GetData(IRpin)) <= 245 )
		{
			_delay_us(20);
		}
		ActivarPWM_Pul(0, 0);
		pwm_pul = 0;
		onoff_pul = 0;
		pulgar_listo = 1;
		
		PORTC |= (1<<pin_pul_listo);	// Poner pin en 1
		count_enc_pul = 0;
		pulsos_pul = 0;         // Para evitar que interrupción de encoder apague pwm - resetear a 0
		printf("Pul listo \n");
		_delay_ms(5);
	}
	else
	{
		PORTC |= (1<<pin_pul_listo);
		printf("Pul listo \n");
	}
	
	fc_lat = PINC & (1 << FC_pin); 
	pulsos_lat = 7000; // Para evitar que interrupción apague pwm
	if (fc_lat == 0)
	{
		dir_lat = 0;
		pwm_lat = 110;
		onoff_lat = 1;
		PORTB &= ~(1 << dirpin_lat);
		ActivarPWM_Lat(pwm_lat, onoff_lat);
		
		while((PINC & (1 << PINC3)) == 0)
		{
			// La interrupción del final de carrera se encarga de apagar motor
			_delay_us(30);
		}
		ActivarPWM_Lat(0, 0);
		pwm_pul = 0;
		onoff_pul = 0;
		
		count_enc_lat = 0;
		pulsos_lat = 0;         // Para evitar que interrupción de encoder apague pwm - resetear a 0
		printf("Lat listo \n");
		_delay_ms(5);
	}
	
	fsr_men = ADC_GetData(FSRpin_men);
	pulsos_men = 3000;         // Para evitar que interrupción de encoder apague pwm 
	if (fsr_men <= 30)
	{
		dir_men = 0;
		pwm_men = 90;
		onoff_men = 1;
		PORTB &= ~(1 << dirpin_men);
		ActivarPWM_Men(pwm_men, onoff_men);
		
		while(ADC_GetData(FSRpin_men) <= 30)
		{
	
		}
		
		ActivarPWM_Men(0, 0);
		pwm_men = 0;
		onoff_men = 0;
		count_enc_men = 0;
		pulsos_men = 0;         // Para evitar que interrupción de encoder apague pwm - resetear a 0
		printf("Men listo \n");
		_delay_ms(5);
	}
	
	abs_lateral = 0;
	abs_pulgar = 0;
	abs_menique = 0;
	
	pul_ready = 0;
	lat_ready = 0;
	men_ready = 0;
	
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
	
	if (abs_pulgar < pulgar[id_mov])
	{
		pulsos_pul = pulgar[id_mov] - abs_pulgar;
		dir_pul = 1;
		signo_pul = 1;
		pwm_pul = 100;
		onoff_pul = 1;
		PORTD |= (1<<dirpin_pul);	// Dirección del movimiento
	}
	else if (abs_pulgar > pulgar[id_mov])
	{
		pulsos_pul = abs(pulgar[id_mov] - abs_pulgar);
		dir_pul = 0;
		signo_pul = -1;
		pwm_pul = 100;
		onoff_pul = 1;
		PORTD &= ~(1<<dirpin_pul);		// Dirección del movimiento
	}
	else if (abs_pulgar == pulgar[id_mov])
	{
		pulsos_pul = 0;
		dir_pul = 0;
		pwm_pul = 0;
		onoff_pul = 0;
		PORTD &= ~(1<<dirpin_pul);		// Dirección del movimiento
	}
	
	
	if (abs_lateral < lateral[id_mov])
	{
		pulsos_lat = lateral[id_mov] - abs_lateral;
		dir_lat = 1;
		signo_lat = 1;
		pwm_lat = 150;
		onoff_lat = 1;
		PORTB |= (1<<dirpin_lat);	// Dirección del movimiento
	}
	else if (abs_lateral > lateral[id_mov])
	{
		pulsos_lat = abs(lateral[id_mov] - abs_lateral);
		dir_lat = 0;
		signo_lat = -1;
		pwm_lat = 150;
		onoff_lat = 1;
		PORTB &= ~(1<<dirpin_lat);		// Dirección del movimiento
	}
	else if (abs_lateral == lateral[id_mov])
	{
		pulsos_lat = 0;
		dir_lat = 0;
		pwm_lat = 0;
		onoff_lat = 0;
		PORTB &= ~(1<<dirpin_lat);		// Dirección del movimiento
	}
	
	if (abs_menique < menique[id_mov])
	{
		pulsos_men = menique[id_mov] - abs_menique;
		dir_men = 1;
		signo_men = 1;
		pwm_men = 100;
		onoff_men = 1;
		PORTB |= (1<<dirpin_men);	// Dirección del movimiento
	}
	else if (abs_menique > menique[id_mov])
	{
		pulsos_men = abs(menique[id_mov] - abs_menique);
		dir_men = 0;
		signo_men = -1;
		pwm_men = 100;
		onoff_men = 1;
		PORTB &= ~(1<<dirpin_men);		// Dirección del movimiento
	}
	else if (abs_menique == menique[id_mov])
	{
		pulsos_men = 0;
		dir_men = 0;
		pwm_men = 0;
		onoff_men = 0;
		PORTB &= ~(1<<dirpin_men);		// Dirección del movimiento
	}
	
	printf("PP: %d \n", pulsos_pul);
	_delay_ms(5);
	printf("PL: %d \n", pulsos_lat);
	_delay_ms(5);
	printf("PM: %d \n", pulsos_men);
	_delay_ms(5);
	
	pul_ready = 0;
	lat_ready = 0;
	men_ready = 0;
}

void pulgarCero(void)
{
	ir_pul = ADC_GetData(IRpin);
	pulsos_pul = 1000;				// Para evitar que interrupción de encoder apague pwm
	if (ir_pul < 245)
	{
		dir_pul = 0;
		pwm_pul = 100;
		onoff_pul = 1;						// Para activar motor
		PORTD &= ~(1<<dirpin_pul);			// Dirección del motor
		ActivarPWM_Pul(pwm_pul, onoff_pul);
		_delay_us(10);
		while((ADC_GetData(IRpin)) <= 245)
		{
			_delay_us(15);
		}
		ActivarPWM_Pul(0, 0);
		pwm_pul = 0;
		onoff_pul = 0;
		pulgar_listo = 1;
		abs_pulgar = 0;
		//printf("CH 3 \n");
		//_delay_ms(5);
		//PORTC |= (1<<pin_pul_listo);	// Enviar señal de pulgar listo
		count_enc_pul = 0;
		//_delay_ms(250);
	}
	else
	{
		pulgar_listo = 1;
		//PORTC |= (1<<pin_pul_listo);	// Enviar señal de pulgar listo
		count_enc_pul = 0;
		abs_pulgar = 0;
		//_delay_ms(250);
	}
	
	//PORTC &= ~(1<<pin_pul_listo);  // Poner en bajo señal enviada
}


ISR(INT0_vect)
{
	if (count_enc_pul < pulsos_pul)
	{
		count_enc_pul ++;
	}
	else
	{
		ActivarPWM_Pul(0, 0);
		count_enc_pul = 0;
		onoff_pul = 0;
		pwm_pul = 0;
		pul_ready = 1;
		printf("Pul come \n");
	}
	
}

ISR(INT1_vect)
{
	if (count_enc_lat < pulsos_lat)
	{
		count_enc_lat ++;
	}
	else
	{
		ActivarPWM_Lat(0, 0);
		count_enc_lat = 0;
		onoff_lat = 0;
		pwm_lat = 0;
		lat_ready = 1;
		printf("Lat come \n");
		
	}
}



ISR (PCINT2_vect)
{
	uint8_t changedbits;

	changedbits = PIND ^ portdhistory;
	portdhistory = PIND;

	if(changedbits & (1 << PIND4))
	{
		if (count_enc_men < pulsos_men)
		{
			count_enc_men ++;
		}
		else
		{
			ActivarPWM_Men(0, 0);
			count_enc_men = 0;
			onoff_men = 0;
			pwm_men = 0;
			men_ready = 1;
			printf("Men come %d \n", men_ready);
			
		}
	}
	
}


ISR (PCINT1_vect)
{
	uint8_t changedbits;

	changedbits = PINC ^ portchistory;
	portchistory = PINC;

	if(changedbits & (1 << PINC3))
	{
		if ( (PINC & (1 << PINC3)) == (1 << PINC3) )
		{
			
			ActivarPWM_Lat(0, 0);
			count_enc_lat = 0;
			onoff_lat = 0;
			pwm_lat = 0;
			printf("Lat come \n");
			fc_lat = 1;
			lat_ready = 1;
			abs_lateral = 0;
			printf("FC val: %d \n", fc_lat);
			lat_ready = 1;
			abs_lateral = 0;
		}
		else
		{
			fc_lat = 0;
			printf("FC val: %d \n", fc_lat);
		}
	}
	
	if(changedbits & (1 << PINC5))
	{
		if ( (PINC & (1 << PINC5)) == (1 << PINC5) )
		{
			count_event ++;
			//ima_listo = 1;
			printf("Evento: %d \n", count_event);
		}
		else
		{
			//printf("IMA 0 \n");
		}
	}
	
}
