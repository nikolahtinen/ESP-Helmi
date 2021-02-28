/*
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?".  Have you defined configASSERT()?  *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *   Investing in training allows your team to be as productive as       *
     *   possible as early as possible, lowering your overall development    *
     *   cost, and enabling you to bring a more robust product to market     *
     *   earlier than would otherwise be possible.  Richard Barry is both    *
     *   the architect and key author of FreeRTOS, and so also the world's   *
     *   leading authority on what is the world's most popular real time     *
     *   kernel for deeply embedded MCU designs.  Obtaining your training    *
     *   from Richard ensures your team will gain directly from his in-depth *
     *   product knowledge and years of usage experience.  Contact Real Time *
     *   Engineers Ltd to enquire about the FreeRTOS Masterclass, presented  *
     *   by Richard Barry:  http://www.FreeRTOS.org/contact
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *    You are receiving this top quality software for free.  Please play *
     *    fair and reciprocate by reporting any suspected issues and         *
     *    participating in the community forum:                              *
     *    http://www.FreeRTOS.org/support                                    *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!

    Modified by lindh LUT
*/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
#include "xil_types.h"
#include <xuartps.h>
#include <xuartps_hw.h>
#include <xttcps.h>             // 16-bit timer counter driver - Triple Timer Counter (TTC)
#include <xscugic.h>
#include <stdlib.h>
#include <stdio.h>

/* Team includes */
#include "esp-kirjasto.h"
#include "zynq_registers.h"


/* Tasks */
static void UART_1();
static void PWM_2();
static void Convert_3();
static void PI_4();
static void printtaa_5();
static void Buttons_6();

//static void SetupUART();
static void Control1();
static void Control2();
static void Control3();
static void Control4();

//static TaskHandle_t xTask4;

/* Semaphorat */
xSemaphoreHandle LEDsem;
xSemaphoreHandle Uin_sem;
xSemaphoreHandle Uout_sem;
xSemaphoreHandle Uref_sem;
xSemaphoreHandle Upwm_sem;
xSemaphoreHandle KP_sem;
xSemaphoreHandle KI_sem;
xSemaphoreHandle modulation_sem;



#define BUTTON1 0x01
#define BUTTON2 0x02
#define BUTTON3 0x04
#define BUTTON4 0x08

#define INTC_DEVICE_ID 		XPAR_PS7_SCUGIC_0_DEVICE_ID
// Device ID number for PS TTC0 (XPAR_XTTCPS_0_DEVICE_ID / XPAR_PS7_TTC_0_DEVICE_ID)
#define TTC_TICK_DEVICE_ID    0U
// Interrupt (IRQ) ID# number for TTC0 counter 0 from UG585 "TRM: - ch. 8.5 TTC" (XPAR_XTTCPS_0_INTR / XPS_TTC0_0_INT_ID)
#define TTC_TICK_INTR_ID 	42U
// constant for pwm input max value
#define PWM_UMAX 40



// Global variables
int modulation = 0;
u8 buttons = 0;
float k_p = 20.0; 	//Hard parameters for PI controller thrown from a hat
float k_i = 500;
int Status;
float u_ref = 2;
float u_in = 0;
float u_out = 0;
float u_old = 0;
float u_pwm = 0;
//M‰‰ritell‰‰n bool muuttuja tyyppi
typedef enum { false, true } bool;
bool saato = false;

//float converter( float u_in);
//float PI(float y_ref, float y_act ,float Ki, float Kp,float * pu1);

int main( void ) {
	  AXI_BTN_TRI |= 0xF;		// Set direction for buttons 0..3 ,  0 means output, 1 means input

	  AXI_LED_TRI = ~0xF;		// Set direction for bits 0-3 to output for the LEDs

	  SetupUART();				// Setup UART for the terminal communications


	  // xtaskCreat(The function that implements the task, Text name for the task, provided to assist debugging only,
	  // 			The stack allocated to the task, The task parameter is not used, so set to NULL,
	  // 			The task runs at the idle priority. Higher number means higher priority)
	  xTaskCreate( UART_1, "UART", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL );
	  xTaskCreate( PWM_2, "PWM", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL );
	  xTaskCreate( Convert_3, "Convert", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL );
	  xTaskCreate( PI_4, "PI",configMINIMAL_STACK_SIZE, NULL,  tskIDLE_PRIORITY+2, NULL );
	  xTaskCreate( printtaa_5, "Printti", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL );   // Higher priority
	  xTaskCreate( Buttons_6, "Buttons", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL ); // Highest of priority

    // Semaphorien initialisointi

	vSemaphoreCreateBinary(LEDsem);
	Uin_sem = xSemaphoreCreateMutex();
	Uout_sem = xSemaphoreCreateMutex();
	Uref_sem = xSemaphoreCreateMutex();
	Upwm_sem = xSemaphoreCreateMutex();
	KI_sem  = xSemaphoreCreateMutex();
	KP_sem = xSemaphoreCreateMutex();
	modulation_sem = xSemaphoreCreateMutex();

	// Start the tasks and timer running.
	// https://www.freertos.org/a00132.html
	vTaskStartScheduler();



	/**
	 * If all is well, the scheduler will now be running, and the following line
	 * will never be reached.  If the following line does execute, then there was
	 * insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	 * to be created.  See the memory management section on the FreeRTOS web site
	 * for more details.
	 */
	for( ;; );
}

static void Control1(){
	// Change mode if button1 pressed or 1 send from terminal
	if (modulation <= 2) {
		modulation = modulation + 1;
	}
	else {
		modulation = 1;
	}
}
// Change parameter in mode 1, if button2 pressed or 2 send from terminal
static void Control2(){
	if (modulation == 1) {
		saato = !saato;
	}
}
// Decrease parameter value in mode 1 or ref value in mode 3, if button3 pressed or - send from terminal
static void Control3(){
	if (modulation == 1) {
		if(saato == true) {
			xSemaphoreTake(KI_sem, ( TickType_t ) 10);
			if(k_i >= 0) {
				k_i = k_i -50;
			}
			xSemaphoreGive(KI_sem);
		}
		else{
			xSemaphoreTake(KP_sem, ( TickType_t ) 10 );
			if(k_p >= 0) {
				k_p = k_p - 1 ;
			}
			xSemaphoreGive(KP_sem);
		}
	}
	else if (modulation ==3){
		xSemaphoreTake(Uref_sem, ( TickType_t ) 10 );
		if (u_ref >= 0.5) {
			u_ref = u_ref -0.5;
		}
		xSemaphoreGive(Uref_sem);
	}

}
// Increase parameter value in mode 1 or ref value in mode 3, if button4 pressed or + send from terminal
static void Control4(){
	if (modulation == 1) {
		if(saato == true) {
			xSemaphoreTake(KI_sem, ( TickType_t ) 10);
			if(k_i >= 0) {
				k_i = k_i + 50;
			}
			xSemaphoreGive(KI_sem);
		}
		else {
			xSemaphoreTake(KP_sem, ( TickType_t ) 10 );
			if(k_p >= 0) {
				k_p = k_p + 1 ;
			}
			xSemaphoreGive(KP_sem);
		}
	}
	else if (modulation ==3) {
		xSemaphoreTake(Uref_sem, ( TickType_t ) 10 );
		if (u_ref <= 9.5) {
			u_ref = u_ref +0.5;
		}
		xSemaphoreGive(Uref_sem);
	}

}


static void PI_4() {

	// ms -> tics, only if tick interval is more than ms, give directly as ticks
	const TickType_t freq = pdMS_TO_TICKS(500);
	TickType_t wakeTime;

	// https://www.freertos.org/a00021.html#xTaskGetTickCount
	wakeTime = xTaskGetTickCount();  // only once initialized
	// Necessary forever loop. A thread should never be able to exit!
	for (;;) { // Same as while(1) or while(true)
		AXI_LED_DATA = AXI_LED_DATA ^ 0x01;

		if (modulation == 3) {

			xSemaphoreTake( Uin_sem, ( TickType_t ) 10 );
			xSemaphoreTake( Uin_sem, ( TickType_t ) 10 );
			xSemaphoreTake( Uout_sem, ( TickType_t ) 10 );
			xSemaphoreTake( KI_sem, ( TickType_t ) 10 );
			xSemaphoreTake( KP_sem, ( TickType_t ) 10 );

			u_in = PI(u_ref, u_out, k_i, k_p, &u_old); //Call PI-controller with reference voltage and output voltage of previous cycle and P I parameters

			xSemaphoreGive( Uin_sem );
			xSemaphoreGive( Uin_sem );
			xSemaphoreGive( Uout_sem );
			xSemaphoreGive( KI_sem );
			xSemaphoreGive( KP_sem );
		} else {
			xSemaphoreTake( Uin_sem, ( TickType_t ) 10 );
			u_in = u_old = 0;
			xSemaphoreGive( Uin_sem );
		}

		xSemaphoreTake(Upwm_sem, ( TickType_t ) 10);
		// Uin value calculated for the PWM led 0-65535
		u_pwm = u_in / PWM_UMAX * 65535;
		xSemaphoreGive(Upwm_sem);
		// https://www.freertos.org/vtaskdelayuntil.html
		vTaskDelayUntil(&wakeTime, freq);
		taskYIELD();
	}
}

static void Convert_3() {


	const TickType_t freq = pdMS_TO_TICKS( 750 );

	TickType_t wakeTime;

	// https://www.freertos.org/a00021.html#xTaskGetTickCount
	wakeTime = xTaskGetTickCount();  // only once initialized

	for( ;; ) {
		if( xSemaphoreTake( Uin_sem, portMAX_DELAY ) == pdTRUE ){
			AXI_LED_DATA =AXI_LED_DATA ^ 0x02;
			// Convert output value if mode 3
			if(modulation ==3){
				u_out = converter(u_in);
			}
			else {
				u_out = 0;
			}
			xSemaphoreGive( Uin_sem );
		}
			// https://www.freertos.org/vtaskdelayuntil.html
			vTaskDelayUntil( &wakeTime, freq );
			taskYIELD();
	}
}

static void Buttons_6() {

	// Task for the button control
	// call control functions depending which button is pressed
	int Button_delay = 50;
	//TickType_t wakeTime;

	//wakeTime = xTaskGetTickCount();
	for( ;; ) {

		//xSemaphoreTake(LEDsem,portMAX_DELAY);
		xSemaphoreTake(modulation_sem,portMAX_DELAY);
		Button_delay = 20;

		buttons = AXI_BTN_DATA;
		switch(buttons) {
			case BUTTON1:
				Control1(modulation);
				Button_delay = 200;
				break;
			case BUTTON2:
				Control2(modulation);
				Button_delay = 200;
				break;
			case BUTTON3:
				Control3(modulation);
				Button_delay = 200;
				break;
			case BUTTON4:
				Control4(modulation);
				Button_delay = 200;
				break;
		}
		xSemaphoreGive(modulation_sem);
		//xSemaphoreGive(LEDsem);

		vTaskDelay( Button_delay );
		taskYIELD();
	}
}


static void UART_1() {

	// Read value from FIFO if exist
	// and call control functions
	const TickType_t freq = pdMS_TO_TICKS( 1000 );
	TickType_t wakeTime;

	wakeTime = xTaskGetTickCount();
	for( ;; ) {

		xSemaphoreTake(LEDsem,portMAX_DELAY);
		AXI_LED_DATA =AXI_LED_DATA ^ 0x04;
		xSemaphoreGive(LEDsem);
		char temp;
		temp = uart_receive();

		while(temp){

			if(temp == 49)	// ascii 49 = 1
				Control1(modulation);
			if(temp == 50)	// ascii 50 = 2
				Control2(modulation);
			if(temp == 43)	// ascii 43 = +
				Control4(modulation);
			if(temp == 45)	// ascii 45 = -
				Control3(modulation);

			temp = uart_receive();
		}

		vTaskDelayUntil( &wakeTime, freq );
		taskYIELD();

	}

}


static void PWM_2() {

	setupPWM();

	// PWM led is on as long as timer_ticks is smaller than u_pvm value
	// totally 65536/512 = 128 steps
	// Brightness depends value of u_pvm

	// Variable initialization
	uint16_t timer_ticks = 0;
	volatile u32* ptr_register = NULL;

	for (;;) {

		AXI_LED_DATA = AXI_LED_DATA ^ 0x08;

		timer_ticks += 512;
		xSemaphoreTake(Upwm_sem, ( TickType_t ) 10);
		// If value is positive
		if (u_pwm > 0) {
			*ptr_register = 0;
			ptr_register = &TTC0_MATCH_0; // if positive led red
			if (timer_ticks < u_pwm) {
				*ptr_register = u_pwm;
			} else if (timer_ticks >= 65536) {
				timer_ticks = 0;
			} else {
				*ptr_register = 0;
			}
		}
		// if value is negative
		else {
			*ptr_register = 0;
			ptr_register = &TTC0_MATCH_1_COUNTER_3; // if negative led blue
			if (timer_ticks < abs(u_pwm)) {
				*ptr_register = abs(u_pwm);
			} else if (timer_ticks >= 65536) {
				timer_ticks = 0;
			} else {
				*ptr_register = 0;
			}
		}
		xSemaphoreGive(Upwm_sem);

		taskYIELD();

	}
}
static void printtaa_5() {

	// Printing to terminal for user which state is on
	// mode 0: start
	// mode 1: configuration
	// mode 2: idle
	// mode 3: modulation

	const TickType_t freq = pdMS_TO_TICKS( 1000 );
		TickType_t wakeTime;

		wakeTime = xTaskGetTickCount();
		for (;;) {
			xSemaphoreTake(modulation_sem,portMAX_DELAY);

			switch(modulation)
			{
				case  0:
					printf("\n\n\n\n\n\n\n\n\n\n\n\nMode: %d ", modulation);
					printf("Paina Ensimmaist‰ nappia / 1 aloittaaksesi\n");
					break;

				case  1:
					printf("\n\n\n\n\n\n\n\n\n\n\n\nMode: %d ", modulation);
					if(saato == true) {
						xSemaphoreTake(KI_sem,portMAX_DELAY);
						printf("Configurointi moodi\nS‰‰dett‰v‰ Parametri on k_i: %2.6f\n",k_i);
						printf("Nappi 4/+ kasvattaa ja nappi 3/- v‰hent‰‰\n");
						printf("Nappi 2/2 vaihtaa parametria\n");
						xSemaphoreGive(KI_sem);
					}
					else {
						xSemaphoreTake(KP_sem,portMAX_DELAY);
						printf("Configurointi moodi\nS‰‰dett‰v‰ Parametri on k_p: %2.6f\n",k_p);
						printf("Nappi 4/+ kasvattaa ja nappi 3/- v‰hent‰‰\n");
						printf("Nappi 2/2 vaihtaa parametria\n");
						xSemaphoreGive(KP_sem);
					}

					break;

				case  2:
					printf("\n\n\n\n\n\n\n\n\n\n\n\nMode: %d ", modulation);
					printf("Idlataan\n");
					break;

				case  3:
					xSemaphoreTake(Uin_sem,portMAX_DELAY);
					xSemaphoreTake(Uout_sem,portMAX_DELAY);
					xSemaphoreTake(Uref_sem,portMAX_DELAY);
					xSemaphoreTake(KP_sem,portMAX_DELAY);
					xSemaphoreTake(KI_sem,portMAX_DELAY);



					printf("\n\n\n\n\n\n\n\n\n\n\n\nMode: %d ", modulation);
					printf("Modulointimoodi\n");
					printf("Referenssi u_ref: %2.6f\n",u_ref);
					printf("S‰‰tˆarvot Kp: %2.2f Ki: %2.2f\n", k_p, k_i);
					printf("Muuntajan u_in/ PI out: %2.6f\n",u_in);
					printf("Muuntajan u_out: %2.6f\n",u_out);

					xSemaphoreGive(Uin_sem);
					xSemaphoreGive(Uout_sem);
					xSemaphoreGive(Uref_sem);
					xSemaphoreGive(KP_sem);
					xSemaphoreGive(KI_sem);


					break;

			}
			xSemaphoreGive(modulation_sem);
			vTaskDelayUntil( &wakeTime, freq );
			taskYIELD();
		}
}
/* ex.c
 *
 *  Created on: 27.1.2021
 *      Author: jriek
 */




