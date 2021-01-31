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

/* LUT includes. */
#include "zynq_registers.h"


static void UART_1();
static void PWM_2();
static void Convert_3();
static void PI_4();
static void printtaa_5();
static void Buttons_6();

static void SetupUART();

//static TaskHandle_t xTask4;
xSemaphoreHandle LEDsem;

#define BUTTON1 0x01	//
#define BUTTON2 0x02	//
#define BUTTON3 0x04
#define BUTTON4 0x08	//



// K‰yttˆliittym‰n muuttujat t‰ss‰, pit‰‰ olla global jotta voidaan muokata interruptin kanssa
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

float converter( float u_in);
float PI(float y_ref, float y_act ,float Ki, float Kp,float * pu1);

int main( void ) {
	  AXI_BTN_TRI |= 0xF; // Set direction for buttons 0..3 ,  0 means output, 1 means input
      AXI_LED_TRI = ~0xF;			// Set direction for bits 0-3 to output for the LEDs
      SetupUART();

      xil_printf( "Hello from FreeRTOS example main\r\n" );

	/**
	 * Create four tasks t
	 * Each function behaves as if it had full control of the controller.
	 * https://www.freertos.org/a00125.html
	 */
      // xtaskCreat(The function that implements the task, Text name for the task, provided to assist debugging only,
      // 			The stack allocated to the task, The task parameter is not used, so set to NULL,
      // 			The task runs at the idle priority. Higher number means higher priority)
      xTaskCreate( UART_1, "PWM", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL );
      xTaskCreate( PWM_2, "UART", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL );
      xTaskCreate( Convert_3, "Convert", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );
      xTaskCreate( PI_4, "PI",configMINIMAL_STACK_SIZE, NULL,  tskIDLE_PRIORITY+4, NULL );
      xTaskCreate( printtaa_5, "Printti", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, NULL );   // Higher priority
      xTaskCreate( Buttons_6, "Buttons", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL ); // Highest of priority


	// LED Sema just as an example

	vSemaphoreCreateBinary(LEDsem);

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

float  converter( float u_in){
	//Defining variables to persist over function calls
	static float i1_k = 0.0;
	static float u1_k = 0.0;
	static float i2_k = 0.0;
	static float u2_k = 0.0;
	static float i3_k = 0.0;
	static float u3_k = 0.0;
	/*static float i1_k_h = 0.0;
	static float u1_k_h = 0.0;
	static float i2_k_h = 0.0;
	static float u2_k_h = 0.0;
	static float i3_k_h = 0.0;  */
	float u3_k_h = 0.0;

	//Hardcoded calculations for model
	i1_k = (1973*i1_k)/2000 - (403*u1_k)/25000 + (1203*u_in)/50000;
	u1_k = (7261*i1_k)/10000 - (1009*i2_k)/5000 - (1377*i3_k)/5000 + (3009*u1_k)/10000 + (3229*u2_k)/100000 + (3281*u3_k)/5000 + (1771*u_in)/100000;
	i2_k = (7001*i1_k)/10000 + (213*i2_k)/1250 + (3881*i3_k)/10000 + (82*u1_k)/125 + (59*u2_k)/250 - (4501*u3_k)/5000 + (427*u_in)/25000;
	u2_k = (2577*i1_k)/5000 - (1377*i2_k)/5000 + (1341*i3_k)/5000 + (153*u1_k)/1250 + (7353*u2_k)/10000 + (341*u3_k)/2500 + (1257*u_in)/100000;
	i3_k = (6727*i1_k)/10000 - (2799*i2_k)/5000 + (421*i3_k)/2500 + (179*u1_k)/200 - (2299*u2_k)/10000 - (6703*u3_k)/10000 + (1641*u_in)/100000;
	u3_k = (2261*i1_k)/10000 + (271*i2_k)/1000 + (1009*i3_k)/5000 + (6421*u1_k)/10000 + (3519*u2_k)/100000 + (643*u3_k)/2000 + (6358362097906761*u_in)/1152921504606846976;
	u3_k_h = u3_k;
	return u3_k_h;

}

float PI(float y_ref, float y_act ,float Ki, float Kp,float * pu1){
	float  u1_old;
	float  error_new, u1_new, u_new;
	float  u1_max = 10;
	u1_old=*pu1;
	error_new = y_ref-y_act;
	u1_new = u1_old + Ki* error_new;
	if (abs(u1_new)>= u1_max){		// check Saturation.
		u1_new = u1_old;				// saturate integer
	}
	u_new =Kp* error_new+ u1_new;	// u=u2+u1
	u1_old = u1_new;
	*pu1 =u1_old;
	return u_new;
}

void SetupUART(){
       uint32_t r = 0; // Temporary value variable

       r = UART_CTRL;
       r &= ~(XUARTPS_CR_TX_EN | XUARTPS_CR_RX_EN); // Clear Tx & Rx Enable
       r |= XUARTPS_CR_RX_DIS | XUARTPS_CR_TX_DIS; // Tx & Rx Disable
       UART_CTRL = r;

       UART_MODE = 0;
       UART_MODE &= ~XUARTPS_MR_CLKSEL; // Clear "Input clock selection" - 0: clock source is uart_ref_clk
       UART_MODE |= XUARTPS_MR_CHARLEN_8_BIT; // Set "8 bits data"
       UART_MODE |= XUARTPS_MR_PARITY_NONE; // Set "No parity mode"
       UART_MODE |= XUARTPS_MR_STOPMODE_1_BIT; // Set "1 stop bit"
       UART_MODE |= XUARTPS_MR_CHMODE_NORM; // Set "Normal mode"

       // baud_rate = sel_clk / (CD * (BDIV + 1) (ref: UG585 - TRM - Ch. 19 UART)
       UART_BAUD_DIV = 6; // ("BDIV")
       UART_BAUD_GEN = 124; // ("CD")
       // Baud Rate = 100Mhz / (124 * (6 + 1)) = 115200 bps

       UART_CTRL |= (XUARTPS_CR_TXRST | XUARTPS_CR_RXRST); // TX & RX logic reset

       r = UART_CTRL;
       r |= XUARTPS_CR_RX_EN | XUARTPS_CR_TX_EN; // Set TX & RX enabled
       r &= ~(XUARTPS_CR_RX_DIS | XUARTPS_CR_TX_DIS); // Clear TX & RX disabled
       UART_CTRL = r;

}

// Check if UART receive FIFO is not empty and return the new data
char uart_receive() {
	if ((UART_STATUS & XUARTPS_SR_RXEMPTY) == XUARTPS_SR_RXEMPTY) return 0;
	return UART_FIFO;
}

// Change mode if button1 pressed or 1 send from terminal
static void Control1(){
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
			if(k_i >= 0) {
				k_i = k_i -50;
			}
		}
		else{
			if(k_p >= 0) {
				k_p = k_p - 1 ;
			}
		}
	}
	else if (modulation ==3){
		if (u_ref >= 0.5) {
			u_ref = u_ref -0.5;
		}
	}

}
// Increase parameter value in mode 1 or ref value in mode 3, if button4 pressed or + send from terminal
static void Control4(){
	if (modulation == 1) {
		if(saato == true) {
			if(k_i >= 0) {
				k_i = k_i + 50;
			}
		}
		else {
			if(k_p >= 0) {
				k_p = k_p + 1 ;
			}
		}
	}
	else if (modulation ==3) {
		if (u_ref <= 9.5) {
			u_ref = u_ref +0.5;
		}
	}
}

/* Implement a function in a C file to generate a periodic interrupt at the
required frequency. */
//void vSetupTickInterrupt( void )
//{
/* FreeRTOS_Tick_Handler() is itself defined in the RTOS port layer.  An extern
declaration is required to allow the following code to compile. */
//extern void FreeRTOS_Tick_Handler( void );

    /* Assume TIMER1_configure() configures a hypothetical timer peripheral called
    TIMER1 to generate a periodic interrupt with a frequency set by its parameter. */
  //  TIMER1_configure( 10);

    /* Next assume Install_Interrupt() installs the function passed as its second
    parameter as the handler for the peripheral passed as its first parameter. */
    //Install_Interrupt( TIMER1, FreeRTOS_Tick_Handler );
//}



static void PI_4() {

	// ms -> tics, only if tick interval is more than ms, give directly as ticks
	const TickType_t delay = pdMS_TO_TICKS( 1000 );

	// Necessary forever loop. A thread should never be able to exit!
	for( ;; ) { // Same as while(1) or while(true)

		AXI_LED_DATA =AXI_LED_DATA ^ 0x01;
		if(modulation ==3){
			u_in = PI(u_ref, u_out, k_i, k_p, &u_old); 	//Call PI-controller with reference voltage and output voltage of previous cycle and P I parameters
		}
		else
			u_in = 0;

		// RTOS specific delay. Normal delay would halt the system for the duration => Is not multitasking!
		// https://www.freertos.org/a00127.html

		vTaskDelay( delay );  // not synchronous , do not use in control or in real time models
	}
}

static void Convert_3() {
	const TickType_t freq = pdMS_TO_TICKS( 750 );
	TickType_t wakeTime;

	/* If juu need floating point context, and in conf it
	 *  is not set for all #define configUSE_TASK_FPU_SUPPORT 2
	 *  use the following */

	// portTASK_USES_FLOATING_POINT();

	// https://www.freertos.org/a00021.html#xTaskGetTickCount
	wakeTime = xTaskGetTickCount();  // only once initialized

	for( ;; ) {
		AXI_LED_DATA =AXI_LED_DATA ^ 0x02;
		if(modulation ==3){
			u_out = converter(u_in);
		}
		else
			u_out = 0;
		// https://www.freertos.org/vtaskdelayuntil.html
		vTaskDelayUntil( &wakeTime, freq );

	}
}

static void Buttons_6() {
	const TickType_t freq = pdMS_TO_TICKS( 500 );
	TickType_t wakeTime;

	wakeTime = xTaskGetTickCount();
	for( ;; ) {

		//xSemaphoreTake(LEDsem,portMAX_DELAY);
		buttons = AXI_BTN_DATA;
		switch(buttons) {
			case BUTTON1:
				Control1();
				break;
			case BUTTON2:
				Control2();
				break;
			case BUTTON3:
				Control3();
				break;
			case BUTTON4:
				Control4();
				break;
		}
		//xSemaphoreGive(LEDsem);

		vTaskDelayUntil( &wakeTime, freq );
	}
}

static void UART_1() {
	const TickType_t freq = pdMS_TO_TICKS( 1000 );
	TickType_t wakeTime;

	wakeTime = xTaskGetTickCount();
	for( ;; ) {

		// Semaphore is just as an example here

		xSemaphoreTake(LEDsem,portMAX_DELAY);
		AXI_LED_DATA =AXI_LED_DATA ^ 0x04;
		xSemaphoreGive(LEDsem);

		//xil_printf("Uart task executed!!!\r\n");
		char temp;
		temp = uart_receive();
		int i = 0;
		while(temp){
			i++;
			if(temp == 49)	// ascii 49 = 1
				Control1();
			if(temp == 50)	// ascii 50 = 2
				Control2();
			if(temp == 43)	// ascii + = 43
				Control4();
			if(temp == 45)	// ascii - = 45
				Control3();
			xil_printf("%d: %d\r\n",i,temp);
			temp = uart_receive();
		}

	}

	vTaskDelayUntil( &wakeTime, freq );

}

static void PWM_2() {

	const TickType_t freq = pdMS_TO_TICKS( 1000 );
	TickType_t wakeTime;

	wakeTime = xTaskGetTickCount();
	for( ;; ) {

		// Semaphore is just as an example here

		xSemaphoreTake(LEDsem,portMAX_DELAY);
		AXI_LED_DATA =AXI_LED_DATA ^ 0x08;
		xSemaphoreGive(LEDsem);

		//uint8_t input = AXI_LED_DATA & 0xF;  // Range: 0..15
		//uint16_t delay = 1000 - 60*input; // Range: 100-1000ms

		//TickType_t freq = pdMS_TO_TICKS( delay );

		vTaskDelayUntil( &wakeTime, freq );
		//vTaskDelayUntil( &wakeTime, 200 );
	}
}
static void printtaa_5() {

	const TickType_t freq = pdMS_TO_TICKS( 750 );
		TickType_t wakeTime;

		wakeTime = xTaskGetTickCount();
		for (;;) {
			switch(modulation)
			{
				case  0:
					xil_printf("\nMode: %d ", modulation);
					printf("Paina Ensimmaist‰ nappia / 1 aloittaaksesi\n");
					break;

				case  1:
					xil_printf("\nMode: %d ", modulation);
					if(saato == true) {
						printf("Configurointi moodi\nS‰‰dett‰v‰ Parametri on k_i: %2.6f\n",k_i);
						printf("Nappi 4/+ kasvattaa ja nappi 3/- v‰hent‰‰\n");
						printf("Nappi 2/2 vaihtaa parametria\n");
					}
					else {
						printf("Configurointi moodi\nS‰‰dett‰v‰ Parametri on k_p: %2.6f\n",k_p);
						printf("Nappi 4/+ kasvattaa ja nappi 3/- v‰hent‰‰\n");
						printf("Nappi 2/2 vaihtaa parametria\n");
					}

					break;

				case  2:
					xil_printf("\nMode: %d ", modulation);
					printf("Idlataan\n");
					break;

				case  3:
					xil_printf("\nMode: %d ", modulation);
					printf("Modulointimoodi\n");
					printf("Referenssi u_ref: %2.6f\n",u_ref);
					printf("S‰‰tˆarvot Kp: %2.2f Ki: %2.2f\n", k_p, k_i);
					printf("Muuntajan u_in/ PI out: %2.6f\n",u_in);
					printf("Muuntajan u_out: %2.6f\n",u_out);

					vTaskDelayUntil( &wakeTime, freq );

					break;

			}
			vTaskDelayUntil( &wakeTime, freq );
		}
}
/* ex.c
 *
 *  Created on: 27.1.2021
 *      Author: jriek
 */




