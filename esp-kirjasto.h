/*
 * esp-kirjasto.h
 *
 *  Created on: 28.2.2021
 *      Author: Niko
 */

#ifndef SRC_ESP_KIRJASTO_H_
#define SRC_ESP_KIRJASTO_H_

#include "zynq_registers.h"


float  converter( float u_in){
	//Defining variables to persist over function calls
	printf("converter\r\n");
	static float i1_k = 0.0;
	static float u1_k = 0.0;
	static float i2_k = 0.0;
	static float u2_k = 0.0;
	static float i3_k = 0.0;
	static float u3_k = 0.0;

	//Hardcoded calculations for model
	i1_k = (1973*i1_k)/2000 - (403*u1_k)/25000 + (1203*u_in)/50000;
	u1_k = (7261*i1_k)/10000 - (1009*i2_k)/5000 - (1377*i3_k)/5000 + (3009*u1_k)/10000 + (3229*u2_k)/100000 + (3281*u3_k)/5000 + (1771*u_in)/100000;
	i2_k = (7001*i1_k)/10000 + (213*i2_k)/1250 + (3881*i3_k)/10000 + (82*u1_k)/125 + (59*u2_k)/250 - (4501*u3_k)/5000 + (427*u_in)/25000;
	u2_k = (2577*i1_k)/5000 - (1377*i2_k)/5000 + (1341*i3_k)/5000 + (153*u1_k)/1250 + (7353*u2_k)/10000 + (341*u3_k)/2500 + (1257*u_in)/100000;
	i3_k = (6727*i1_k)/10000 - (2799*i2_k)/5000 + (421*i3_k)/2500 + (179*u1_k)/200 - (2299*u2_k)/10000 - (6703*u3_k)/10000 + (1641*u_in)/100000;
	u3_k = (2261*i1_k)/10000 + (271*i2_k)/1000 + (1009*i3_k)/5000 + (6421*u1_k)/10000 + (3519*u2_k)/100000 + (643*u3_k)/2000 + (6358362097906761*u_in)/1152921504606846976;


	return u3_k;
}

float PI(float y_ref, float y_act ,float Ki, float Kp,float * pu1){
	float  u1_old;
	float  error_new, u1_new, u_new;
	float  u1_max = 10;
	u1_old=*pu1;
	error_new = y_ref-y_act;


	u1_new = u1_old + Ki* error_new;
	if(abs(u1_new)>= u1_max){		// check Saturation.
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

void setupPWM(){
	// Two TTC module in the Zynq PS (TTC0 & TTC1)
		// Each TTC module contains three independent 16-bit prescalers and 16-bit up/down counters (0,1,2)
		// The register naming follows Xilinx TRM UG585 for consistency - however it's not a very good example of proper naming!

		// First we need to set up the 'Clock Control' -register - TTC0_CLK_CNTRLx :
		//     1. Set prescale to 0 (plus 1) (hint: (value << XTTCPS_CLK_CNTRL_PS_VAL_SHIFT)
		//     2. Enable prescaler (hint: use XTTCPS_CLK_CNTRL_PS_EN_MASK mask)
		TTC0_CLK_CNTRL = (0 << XTTCPS_CLK_CNTRL_PS_VAL_SHIFT) | XTTCPS_CLK_CNTRL_PS_EN_MASK;
		TTC0_CLK_CNTRL3 = (0 << XTTCPS_CLK_CNTRL_PS_VAL_SHIFT)| XTTCPS_CLK_CNTRL_PS_EN_MASK; // Set identical to TTC0_CLK_CNTRL

		// Then let's set correct values to 'Operational mode and reset' -register - TTC0_CNT_CNTRLx :
		//     1. Reset count value (hint: use XTTCPS_CNT_CNTRL_RST_MASK mask)
		//     2. Disable counter (XTTCPS_CNT_CNTRL_DIS_MASK)
		//     3. Set timer to Match mode (XTTCPS_CNT_CNTRL_MATCH_MASK)
		//     4. Enable output Waveform (XTTCPS_CNT_CNTRL_POL_WAVE_MASK)
		//        (Waveform output is default to EMIO, which is connected in the FPGA to the RGB led (LD6)
		TTC0_CNT_CNTRL = XTTCPS_CNT_CNTRL_RST_MASK | XTTCPS_CNT_CNTRL_DIS_MASK | XTTCPS_CNT_CNTRL_MATCH_MASK | XTTCPS_CNT_CNTRL_POL_WAVE_MASK;
		TTC0_CNT_CNTRL3 = XTTCPS_CNT_CNTRL_RST_MASK | XTTCPS_CNT_CNTRL_DIS_MASK | XTTCPS_CNT_CNTRL_MATCH_MASK | XTTCPS_CNT_CNTRL_POL_WAVE_MASK; // Set identical to TTC0_CNT_CNTRL
		//     1. Initialize match value to 0
		TTC0_MATCH_0 = 0;
		TTC0_MATCH_1_COUNTER_3 = 0;

		// Operational mode and reset register - TTC0_CNT_CNTRLx
		//     1. Start timer (hint: clear operation using XTTCPS_CNT_CNTRL_DIS_MASK)
		TTC0_CNT_CNTRL &= ~XTTCPS_CNT_CNTRL_DIS_MASK;
		TTC0_CNT_CNTRL3 &= ~XTTCPS_CNT_CNTRL_DIS_MASK;
}


#endif /* SRC_ESP_KIRJASTO_H_ */
