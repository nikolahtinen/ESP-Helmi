/*
 * AXI_interrupts.c
 *
 *  Created on: 16 Oct 2020
 *      Author: Adminuko
 */


#include <xparameters.h>
#include <xgpio.h>
#include <xuartps.h>
#include <xscugic.h>
#include <sleep.h>

#define BUTTONS_channel		2
#define BUTTONS_AXI_ID		XPAR_AXI_GPIO_SW_BTN_DEVICE_ID

#define SWITCHES_channel	1
#define SWITCHES_AXI_ID		XPAR_AXI_GPIO_SW_BTN_DEVICE_ID

#define LEDS_channel		1
#define LEDS_AXI_ID			XPAR_AXI_GPIO_LED_DEVICE_ID

#define INTC_DEVICE_ID 		XPAR_PS7_SCUGIC_0_DEVICE_ID
#define INT_PushButtons		61

#define LD0 		0x1
#define LD1		 	0x2
#define LD2 		0x4
#define LD3 		0x8

XGpio BTNS_SWTS, LEDS;
static XScuGic INTCInst;
u8 buttons = 0;

static int IntcInitFunction(u16 DeviceId);
int InterruptSystemSetup(XScuGic *XScuGicInstancePtr);
void PushButtons_Intr_Handler(void *data);
float converter( float u_in);
float PI(float y_ref, float y_act ,float Ki, float Kp,float * pu1);

int main(void)
{
	int Status;
	float u_in = 2.0;
	float u_out = 0.0;
	float u_old = u_in;

	float k_p = 5.0; 	//Hard parameters for PI controller thrown from a hat
	float k_i = 5.0;


	// Initializes BTNS_SWTS as an XGPIO.
	Status = XGpio_Initialize(&BTNS_SWTS, BUTTONS_AXI_ID);
	if (Status != XST_SUCCESS)
	{
		xil_printf("Buttons and switches error\n");
		return XST_FAILURE;
	}

	Status = XGpio_Initialize(&LEDS, LEDS_AXI_ID);
	if (Status != XST_SUCCESS)
	{
		xil_printf("LEDs error\n");
		return XST_FAILURE;
	}

	XGpio_SetDataDirection(&BTNS_SWTS, BUTTONS_channel, 0xF);
	XGpio_SetDataDirection(&BTNS_SWTS, SWITCHES_channel, 0xF);
	XGpio_SetDataDirection(&LEDS, LEDS_channel, 0x0);

	// Initializes interruptions.
	Status = IntcInitFunction(INTC_DEVICE_ID);


	while(1)
	{
		u_out = PI(u_in, u_out, k_i, k_p, &u_old); 	//Call PI-controller with reference voltage and output voltage of previous cycle and P I parameters
		u_out = converter(u_out);
		printf("%2.6f\r\n",u_out);
		sleep(1);
		// Nothing here.
	}

	return 0;
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

static int IntcInitFunction(u16 DeviceId)
{
	int Status;

	XScuGic_Config *IntcConfig;

	// Interrupt controller initialization
	IntcConfig = XScuGic_LookupConfig(DeviceId);
	Status = XScuGic_CfgInitialize(&INTCInst, IntcConfig, IntcConfig->CpuBaseAddress);
	if(Status != XST_SUCCESS) return XST_FAILURE;

	// Call to interrupt setup
	Status = InterruptSystemSetup(&INTCInst);
	if(Status != XST_SUCCESS) return XST_FAILURE;

	XScuGic_SetPriorityTriggerType(&INTCInst, INT_PushButtons, 0, 3);	//Max priority, rising edge.

	Status = XScuGic_Connect(&INTCInst,	INT_PushButtons, (Xil_ExceptionHandler)PushButtons_Intr_Handler, (void *) 0);
	if(Status != XST_SUCCESS) return XST_FAILURE;

	XScuGic_Enable(&INTCInst, INT_PushButtons);

	return XST_SUCCESS;
}

int InterruptSystemSetup(XScuGic *XScuGicInstancePtr)
{
	/*
	 * Initialize the interrupt controller driver so that it is ready to use.
	 * */

	XGpio_InterruptEnable(&BTNS_SWTS, 0xF);
	XGpio_InterruptGlobalEnable(&BTNS_SWTS);

	Xil_ExceptionInit();

	// Enable interrupts.

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_FIQ_INT,
			 	 	 	 	 	 (Xil_ExceptionHandler) PushButtons_Intr_Handler,
			 	 	 	 	 	 XScuGicInstancePtr);
	Xil_ExceptionEnableMask(XIL_EXCEPTION_FIQ);

	return XST_SUCCESS;
}

void PushButtons_Intr_Handler(void *data)
{
	buttons = XGpio_DiscreteRead(&BTNS_SWTS, BUTTONS_channel);
	switch(buttons)
	{
		case LD0:
			XGpio_DiscreteWrite(&LEDS, LEDS_channel, LD0);	// LD0.
			break;
		case LD1:
			XGpio_DiscreteWrite(&LEDS, LEDS_channel, LD1);	// LD1.
			break;
		case LD2:
			XGpio_DiscreteWrite(&LEDS, LEDS_channel, LD2);	// LD2.
			break;
		case LD3:
			XGpio_DiscreteWrite(&LEDS, LEDS_channel, LD3);	// LD3.
			break;
	}
	XGpio_InterruptClear(&BTNS_SWTS,0xF);
}
