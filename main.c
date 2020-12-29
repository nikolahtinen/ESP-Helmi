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

int main(void)
{
	int Status;

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
		// Nothing here.
	}

	return 0;
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
