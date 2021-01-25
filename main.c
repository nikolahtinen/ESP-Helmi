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
#include <xttcps.h>             // 16-bit timer counter driver - Triple Timer Counter (TTC)
#include <zynq_registers.h>
#include <xuartps_hw.h>
#include <xil_printf.h>

#define BUTTONS_channel		2
#define BUTTONS_AXI_ID		XPAR_AXI_GPIO_SW_BTN_DEVICE_ID

#define SWITCHES_channel	1
#define SWITCHES_AXI_ID		XPAR_AXI_GPIO_SW_BTN_DEVICE_ID

#define LEDS_channel		1
#define LEDS_AXI_ID			XPAR_AXI_GPIO_LED_DEVICE_ID

#define INTC_DEVICE_ID 		XPAR_PS7_SCUGIC_0_DEVICE_ID
#define INT_PushButtons		61

// Device ID number for PS TTC0 (XPAR_XTTCPS_0_DEVICE_ID / XPAR_PS7_TTC_0_DEVICE_ID)
#define TTC_TICK_DEVICE_ID    0U
// Interrupt (IRQ) ID# number for TTC0 counter 0 from UG585 "TRM: - ch. 8.5 TTC" (XPAR_XTTCPS_0_INTR / XPS_TTC0_0_INT_ID)
#define TTC_TICK_INTR_ID 42U

#define LD0 		0x1
#define LD1		 	0x2
#define LD2 		0x4
#define LD3 		0x8

// constant for pwm input max value
#define PWM_UMAX 5

XGpio BTNS_SWTS, LEDS;
static XScuGic INTCInst;
u8 buttons = 0;

// Set up routines for timer counters and interrupts
static void SetupTicker();
static void SetupTimer();
static void TickHandler(void);
static void SetupInterruptSystem(XScuGic * InterruptControllerInstancePtr);

volatile float u_pwm = 0;
float u_in = 2.0;

// Käyttöliittymän muuttujat tässä, pitää olla global jotta voidaan muokata interruptin kanssa
int modulation = 0;
float k_p = 20.0; 	//Hard parameters for PI controller thrown from a hat
float k_i = 500;

//Määritellään bool muuttuja tyyppi
typedef enum { false, true } bool;
bool saato = false;



static int IntcInitFunction(u16 DeviceId);
int InterruptSystemSetup(XScuGic *XScuGicInstancePtr);
void PushButtons_Intr_Handler(void *data);
float converter( float u_in);
float PI(float y_ref, float y_act ,float Ki, float Kp,float * pu1);

/* The XScuGic driver instance data. The user is required to allocate a
 * variable of this type for every intc device in the system. A pointer
 * to a variable of this type is then passed to the driver API functions.
 */
XScuGic InterruptControllerInstance; // Interrupt controller instance

int main(void)
{
	int Status;
	float u_out = 0.0;
	float u_old = u_in;

	// Connect the Intc to the interrupt subsystem such that interrupts can occur.  This function is application specific.
	SetupInterruptSystem( &InterruptControllerInstance);
	 // Set up  the Ticker timer
	SetupTimer();
	SetupTicker();

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
	// Connect the Intc to the interrupt subsystem such that interrupts can occur.  This function is application specific.
	SetupInterruptSystem( &InterruptControllerInstance);
	// Set up  the Ticker timer
	SetupTimer();
	SetupTicker();

	while(1)
		{
		switch(modulation)
		{
			case  0:
				printf("Paina Ensimmaistä nappia aloittaaksesi\n");
				break;

			case  1:
				if(saato == true)
				{
					printf("Configurointi moodi\n Säädettävä Parametri on k_i, k_i=%2.6f\n",k_i);
				} else
				{
					printf("Configurointi moodi\n Säädettävä Parametri on k_p, k_p=%2.6f\n",k_p);
				}

				break;

			case  2:
				printf("Idlataan\n");
				break;

			case  3:
				printf("Modulointimoodi\n");
				while(modulation == 3){
					u_out = PI(u_in, u_out, k_i, k_p, &u_old); 	//Call PI-controller with reference voltage and output voltage of previous cycle and P I parameters
					u_out = converter(u_out);
					printf("Referenssi u_ref= %2.6f\n Muuntajan u_out= %2.6f\n\n",u_in,u_out);
					sleep(1);
					u_pwm = u_out/PWM_UMAX * 1000;
				}
				break;

		}
		sleep(1);

		//Kirjotentaan ensimmäinen ledi kun siirrytään configuraatio moodiin
		//XGpio_DiscreteWrite(&LEDS, LEDS_channel, LD0);


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
	*/

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

static void SetupInterruptSystem(XScuGic * InterruptControllerInstancePtr) {

  //XScuGic_Config structure contains configuration information for the device
  XScuGic_Config InterruptControllerConfigInstance; // Initialize structure of type XScuGic_Config
  XScuGic_Config * InterruptControllerConfigPtr = & InterruptControllerConfigInstance; // These could be also declared global

  // Initialize the interrupt controller driver (XScuGic_Config structure)
  InterruptControllerConfigPtr->DeviceId = XPAR_PS7_SCUGIC_0_DEVICE_ID;
  InterruptControllerConfigPtr->CpuBaseAddress = XPAR_PS7_SCUGIC_0_BASEADDR;
  InterruptControllerConfigPtr->DistBaseAddress = XPAR_PS7_SCUGIC_0_DIST_BASEADDR;
  InterruptControllerConfigPtr->HandlerTable[INTC_DEVICE_ID].CallBackRef = NULL;
  InterruptControllerConfigPtr->HandlerTable[INTC_DEVICE_ID].Handler = NULL;

  /**
   * CfgInitialize a specific interrupt controller instance/driver. The
   * initialization entails:
   *
   * - Initialize fields of the XScuGic structure
   * - Initial vector table with stub function calls
   * - All interrupt sources are disabled
   */


  XScuGic_CfgInitialize(InterruptControllerInstancePtr, InterruptControllerConfigPtr, InterruptControllerConfigPtr->CpuBaseAddress);
  // Connect the interrupt controller interrupt handler to the hardware interrupt handling logic in the ARM processor.
  // Initialize the exception vector table
  Xil_ExceptionInit();
  // Enable interrupts.
  Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT, (Xil_ExceptionHandler) XScuGic_InterruptHandler, InterruptControllerInstancePtr);
  // Enable interrupts in the ARM
  Xil_ExceptionEnable();

}

void PushButtons_Intr_Handler(void *data)
{
	buttons = XGpio_DiscreteRead(&BTNS_SWTS, BUTTONS_channel);
	switch(buttons)
	{
		case LD0:
			if (modulation <= 2)
			{
				modulation = modulation + 1;
			} else
			{
				modulation = 1;
			}
			break;
		case LD1:
			if (modulation == 1)
			{
				saato = !saato;
			}
			break;
		case LD2:
			if (modulation == 1)
			{
				if(saato == true)
				{
					if(k_i >= 0){
						k_i = k_i -50;
					}
				} else
				{
					if(k_p >= 0){
						k_p = k_p - 1 ;
					}
				}

			} else if (modulation ==3)
			{
				if (u_in >= 0.5)
				{
					u_in = u_in -0.5;
				}
			}
			break;
		case LD3:
			if (modulation == 1)
			{
				if(saato == true)
				{
					if(k_i >= 0){
						k_i = k_i + 50;
					}
				} else
				{
					if(k_p >= 0){
						k_p = k_p + 1 ;
					}
				}

			} else if (modulation ==3)
			{
				if (u_in <= 9.5)
				{
					u_in = u_in +0.5;
				}
			}
			break;

		// Vanha switch case interrupt esimerkistä
		/*case LD0:
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
			break;*/
	}
	XGpio_InterruptClear(&BTNS_SWTS,0xF);
}
void SetupTimer() {
  TTC0_CNT_CNTRL |= XTTCPS_CNT_CNTRL_DIS_MASK; //Counter Control Register: "Disable the counter"
  // Reset the count control register to it's default value.
  TTC0_CNT_CNTRL = XTTCPS_CNT_CNTRL_RESET_VALUE; //Counter Control Register:" Reset value"

  // Reset the rest of the registers to the default values.
  TTC0_CLK_CNTRL = 0;
  TTC0_INTERVAL_VAL = 0;
  TTC0_MATCH_1 	= 0;
  TTC0_MATCH_2_COUNTER_2 = 0;
  TTC0_MATCH_3_COUNTER_2 = 0;
  TTC0_IER = 0;

  // Reset the counter value
  // TTC0_CNT_CNTRL |= XTTCPS_CNT_CNTRL_RST_MASK; // Counter Control Register: "Reset counter"

  // Set the options
  //Counter Control Register: "Reset counter" | "Disable the counter" | "Match mode" | "Interval mode" |"Enable output Waveform"
  TTC0_CNT_CNTRL = XTTCPS_CNT_CNTRL_RST_MASK | XTTCPS_CNT_CNTRL_DIS_MASK | XTTCPS_CNT_CNTRL_MATCH_MASK | XTTCPS_CNT_CNTRL_INT_MASK| XTTCPS_CNT_CNTRL_POL_WAVE_MASK ;

  TTC0_MATCH_0	= 0;
  // Set the interval and prescale. Base clock is 111MHz
  // Prescale value (N): if prescale is enabled, the  count rate is divided by 2^(N+1)
  // 1 / (111MHz) * 555 * 2^(1+1) = 0.00002... [seconds]
  TTC0_INTERVAL_VAL = 555;
  TTC0_CLK_CNTRL &= ~(XTTCPS_CLK_CNTRL_PS_VAL_MASK | XTTCPS_CLK_CNTRL_PS_EN_MASK); // Clock Control register - clear: "Prescale value" & "Prescale enable"
  TTC0_CLK_CNTRL |= (1 << XTTCPS_CLK_CNTRL_PS_VAL_SHIFT) | XTTCPS_CLK_CNTRL_PS_EN_MASK; // Clock Control register - set: "Prescale value" & "Prescale enable"
}
/*
 * This function sets up Generic Interrupt Controller (GIC) and Triple Timer Counter (TTC) for interrupts
 */
void SetupTicker(void) {
  // Connect to the interrupt controller (pointer to function)
  InterruptControllerInstance.Config->HandlerTable[TTC_TICK_INTR_ID].Handler = (Xil_InterruptHandler) TickHandler;
  // Enable the interrupt for the Timer counter
  // ICDISER1:  Interrupt Controller Distributor (ICD) - Interrupt Set-enable Register (ISER) 1
  ICDISER1 = 1 << (TTC_TICK_INTR_ID % 32); // Modulo operator (% 2^n) stripping off all but the n LSB bits (removes offset 32)
  // Enable the interrupts for the tick timer/counter. We only care about the interval timeout.
  // TCC0_IER: Triple Timer Counter (TCC) 0 - Interrupt Enable register (IER)
  TTC0_IER |= XTTCPS_IXR_INTERVAL_MASK; // XTTCPS_IXR_INTERVAL_MASK: Interval Interrupt mask
  // Start the tick timer/counter
  TTC0_CNT_CNTRL &= ~XTTCPS_CNT_CNTRL_DIS_MASK; // Clear operation using XTTCPS_CNT_CNTRL_DIS_MASK
}

/*
 * This function is the timer interrupt handler/
 * the function that's run on interrupt.
 */
static void TickHandler(void) {


	static unsigned int timer_ticks;		// local visibility,as global otherwise, alternative simple tick
	// Read the interrupt status to clear the interrupt.
	// TTC0_ISR: Triple Timer Counter (TTC) 0 - Interrupt Status Register (ISR)
	TTC0_ISR; // Cleared on read
	// Interrupt occurs every 20us --> 1000 ticks period is 20ms
	timer_ticks++;
	if (timer_ticks < u_pwm)
	{
		TTC0_MATCH_0  = u_pwm;
	}
	else if(timer_ticks >= 1000)
	{
		timer_ticks=0;
		//AXI_LED_DATA ^= 0b1111; // Toggle (XOR operator (^)) two LEDs
	}
	else
	{
		TTC0_MATCH_0  = 0;
	}
}
