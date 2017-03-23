/* gameoflife.c - ECE 544 final Project Application
 *
 * Copyright (c) 2017 Portland State University by Richa Thakur,Wesley Chavez ,Jeba farhana.  All rights reserved.
 *
 * Author	:		Richa Thakur, Wesley Chavez, Jeba Farhana
 * Date		:		3/20/2017
 * Version	:		1.0
 *
 * Revision History:
 * -----------------
 *  03-21-2017		RT,WC,JF		Xilkernel supported application implementing the simulation of
 *  								 John Conway's cellular automaton "GAME OF LIFE"
 *
 * 	Description:
 * 	------------
 * 	This program implements ECE 544 Final Project (John Conway's GAME OF LIFE).  It is a Xilinx-based design that simulates
 * 	the mathematical theory of John Conway who in 1970 at Cambridge discovered the GAME OF LIFE which is basically a
 * 	cellular automaton that describes how from simple basic rules the simple form can change into complex forms simulating
 * 	for several of its generations. this application includes several of the key functions provided by Xilkernel.
 * 	The application sets up 2 independent threads and a master thread.
 * 	The three threads communicate via a message queue and their functionalities are described as below:
*   a)Pattern_Creation_Thread : This thread invokes the calculation of the next frame based on the existing frame
* 								It acquires the lock on the critical section in order to calculate the frame based on the existing base pattern
* 								It also get the rotary count to control the speed at which frames are generated
* 								Calls the toggleBram function which identifies the BRAM at which the current frame is displayed and then toggles the frame
* 								Once done, this thread releases the lock on the critical section based on the post method on the semaphore
 * 	b)Pattern_selection_Thread	:	This thread selects the base pattern based on the status of the switches
* 									It acquires lock on the critical section only when the status of the switches are changed
* 									Once the lock is acquired, it checks generates the base pattern and also displays the selection in the OLEDBRGB
* 									Once the base pattern is selected and generated, this thread releases the lock so that pattern creation thread can create the nextframe
* The pushbuttons are used as follows:
* 	o 	press BtnUp and BtnDown to locate the Y coordinate
* 	o 	press BtnLeft and BtnRight to locate the X coordinate
* 	o 	press BtnC to confirm the user created cell dead or alive
* 	o 	press the rotary switch to change the frame rate/speed of simulation
* The 4th thread (the master thread) sets up the MicroBlaze, initializes the peripherals, creates the other threads and
* the semaphore and then enters a main loop. Status messages are sent via STDOUT through the USB serial port
*  which can be connected to a PC running a terminal emulator Tera term.

/************************************************************************************************************************************/
//Include libraries

#include "xmk.h"
#include "os_config.h"
#include "config/config_param.h"
#include "sys/ksched.h"
#include "sys/init.h"
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <semaphore.h>
#include <sys/intr.h>
#include <sys/timer.h>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include "xintc.h"
#include "xparameters.h"
#include "platform_config.h"
#include "platform.h"
#include "stdbool.h"
#include "xgpio.h"
#include "xtmrctr.h"
#include "xstatus.h"
#include <stdlib.h>
#include <math.h>
#include "nexys4IO.h"
#include "pmodENC.h"
#include "PmodOLEDrgb.h"
#include "xbram.h"


/************************** Constant Definitions ****************************/
// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0

// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_S00_AXI_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_S00_AXI_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 5KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		5000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			5

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_OUTPUT_0_CHANNEL		1


// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR

#define BRAM_0_DEVICE_ID			XPAR_AXI_BRAM_CTRL_0_DEVICE_ID
#define BRAM_0_BASEADDR				XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR
#define BRAM_0_HIGHADDR				XPAR_AXI_BRAM_CTRL_0_S_AXI_HIGHADDR

#define BRAM_1_DEVICE_ID			XPAR_AXI_BRAM_CTRL_1_DEVICE_ID
#define BRAM_1_BASEADDR				XPAR_AXI_BRAM_CTRL_1_S_AXI_BASEADDR
#define BRAM_1_HIGHADDR				XPAR_AXI_BRAM_CTRL_1_S_AXI_HIGHADDR

/************************** Variable Definitions ****************************/
unsigned long timeStamp = 0;


/************************** Function Prototypes *****************************/
int do_init_nx4io(u32 BaseAddress);
int AXI_Timer_initialize(void);
int do_init();									//Initialize the AXI timer, gpio, interrupt, FIT timer, Encoder,OLED display
void* master_thread(void *arg);					//master thread that invokes the child thread
void* Pattern_Creation_Thread(void *arg);		//Thread that invokes calculation of next frame and creates the pattern
void* Pattern_Selection_Thread(void *arg);		//Thread that selects the base patter based on the status of the switches
void usleep(u32 usecs);
void calcNextFrame(void);						//Calculates the next frame depending on neighbouring cells
void toggleBramDisplay(void);					//Identifies the BRAM on which the current frame is stored
void pauseSim(void);							//Pauses and resumes the simulation
void clearSim(void);							//Clears the simulation
void FIT_Handler(void);							//Fixed interrupt Timer Handler

PmodENC 	pmodENC_inst;				// PmodENC instance ref
PmodOLEDrgb	pmodOLEDrgb_inst;			// PmodOLED instance ref
XGpio		GPIOInst0;					// GPIO instance
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				//  timer instance

XBram			XBram0_inst;			//BRAM0 instance
XBram_Config	XBram_Config0;

XBram			XBram1_inst;			//BRAM1 instance
XBram_Config	XBram_Config1;

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler.

volatile u32 gpio_out;
volatile u32 fit_count = 0;
volatile u32 fit_flag = 0;
volatile u32 high_fit_count = 5000;
volatile u32 bram_display = 0;
volatile u16 sw = 0;
volatile u16 sw_init = 0;
sem_t cond_sema;					//semaphore
pthread_t patternselection;			//patternselection thread
pthread_t patterncreation;			//pattern creation thread

/************************** MAIN PROGRAM ************************************/
int main()
{
	int sts;
    init_platform();

    sts = do_init();		// initialize the peripherals
    if (XST_SUCCESS != sts)
    {
    	exit(1);
    }

    xilkernel_init(); //Kernel Initialization
    pthread_t mst_thread;
    sts = pthread_create(&mst_thread, NULL, (void*) master_thread, NULL); //Master thread invocation
    xilkernel_start();
    pthread_join(master_thread, NULL);  //Wait for the master thread to complete
    cleanup_platform();
    return 0;

}
/*****************************************MASTER THREAD********************************************************
* Master thread is used to create other child threads
* It also registers the FIT handler and enable the interrupts
* Intializes the semaphore that is used to lock the thread whenever control inputs such as switches are changed
*
*****************************************************************************************************************/

void* master_thread(void *arg)
{

	int status;

	//create the pattern selection thread
	status = pthread_create(&patternselection, NULL, (void*) Pattern_Selection_Thread, NULL);
	if (status != 0)
	{
		xil_printf("MASTER THREAD ERROR - Pattern Selection Thread: Master Thread Terminating\r\n");
		return (void*) -3;
	}

	//creates the pattern creation thread
	status = pthread_create(&patterncreation, NULL, (void*) Pattern_Creation_Thread, NULL);
	if (status != 0)
	{
		xil_printf("MASTER THREAD ERROR - Pattern Creation Thread: Master Thread Terminating\r\n");
		return (void*) -3;
	}

	//initializes the semaphore
	status = sem_init (&cond_sema, 0, 0);
	if (status != 0)
	{
		xil_printf("MASTER THREAD ERROR - Condition Sema: Master Thread Terminating\r\n");
		return (void*) -3;
	}
	else
	{
		xil_printf ("MASTER THREAD: Condition semaphore has been initialized\n\r");
	}


	//registers the fixed interval timer interrupt handler
	status = register_int_handler(FIT_INTERRUPT_ID, (void*) FIT_Handler, NULL);
    if (status != XST_SUCCESS){
		return (void*) -4;
    }

	enable_interrupt(FIT_INTERRUPT_ID);
	xil_printf("MASTER: Interrupts have been enabled\r\n");

	while(1){

	}
    return NULL;
}

/**
 * Function Name: do_init()
 *
 * Return: XST_FAILURE or XST_SUCCESS
 *
 * Description: Initialize the AXI timer, gpio, interrupt, FIT timer, Encoder,
 * 				OLED display
 */
int do_init()
{
	int status;

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status == XST_FAILURE)
	{
		exit(1);
	}

	// initialize the PMod544IO driver and the PmodENC and PmodCLP
	status = pmodENC_initialize(&pmodENC_inst, PMODENC_BASEADDR);
	if (status == XST_FAILURE)
	{
		exit(1);
	}

	// Initialize the AXI Timer
	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	// Initialize the OLED display
	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// GPIO0 channel 1 is a 8-bit output port.
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0x00);

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	  return XST_FAILURE;
	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);

	status = XBram_CfgInitialize(&XBram0_inst, &XBram_Config0, BRAM_0_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	status = XBram_CfgInitialize(&XBram1_inst, &XBram_Config1, BRAM_1_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}

/*****************************************PATTERN_CREATION_THREAD********************************************************
* This thread invokes the calculation of the next frame based on the existing frame
* It acquires the lock on the critical section in order to calculate the frame based on the existing base pattern
* It also get the rotary count to control the speed at which frames are generated
* Calls the togglebram function which identifies the BRAM at which the current frame is displayed and then toggles the frame
* Once done, this thread releases the lock on the critical section based on the post method on the semaphore
*****************************************************************************************************************/

void* Pattern_Creation_Thread(void *arg)
{

	bool RotaryNoNeg = true;
	int RotaryIncr = 1;
	u16 RotaryCnt;
	// Initialize Encoder knob, and clear to 0
	pmodENC_init(&pmodENC_inst, RotaryIncr, RotaryNoNeg);
	pmodENC_clear_count(&pmodENC_inst);
	NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
	NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);

	sw_init = NX4IO_getSwitches();
	sw = sw_init;

	while(1)
	{
		// Rotary count notifies the FIT counter (which runs at 5kHz) of the frame rate.
		// If RotaryCnt is 0, high_fit_count is 5000, and the frame rate is 1Hz (5kHz/5000)
		// If RotaryCnt is 49, high_fit_count is 100, and the frame rate is 50Hz (5kHz/100)
		sem_wait(&cond_sema);

		pmodENC_read_count(&pmodENC_inst, &RotaryCnt);
		if (RotaryCnt < 50)
		{
			high_fit_count = (u32)(5000 - (RotaryCnt*100));
		}

		// Switch 0 pauses the simulation for creating/destroying life
		sw = NX4IO_getSwitches();
		if ((sw & 1) == 1)
		{
			pauseSim();
		}


		// Calculate next frame (write to other BRAM) based on current frame (read from current BRAM)
		calcNextFrame();

		// If the FIT raises the flag, it's time to switch the BRAMs roles, and display the next frame
		if (fit_flag == 1)
		{
			toggleBramDisplay();
			fit_flag = 0;
		}
		sem_post(&cond_sema);

	}
}

/*****************************************PATTERN_SELECTION_THREAD********************************************************
* This thread selects the base pattern based on the status of the switches
* It acquires lock on the critical section only when the status of the switches are changed
* Once the lock is acquired, it checks generates the base pattern and also displays the selection in the OLEDBRGB
* Once the base pattern is selected and generated, this thread releases the lock so that pattern creation thread can create the nextframe
*****************************************************************************************************************/
//GenerateBasePattern
void* Pattern_Selection_Thread(void *arg){
	while(1){
	if((sw&65280) != (sw_init&65280))
	{
	sem_wait(&cond_sema);
	sw_init = sw;

	clearSim();
	OLEDrgb_Clear(&pmodOLEDrgb_inst);

	u32 base_address;
	// Read and write to the BRAM currently being displayed
	if(bram_display == 0)
	{
		base_address = BRAM_0_BASEADDR;
	}
	else
	{
		base_address = BRAM_1_BASEADDR;
	}

	if(32768==(sw & 32768))   //10 cell row
	{
		XBram_WriteReg(base_address, 128, 31);
		XBram_WriteReg(base_address, 384, 4160749568);

		//OLED display for 10 cell row
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 1);  //10 Cell Row
	    OLEDrgb_PutString(&pmodOLEDrgb_inst,"10 Cell Row");
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,36,20,55,25,255,50,50,true,255,50,50);
	}
	else if(16384==(sw & 16384))  //Glider
	{
		XBram_WriteReg(base_address, 64+64, 1);
		XBram_WriteReg(base_address, 72+64, 3);
		XBram_WriteReg(base_address, 324+64, 2147483648);
		XBram_WriteReg(base_address, 328+64, 2147483648);

		//OLED display for Glider
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);   //Glider
	    OLEDrgb_PutString(&pmodOLEDrgb_inst,"Glider");
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,20,55,25,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,55,26,63,31,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,55,32,63,36,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,32,55,36,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,39,32,47,36,255,50,50,true,255,50,50);
	}
	else if(8192==(sw & 8192))  //Small Exploder
	{
		XBram_WriteReg(base_address, 72+64, 1);
		XBram_WriteReg(base_address, 76+64, 1);
		XBram_WriteReg(base_address, 320+64, 2147483648);
		XBram_WriteReg(base_address, 324+64, 2147483648);
		XBram_WriteReg(base_address, 328+64, 1073741824);
		XBram_WriteReg(base_address, 332+64, 1073741824);
		XBram_WriteReg(base_address, 336+64, 2147483648);

		//OLED display for Small Exploder
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 1);  //Small Exploder
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"S. Exploder");
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,20,55,25,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,55,26,63,31,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,26,55,31,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,39,26,47,31,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,39,32,47,36,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,55,32,63,36,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,36,55,40,255,50,50,true,255,50,50);

	}
	else if(4096==(sw & 4096))  //Exploder
	{
		XBram_WriteReg(base_address, 64+64, 1);
		XBram_WriteReg(base_address, 68+64, 1);
		XBram_WriteReg(base_address, 72+64, 1);
		XBram_WriteReg(base_address, 76+64, 1);
		XBram_WriteReg(base_address, 80+64, 1);
		XBram_WriteReg(base_address, 320+64, 1342177280);
		XBram_WriteReg(base_address, 324+64, 268435456);
		XBram_WriteReg(base_address, 328+64, 268435456);
		XBram_WriteReg(base_address, 332+64, 268435456);
		XBram_WriteReg(base_address, 336+64, 1342177280);

		//OLED display for Exploder
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 1);   //Exploder
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Exploder");
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,20,55,25,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,36,55,40,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,63,20,71,25,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,63,25,71,30,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,63,30,71,35,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,63,35,71,40,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,31,20,39,25,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,31,25,39,30,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,31,30,39,35,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,31,35,39,40,255,50,50,true,255,50,50);

	}
	else if(2048==(sw&2048))  //Tumbler
	{
		XBram_WriteReg(base_address, 64+64, 6);
		XBram_WriteReg(base_address, 68+64, 6);
		XBram_WriteReg(base_address, 72+64, 2);
		XBram_WriteReg(base_address, 76+64, 10);
		XBram_WriteReg(base_address, 80+64, 10);
		XBram_WriteReg(base_address, 84+64, 12);
		XBram_WriteReg(base_address, 320+64, 3221225472);
		XBram_WriteReg(base_address, 324+64, 3221225472);
		XBram_WriteReg(base_address, 328+64, 2147483648);
		XBram_WriteReg(base_address, 332+64, 2684354560);
		XBram_WriteReg(base_address, 336+64, 2684354560);
		XBram_WriteReg(base_address, 340+64, 1610612736);

		//OLED display for Tumbler
		 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 1);   //Tumbler
	     OLEDrgb_PutString(&pmodOLEDrgb_inst,"Tumbler");
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,20,55,25,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,26,55,31,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,32,55,37,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,38,55,43,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,38,20,46,25,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,38,26,46,31,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,38,50,47,55,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,30,50,38,55,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,30,38,38,43,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,30,44,38,49,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,58,20,66,25,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,58,26,66,31,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,58,32,66,37,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,58,38,66,43,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,67,20,74,25,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,67,26,74,31,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,66,50,74,55,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,74,50,82,55,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,74,38,82,43,255,50,50,true,255,50,50);
	     OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,74,44,82,49,255,50,50,true,255,50,50);
	}
	else if(1024==(sw&1024))  //lightweight Spaceship
	{
		XBram_WriteReg(base_address, 64+64, 3);
		XBram_WriteReg(base_address, 68+64, 4);
		XBram_WriteReg(base_address, 76+64, 4);
		XBram_WriteReg(base_address, 320+64, 3221225472);
		XBram_WriteReg(base_address, 324+64, 1073741824);
		XBram_WriteReg(base_address, 328+64, 1073741824);
		XBram_WriteReg(base_address, 332+64, 2147483648);

		//OLED display for lightweight Spaceship
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 1);   //Spaceship
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Spaceship");
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,20,55,25,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,39,20,47,25,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,55,20,63,25,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,63,20,71,25,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,63,25,71,30,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,63,30,71,35,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,55,35,63,40,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,31,35,39,40,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,31,25,39,30,255,50,50,true,255,50,50);
	}
	else if(512==(sw&512))  // Gosper Glider Gun!!!
	{

		XBram_WriteReg(base_address, 64+64, 1575936);
		XBram_WriteReg(base_address, 68+64, 1577984);
		XBram_WriteReg(base_address, 72+64, 6168);
		XBram_WriteReg(base_address, 76+64, 20);
		XBram_WriteReg(base_address, 80+64, 16);
		XBram_WriteReg(base_address, 320+56, 805699584);
		XBram_WriteReg(base_address, 324+56, 1342570496);
		XBram_WriteReg(base_address, 328+56, 1610612736);

		XBram_WriteReg(base_address, 348+56, 196608);
		XBram_WriteReg(base_address, 352+56, 163840);
		XBram_WriteReg(base_address, 356+56, 131072);
		XBram_WriteReg(base_address, 368+56, 469762048);
		XBram_WriteReg(base_address, 372+56, 268435456);
		XBram_WriteReg(base_address, 376+56, 134217728);

		//OLED display for Gosper Glider Gun
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 1);  //Gosper Glider Gun
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"G.G. Gun");
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,30,15,38,20,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,30,21,38,26,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,38,15,46,20,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,38,21,46,26,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,54,25,62,30,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,54,31,62,36,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,63,21,71,26,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,63,26,71,31,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,71,21,78,26,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,71,26,78,31,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,80,15,88,20,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,88,10,94,15,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,88,15,94,20,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,80,10,88,15,255,50,50,true,255,50,50);
        OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,80,35,88,40,255,50,50,true,255,50,50);
        OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,72,40,80,45,255,50,50,true,255,50,50);
        OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,72,35,80,40,255,50,50,true,255,50,50);
        OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,88,40,94,45,255,50,50,true,255,50,50);


	}
	else if(256==(sw&256))  // Random
	{
		u32 randNo;
		int i;
		for (i = 1; i<63; i++)
		{
			randNo = (u32)rand(); //random function to get the random frames
			XBram_WriteReg(base_address, i*4, randNo); // write to the random frames
		}
		for (i = 65; i<127; i++)
		{
			randNo = (u32)rand(); //random function to get the random frames
			XBram_WriteReg(base_address, i*4, randNo<<1); // write to the random frames
		}

		//OLED display for random
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 1);  //random pattern
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"RANDOM");
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,30,15,38,20,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,30,21,38,26,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,38,15,46,20,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,38,21,46,26,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,80,35,88,40,255,50,50,true,255,50,50);
        OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,72,40,80,45,255,50,50,true,255,50,50);
        OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,72,35,80,40,255,50,50,true,255,50,50);
        OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,88,40,96,45,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,63,20,71,25,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,63,25,71,30,255,50,50,true,255,50,50);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,58,20,66,25,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,58,26,66,31,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,58,32,66,37,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,20,55,25,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,26,55,31,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,32,55,37,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,47,38,55,43,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,38,20,46,25,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,38,26,46,31,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,38,50,47,55,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,30,50,38,55,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,30,38,38,43,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,30,44,38,49,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,58,20,66,25,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,58,26,66,31,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,58,32,66,37,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,58,38,66,43,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,67,20,74,25,255,50,50,true,255,50,50);
	    OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,67,26,74,31,255,50,50,true,255,50,50);
	}
	sem_post(&cond_sema);
	}
	}
}

/********************************CLEAR SIM********************************************
*Clears the simulation by writing 0 to the base address in both the BRAMs
* @param None
*****************************************************************************/

void clearSim(void)
{
	int i;
	for (i = 0; i<=512; i=i+4)
	{
		XBram_WriteReg(BRAM_0_BASEADDR, i, 0); // Add 0 to all the frames of BRAM0
		XBram_WriteReg(BRAM_1_BASEADDR, i, 0); //Add 0 to all the frames of BRAM1
	}
}

/********************************PAUSE SIM********************************************/
/**
*Pauses the simulation and allows user to create their own pattern.
* @param None
*****************************************************************************/

void pauseSim(void)
{
	// x and y positions of cursor
	u32 x_pos = 0;
	u32 y_pos = 0;

	u32 bram_read_val = 0;
	u32 bram_write_val = 0;
	u32 base_address;

	// Read and write to the BRAM currently being displayed
	if(bram_display == 0)
	{
		base_address = BRAM_0_BASEADDR;
	}
	else
	{
		base_address = BRAM_1_BASEADDR;
	}

	u16 sw;
	sw = NX4IO_getSwitches();

	// Read the BRAM in row 0, and write a 1 where the cursor is currently positioned
	bram_read_val = XBram_ReadReg(base_address,0);
	bram_write_val = 2147483648 | bram_read_val;
	XBram_WriteReg(base_address, 0, bram_write_val);

	// Flipping the switch down will cause the simulation to resume
	while ((sw & 1) == 1)
	{
		// Move cursor to the right and write to BRAM, so the current cursor position is displayed


		if (NX4IO_isPressed(BTNR))
		{
			x_pos ++;
			if (x_pos < 32)
			{
				bram_write_val = (2147483648>>x_pos) | bram_read_val;
				XBram_WriteReg(base_address, y_pos*4, bram_write_val);
			}
			else if (x_pos == 32)
			{
				XBram_WriteReg(base_address, y_pos*4, bram_read_val);
				bram_read_val = XBram_ReadReg(base_address,256+y_pos*4);
				bram_write_val = 2147483648 | bram_read_val;
				XBram_WriteReg(base_address, 256+y_pos*4, bram_write_val);
			}
			else
			{
				bram_write_val = (2147483648>>(x_pos-32)) | bram_read_val;
				XBram_WriteReg(base_address, 256+y_pos*4, bram_write_val);
			}

		}
		// Move cursor to the left and write to BRAM, so the current cursor position is displayed
		if (NX4IO_isPressed(BTNL))
		{
			x_pos --;
			if (x_pos < 31)
			{
				bram_write_val = (2147483648>>x_pos) | bram_read_val;
				XBram_WriteReg(base_address, y_pos*4, bram_write_val);
			}
			else if (x_pos == 31)
			{
				XBram_WriteReg(base_address, 256+y_pos*4, bram_read_val);
				bram_read_val = XBram_ReadReg(base_address,y_pos*4);
				bram_write_val = (2147483648>>31) | bram_read_val;
				XBram_WriteReg(base_address, y_pos*4, bram_write_val);
			}
			else
			{
				bram_write_val = (2147483648>>(x_pos-32)) | bram_read_val;
				XBram_WriteReg(base_address, 256+y_pos*4, bram_write_val);
			}

		}
		// Write the current row, move the cursor up, read that address, then write the new cursor position to BRAM
		if (NX4IO_isPressed(BTNU))
		{
			if (x_pos < 32)
			{
				XBram_WriteReg(base_address, y_pos*4, bram_read_val);
				y_pos --;
				bram_read_val = XBram_ReadReg(base_address,y_pos*4);
				bram_write_val = (2147483648>>x_pos) | bram_read_val;
				XBram_WriteReg(base_address, y_pos*4, bram_write_val);
			}
			else
			{
				XBram_WriteReg(base_address, 256+y_pos*4, bram_read_val);
				y_pos --;
				bram_read_val = XBram_ReadReg(base_address,256+y_pos*4);
				bram_write_val = (2147483648>>(x_pos-32)) | bram_read_val;
				XBram_WriteReg(base_address, 256+y_pos*4, bram_write_val);
			}

		}
		// Write the current row, move the cursor down, read that address, then write the new cursor position to BRAM
		if (NX4IO_isPressed(BTND))
		{
			if (x_pos < 32)
			{
				XBram_WriteReg(base_address, y_pos*4, bram_read_val);
				y_pos ++;
				bram_read_val = XBram_ReadReg(base_address,y_pos*4);
				bram_write_val = (2147483648>>x_pos) | bram_read_val;
				XBram_WriteReg(base_address, y_pos*4, bram_write_val);
			}
			else
			{
				XBram_WriteReg(base_address, 256+y_pos*4, bram_read_val);
				y_pos ++;
				bram_read_val = XBram_ReadReg(base_address,256+y_pos*4);
				bram_write_val = (2147483648>>(x_pos-32)) | bram_read_val;
				XBram_WriteReg(base_address, 256+y_pos*4, bram_write_val);
			}
		}
		// Button C toggles (creates/destroys life) at the current pixel
		if (NX4IO_isPressed(BTNC))
		{
			if (x_pos < 32)
			{
				bram_read_val = (2147483648>>x_pos) ^ bram_read_val;
				XBram_WriteReg(base_address, y_pos*4, bram_read_val);
			}
			else
			{
				bram_read_val = (2147483648>>(x_pos-32)) ^ bram_read_val;
				XBram_WriteReg(base_address, 256+y_pos*4, bram_read_val);
			}

		}
		// Sleep for 40000 microseconds
		usleep(40000);
		// Get switch values again in case user wants to resume simulation
		sw = NX4IO_getSwitches();
	}

}

/********************************CALCULATE NEXT FRAME********************************************/
/**
*This method calculates the next frame based on the current selected frame and the neighbours
*
* @param None

*****************************************************************************/
void calcNextFrame(void)
{
	u32 read_base_address;
	u32 write_base_address;
	u32 a,b,c,d,e,f,b_write,e_write;

	// Read from one BRAM (current frame being displayed) and write to the other (next frame that we're computing)
	// bram_display is volatile
	if(bram_display == 0)
	{
		read_base_address = BRAM_0_BASEADDR;
		write_base_address = BRAM_1_BASEADDR;
	}
	else
	{
		read_base_address = BRAM_1_BASEADDR;
		write_base_address = BRAM_0_BASEADDR;
	}
	// For each half of the frame
	int i;
	for (i = 0; i < 2; i++)
	{
		// Read 3 consecutive rows to compute row currently positioned on b
		a = XBram_ReadReg(read_base_address,i*256+0);
		b = XBram_ReadReg(read_base_address,i*256+4);
		c = XBram_ReadReg(read_base_address,i*256+8);

		// Rows 1-62. (Rows 0 and 63 we don't compute)
		int j;
		for (j = 1; j < 63; j++)
		{

			u32 bram_write_val = 0;

			// Columns 1-30. (Columns 0 and 31 we don't compute)
			int k;
			for (k = 1; k < 31; k++)
			{
				// Compute number of alive neighbors around current pixel
				u32 neighbors = 0;
				neighbors += (a>>(k-1))&(u32)1;
				neighbors += (a>>k)&(u32)1;
				neighbors += (a>>(k+1))&(u32)1;
				neighbors += (b>>(k-1))&(u32)1;
				neighbors += (b>>(k+1))&(u32)1;
				neighbors += (c>>(k-1))&(u32)1;
				neighbors += (c>>k)&(u32)1;
				neighbors += (c>>(k+1))&(u32)1;

				// Is the current pixel alive?
				u32 active = (b>>k)&(u32)1;

				// Rules of The Game of Life
				// Currently alive
				if (active == 1)
				{
					// Current pixel dies if lonely or overcrowded, shift in a 0
					if (neighbors < 2 || neighbors > 3)
					{

						bram_write_val = (bram_write_val >> 1);
					}
					// Current pixel lives on, shift in a 1
					else
					{
						bram_write_val = (bram_write_val >> 1)+2147483648;
					}
				}
				// Currently dead
				else
				{
					// Reproduction, shift in a 1
					if (neighbors == 3)
					{
						bram_write_val = (bram_write_val >> 1)+2147483648;
					}
					// Still dead, shift in a 0
					else
					{
						bram_write_val = (bram_write_val >> 1);
					}
				}
			}

			// Write the values computed for current row to BRAM
			bram_write_val = (bram_write_val >> 1);
			XBram_WriteReg(write_base_address, i*256+j*4, bram_write_val);

			// Move down a row
			a = b;
			b = c;
			c = XBram_ReadReg(read_base_address,i*256+(j+2)*4);
		}
	}

	// Next, compute boundaries (middle of the frame)
	// For this, we need to read 8 registers, two from the BRAM we just wrote to,
	// and six for the memory locations surrounding the current pixel, since
	// border conditions apply here.

	a = XBram_ReadReg(read_base_address,0);
	b = XBram_ReadReg(read_base_address,4);
	b_write = XBram_ReadReg(write_base_address,4);
	c = XBram_ReadReg(read_base_address,8);
	d = XBram_ReadReg(read_base_address,256);
	e = XBram_ReadReg(read_base_address,260);
	e_write = XBram_ReadReg(write_base_address,260);
	f = XBram_ReadReg(read_base_address,264);

	u32 bram_write_val_b;
	u32 bram_write_val_e;
	u32 neighbors_b;
	u32 neighbors_e;
	u32 active_b;
	u32 active_e;

	// Rows 1-62. (Rows 0 and 63 we don't compute)
	int j;
	for (j = 1; j < 63; j++)
	{

		bram_write_val_b = 0;
		bram_write_val_e = 0;

		// Compute number of alive neighbors around current pixel
		neighbors_b = 0;
		neighbors_e = 0;

		neighbors_b +=  a&(u32)1;
		neighbors_b += (a>>1)&(u32)1;
		neighbors_b += (b>>1)&(u32)1;
		neighbors_b +=  c&(u32)1;
		neighbors_b += (c>>1)&(u32)1;
		neighbors_b += (d>>31)&(u32)1;
		neighbors_b += (e>>31)&(u32)1;
		neighbors_b += (f>>31)&(u32)1;

		neighbors_e +=  a&(u32)1;
		neighbors_e +=  b&(u32)1;
		neighbors_e +=  c&(u32)1;
		neighbors_e += (d>>31)&(u32)1;
		neighbors_e += (d>>30)&(u32)1;
		neighbors_e += (e>>30)&(u32)1;
		neighbors_e += (f>>31)&(u32)1;
		neighbors_e += (f>>30)&(u32)1;

		// Is the current pixel alive?
		active_b = b&(u32)1;
		active_e = (e>>31)&(u32)1;

		// Rules of The Game of Life
		// Currently alive
		if (active_b == 1)
		{
			// Current pixel dies if lonely or overcrowded, write a 0
			if (neighbors_b < 2 || neighbors_b > 3)
			{

				bram_write_val_b = (b_write & 4294967294);
			}
			// Current pixel lives on, write a 1
			else
			{
				bram_write_val_b = (b_write | 1);
			}
		}
		// Currently dead
		else
		{
			// Reproduction, write a 1
			if (neighbors_b == 3)
			{
				bram_write_val_b = (b_write | 1);
			}
			// Still dead, write a 0
			else
			{
				bram_write_val_b = (b_write & 4294967294);
			}
		}

		// Rules of The Game of Life
		// Currently alive
		if (active_e == 1)
		{
			// Current pixel dies if lonely or overcrowded, write a 0
			if (neighbors_e < 2 || neighbors_e > 3)
			{

				bram_write_val_e = (e_write & 2147483647);
			}
			// Current pixel lives on, write a 1
			else
			{
				bram_write_val_e = (e_write | 2147483648);
			}
		}
		// Currently dead
		else
		{
			// Reproduction, write a 1
			if (neighbors_e == 3)
			{
				bram_write_val_e = (e_write | 2147483648);
			}
			// Still dead, write a 0
			else
			{
				bram_write_val_e = (e_write & 2147483647);
			}
		}

		// Write the values computed for current row to BRAM
		XBram_WriteReg(write_base_address, j*4, bram_write_val_b);
		XBram_WriteReg(write_base_address, 256+j*4, bram_write_val_e);

		// Move down a row
		a = b;
		b = c;
		b_write = XBram_ReadReg(write_base_address,(j+1)*4);
		c = XBram_ReadReg(read_base_address,(j+2)*4);
		d = e;
		e = f;
		e_write = XBram_ReadReg(write_base_address,256+(j+1)*4);
		f = XBram_ReadReg(read_base_address,256+(j+2)*4);
	}
}

/********************************TOGGLE BRAM DISPLAY********************************************/
/**
* The current frame is informed to both hardware and software through the 
* gpio_out port whose bit 0 defines whether it is stored in BRAM 0 or BRAM 1. 
* If the gpio_out[0] = 0 then the current frame is stored in BRAM 0 
* and gpio_out[0] = 1 then the current frame is stored in BRAM 1.
*
* @param None

*****************************************************************************/
void toggleBramDisplay(void)
{
	// If current frame is stored in BRAM 0, let software and hardware know
	if (bram_display == 0)
	{
		bram_display = 1;
		XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 1);
	}
	// If current frame is stored in BRAM 1, let software and hardware know
	else
	{
		bram_display = 0;
		XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0);
	}
}

/*********************** HELPER FUNCTIONS ***********************************/

/****************************************************************************/
/**
* insert delay (in microseconds) between instructions.
*
* This function should be in libc but it seems to be missing.  This emulation implements
* a delay loop with (really) approximate timing; not perfect but it gets the job done.
*
* @param	usec is the requested delay in microseconds
*
* @return	*NONE*
*
* @note
* This emulation assumes that the microblaze is running @ 100MHz and takes 15 clocks
* per iteration - this is probably totally bogus but it's a start.
*
*****************************************************************************/

static const u32	DELAY_1US_CONSTANT	= 15;	// constant for 1 microsecond delay

void usleep(u32 usec)
{
	volatile u32 i, j;

	for (i = 0; i < usec; i++)
	{
		for (j = 0; j < DELAY_1US_CONSTANT; j++);
	}
	return;
}


/****************************************************************************/
/**
* initialize the Nexys4 LEDs and seven segment display digits
*
* Initializes the NX4IO driver, turns off all of the LEDs and blanks the seven segment display
*
* @param	BaseAddress is the memory mapped address of the start of the Nexys4 registers
*
* @return	XST_SUCCESS if initialization succeeds.  XST_FAILURE otherwise
*
* @note
* The NX4IO_initialize() function calls the NX4IO self-test.  This could
* cause the program to hang if the hardware was not configured properly
*
*****************************************************************************/
int do_init_nx4io(u32 BaseAddress)
{
	int sts;

	// initialize the NX4IO driver
	sts = NX4IO_initialize(BaseAddress);
	if (sts == XST_FAILURE)
		return XST_FAILURE;

	// turn all of the LEDs off using the "raw" set functions
	// functions should mask out the unused bits..something to check w/
	// the debugger when we bring the drivers up for the first time
	NX4IO_setLEDs(0xFFF0000);
	NX4IO_RGBLED_setRGB_DATA(RGB1, 0xFF000000);
	NX4IO_RGBLED_setRGB_DATA(RGB2, 0xFF000000);
	NX4IO_RGBLED_setRGB_CNTRL(RGB1, 0xFFFFFFF0);
	NX4IO_RGBLED_setRGB_CNTRL(RGB2, 0xFFFFFFFC);

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	return XST_SUCCESS;

}

/********************************FIT HANDLER********************************************/
/**
*  Interrupt handler for the 5kHz fixed interval timer
*  The handler increments the fit count and resets the fit count to 0 when it gets greater than high_fit_count
*  The value of high_fit_count is initialized, however it changes based on the rotary count value
* @param None

*****************************************************************************/
void FIT_Handler(void)
{
	fit_count++;

	if (fit_count >= high_fit_count)
	{
		fit_count = 0; //reset the fit count
		fit_flag = 1;
	}

}
