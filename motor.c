	/*----------------------------------------------------------------------------
	 * CMSIS-RTOS 'main' function template
	 *---------------------------------------------------------------------------*/
	 
	#include "RTE_Components.h"
	#include  CMSIS_device_header
	#include "cmsis_os2.h"

	/*----------------------------------------------------------------------------
	 * Application main thread
	 *---------------------------------------------------------------------------*/
	 
	 // use portD, 0 1 2 3
	void InitGPIO() {
		//Turn on portd clock
		SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
		
		//clear function for portd 0,1,2 and 3
		PORTD->PCR[0] &= ~PORT_PCR_MUX_MASK;
		PORTD->PCR[1] &= ~PORT_PCR_MUX_MASK;
		PORTD->PCR[2] &= ~PORT_PCR_MUX_MASK;
		PORTD->PCR[3] &= ~PORT_PCR_MUX_MASK;
		
		//turn on pwm for portd 0,1, 2 and 3
		PORTD->PCR[0] |= PORT_PCR_MUX(4);
		PORTD->PCR[1] |= PORT_PCR_MUX(4);
		PORTD->PCR[2] |= PORT_PCR_MUX(4);
		PORTD->PCR[3] |= PORT_PCR_MUX(4);
		
		//turn on tpm0 clock
		SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
		
		//turn on sopt??
		SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
		SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
		
		//clear prescaler and cmod for TPM0
		TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
		
		//Set prescalar to 32 and cmod to use internal clock for TPM0
		TPM0->SC |= TPM_SC_PS(5);
		TPM0->SC |= TPM_SC_CMOD(1);
		
		//ENABLE EDGE ALIGNED PWM for TPM0
		TPM0->SC &= ~TPM_SC_CPWMS_MASK;
		
		//channel 0 config for edge-aligned pwm
		TPM0_C0SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
		TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
		
		TPM0_C1SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
		TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
		
		TPM0_C2SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
		TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
		
		TPM0_C3SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
		TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
		
		//SET Mod value for TPM0
		TPM0->MOD = 6000;
		
	}
	//to turn on tpm2
	//tpm2_CnV = value

	void motor_thread (void *argument) {
		//say the input -3,...,3 received from uart is in var X and Y
		int Yval = Y * 1000;
		int Xval = X * 1000;
		
		//this is the X,Y pos decoder to Left, Right Wheel Vals
		int leftMotorValue = Yval + Xval;
		int rightMotorValue = Yval - Xval;
		
		
		//Channel 0 and 1 is left motor, Channel 2 and 3 is right motor
		//If C0 High and C1 Low, motor moves forward.
		//If C0 Low and C0 High, motor moves backward.
		//Same for C2 and C3.
		if (leftMotorValue > 0) {
			TPM0_C0V = leftMotorValue;
			TPM0_C1V = 0;
		}
		else if (leftMotorValue < 0) {
			TPM0_C0V = 0;
			TPM0_C1V = leftMotorValue;
		}
		if (rightMotorValue > 0) {
			TPM0_C2V = rightMotorValue;
			TPM0_C3V = 0;
		}
		else if (rightMotorValue < 0) {
			TPM0_C2V = 0;
			TPM0_C3V = rightMotorValue;
		}
		
		if (rightMotorValue == 0 && leftMotorValue == 0) {
			TPM0_C0V = 0;
			TPM0_C1V = 0;
			TPM0_C2V = 0;
			TPM0_C3V = 0;
		}
	}

	void app_main (void *argument) {
		
		// ...
		for (;;) {
			
		}
	}
	 
	int main (void) {
	 
		// System Initialization
		SystemCoreClockUpdate();
		// ...
		InitGPIO();
		
	 
		osKernelInitialize();                 // Initialize CMSIS-RTOS
		osThreadNew(app_main, NULL, NULL);    // Create application main thread
		osKernelStart();                      // Start thread execution
		for (;;) {}
	}
