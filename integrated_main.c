/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define PTB0_Pin 0 // PortD Pin 0
#define MASK(x) (1 << (x))

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19

osThreadId_t runningTuneId, finishTuneId, motorId;
osEventFlagsId_t finishFlag;

//CONSTANTS
const osThreadAttr_t highPrio = {
	.priority = osPriorityHigh
};

/*----------------------------------------------------------------------------
 * Application main thread. With threads: motor_thread
 *---------------------------------------------------------------------------*/

void initGPIO(void) {
    //Enable Port Clocks
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; //enable port B previously in Aaron code is SIM_SCGC5
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; //enable port D
    initTPM(); //initialize TPM for everything
    initBuzzer(); //PTB Pin 0 and TPM1 Channel 0
    initMotor(); //PTD Pin 0,1,2,3 and TPM0 Channel 0,1,2,3
    initLED(); //PTB Pin 18,19. GPIO Out, Init=High.
}
void initTPM(void) {
    /*-----TPM 0-----*/
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; //enable TPM0 module
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
    
    /*-----TPM 1-----*/
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; //enable TPM1 module

    TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));  //prescale factor 128
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

    /*-----Set TPM clk source-----*/
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //set TPM clock source
}

void initMotor(void) {
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
    
    //SET Mod value for TPM0
    TPM0->MOD = 6000;
}

void initBuzzer(void)
{
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3); //set to GPIO
}

void initLED(void) {
    //** ENABLING LEDS TO TEST **//
	// Configure MUX settings to make all 3 pins GPIO 
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK; //CLEAR MUX BITS FOR RED LED
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1); //SET MUX TO 001 FOR RED LED
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK; 
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1); 
    // Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));;
	//set pins to high
	PTB->PDOR |= (MASK(RED_LED) | MASK(GREEN_LED));	
}

__NO_RETURN void motor_thread (void *argument) {
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

void onLED(int port, int ID_LED){
	//set pin to low
	switch(port) {  
		case 0:   
			PTA->PDOR &= ~MASK(ID_LED);
		break;
		case 1:   
			PTB->PDOR &= ~MASK(ID_LED);
		break;
		case 2:   
			PTC->PDOR &= ~MASK(ID_LED);
		break;
		case 3:   
			PTD->PDOR &= ~MASK(ID_LED);
		break;
	}
}

void offLED(int port, int ID_LED) {
	//set pin to high
	switch(port) {
		case 0:   //PORTA
		 PTA->PDOR |= MASK(ID_LED); 
		break;
		case 1:
		 PTB->PDOR |= MASK(ID_LED); 
		break;
		case 2:
			PTC->PDOR |= MASK(ID_LED); 
		break;
		case 3:
			PTD->PDOR |= MASK(ID_LED);
		break;
	}
}

/* DELAY Function */
static void delay(volatile uint32_t nof){
	while(nof!= 0){
		__ASM("NOP");  nof--;
	}
}

void PlayC(void) {
	TPM1->MOD = 1432; //controls frequency generated freq desired
	TPM1_C0V = TPM1->MOD / 2; //controls duty cycle relative to frequency, e.g. 50% duty cycle = TPM1->MOD / 2

}

void PlayD(void) {
    TPM1->MOD = 1276;
    TPM1_C0V = TPM1->MOD / 4;
}

void PlayE(void) {
    TPM1->MOD = 1136;
    TPM1_C0V = TPM1->MOD / 4;
}

void PlayF(void) {
    TPM1->MOD = 1075;
    TPM1_C0V = TPM1->MOD / 4;
}

void PlayG(void) {
    TPM1->MOD = 956;
    TPM1_C0V = TPM1->MOD / 4;
}

void PlayA(void) {
    TPM1->MOD = 852;
    TPM1_C0V = TPM1->MOD / 4;
}

void PlayB(void) {
    TPM1->MOD = 759;
    TPM1_C0V = TPM1->MOD / 4;
}

void Stop(void) {
	TPM1->MOD = 0;
	TPM1_C0V = 0;
}

__NO_RETURN void playRunningTune_thread(void* arguments) {	
	for(;;) {
		//PlayC();
		//osDelay(24000000);
		//Stop();
		//osDelay(24000000);
		
		onLED(1, GREEN_LED);
		osDelay(1000000);
		offLED(1, GREEN_LED);
		osDelay(1000000);
		osEventFlagsSet(finishFlag, 1);
	}
}

__NO_RETURN void playFinishTune_thread(void* arguments) { 
	for(;;) {	
		osEventFlagsWait(finishFlag, 1, osFlagsWaitAny, 	osWaitForever); //wait for the finish flag to be set
		//PlayG();
		//osDelay(12000000);
		//PlayA();
		//osDelay(12000000);
		//PlayB();
		//osDelay(12000000);
		//Stop();
		//osDelay(12000000);
		
		onLED(1, RED_LED);
		osDelay(1000000);
		offLED(1, RED_LED);
		osDelay(1000000);
	
	}
}

int main (void) {
 
    // System Initialization
    SystemCoreClockUpdate();
    initGPIO(); //initialize everything including buzzer and motor

	//Kernel Initialisation
    osKernelInitialize();	
    finishFlag = osEventFlagsNew(NULL); 
    osEventFlagsClear(finishFlag, 1); //clear the flag
    runningTuneId = osThreadNew(playRunningTune_thread, NULL, NULL); //create thread for running tune
    finishTuneId = osThreadNew(playFinishTune_thread, NULL, &highPrio); //finish thread set to higher priority to cut in when flag is set
    motorId = osThreadNew(motor_thread, NULL, NULL);
    osKernelStart();                      
    for (;;) {}
}
