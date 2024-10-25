/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define PTB0_Pin 0 // PortD Pin 1
#define MASK(x) (1 << (x))

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19

osThreadId_t runningTuneThread, finishTuneThread;
osEventFlagsId_t finishFlag;

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void initBuzzer(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; //enable port B
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3); //set to GPIO
	
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; //enable TPM1 module
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //set TPM clock source
		
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));  //prescale factor 128
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	
	
	//** ENABLING LEDS TO TEST **//
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK)); 
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

void playRunningTune(void* arguments) {	
	for(;;) {
		//PlayC();
		//osDelay(24000000);
		//Stop();
		//osDelay(24000000);
		
		onLED(1, GREEN_LED);
		delay(1000000);
		offLED(1, GREEN_LED);
		delay(1000000);
		osEventFlagsSet(finishFlag, 1);
	}
}

void playFinishTune(void* arguments) { 
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
		delay(1000000);
		offLED(1, RED_LED);
		delay(1000000);
	
	}
}


const osThreadAttr_t highPrio = {
	.priority = osPriorityHigh
};

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initBuzzer();


	//*** NEED AN ISR TO TRIGGER AND SET FINISHFLAG ***
	//Stop();
  // ...
 
	//Kernel Initialisation
  osKernelInitialize();	
	finishFlag = osEventFlagsNew(NULL); 
	osEventFlagsClear(finishFlag, 1); //clear the flag
	runningTuneThread = osThreadNew(playRunningTune, NULL, NULL); //create thread for running tune
	finishTuneThread = osThreadNew(playFinishTune, NULL, &highPrio); //finish thread set to higher priority to cut in when flag is set
  osKernelStart();                      
  for (;;) {}
}
