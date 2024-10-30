/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

//CONSTANTS
#define PTB0_Pin 0 // PortD Pin 0
#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define UART2_RX_PIN 23 // UART2 RX Pin (PTE23)
#define UART2_TX_PIN 22 // UART2 TX Pin (PTE22)
#define Q_SIZE 32 // Queue Size

//Sharlyn's Constants
#define GLED1 7 //CHANGE to Portc Pin X
#define GLED2 0 //CHANGE to Portc Pin X
#define GLED3 3 //CHANGE to PortC Pin X
#define RLED1 4 //CHANGE to PortC Pin X
#define RLED2 5 //CHANGE to PortC Pin X
#define RLED3 6 //CHANGE to PortC Pin X
#define MASK(x) (1 << (x)) // Shift 1 to the left for x times
#define MOVE 1000 //CHANGE to 500ms
#define STOP 2000 //CHANGE to 200ms

#define MASK(x) (1 << (x))

const osThreadAttr_t finishTuneThreadAttr = {
    .name = "FinishTuneThread",
	.priority = osPriorityHigh
};
const osThreadAttr_t motorThreadAttr = {
    .name = "MotorThread",
    .priority = osPriorityNormal
};
const osThreadAttr_t runningTuneThreadAttr = {
    .name = "RunningTuneThread",
    .priority = osPriorityNormal
};
const osThreadAttr_t brainAttr = {
    .name = "BrainThread",
    .priority = osPriorityNormal
};
const osThreadAttr_t GLedSwitchAttr = {
    .name = "GLedSwitchThread",
    .priority = osPriorityNormal
};
const osThreadAttr_t GLedAllAttr = {
    .name = "GLedAllThread",
    .priority = osPriorityNormal
};
const osThreadAttr_t RLedMoveAttr = {
    .name = "RLedMoveThread",
    .priority = osPriorityNormal
};
const osThreadAttr_t RLedStopAttr = {
    .name = "RLedStopThread",
    .priority = osPriorityNormal
};

/* DELAY Function */
static void delay(volatile uint32_t nof){
	while(nof!= 0){
		__ASM("NOP");  nof--;
	}
}

/*------------------------*
    QUEUE Data Structure
*------------------------*/
// Queue Structure
typedef struct {
    unsigned char Data[Q_SIZE];
    unsigned int Head;
    unsigned int Tail;
    unsigned int Size;
} Q_T;

// Queue Functions
void Q_Init(Q_T *q) {
    unsigned int i;
    for (i = 0; i < Q_SIZE; i++)
        q->Data[i] = 0;  // to simplify our lives when debugging
    q->Head = 0;
    q->Tail = 0;
    q->Size = 0;
}

int Q_Empty(Q_T *q) {
    return q->Size == 0;
}

int Q_Full(Q_T *q) {
    return q->Size == Q_SIZE;
}

int Q_Enqueue(Q_T *q, unsigned char d) {
    if (!Q_Full(q)) {
        q->Data[q->Tail++] = d;
        q->Tail %= Q_SIZE;
        q->Size++;
        return 1;  // success
    } else {
        return 0;  // failure
    }
}

unsigned char Q_Dequeue(Q_T *q) {
    unsigned char t = 0;
    if (!Q_Empty(q)) {
        t = q->Data[q->Head];
        q->Data[q->Head++] = 0;  // to simplify debugging
        q->Head %= Q_SIZE;
        q->Size--;
    }
    return t;
}

// Queue Instance
Q_T RxQ;

/*-----------------------*
    Message Struct
*------------------------*/
typedef struct 
{
    int8_t x_data;
    int8_t y_data;
} message_t;

typedef struct
{
    uint8_t data;
} uart_message_t;

/*-------------------------*
      GLOBAL VARIABLES
*--------------------------*/

/*----------------------------------------------------------------------------
 * Application main thread. With threads: motor_thread
 *---------------------------------------------------------------------------*/
osThreadId_t runningTune_handle, finishTune_handle, motor_handle, brain_handle, GLedSwitch_handle, GLedAll_handle, RLedMove_handle, RLedStop_handle;
osEventFlagsId_t finishFlag, ledFlag;
osMessageQueueId_t xyMessage, uartMessage;

/*------------------------*
    INTERRUPTS and IRQs
*------------------------*/
// UART Interrupt Handler
void UART2_IRQHandler(void) {
    // Check if receive data register is full
    uart_message_t myUARTData;
    NVIC_ClearPendingIRQ(UART2_IRQn);
    if (UART2->S1 & UART_S1_RDRF_MASK) {
        // Received a character
        myUARTData.data = UART2->D;
        osMessageQueuePut(uartMessage, &myUARTData, NULL, 0);
    }
}

/*---Initialization----*/
void initUART2(void) {
    PORTE->PCR[UART2_TX_PIN] = PORT_PCR_MUX(4);
    PORTE->PCR[UART2_RX_PIN] = PORT_PCR_MUX(4);

    uint32_t bus_clock = SystemCoreClock / 2;
    uint16_t baud_divisor = (bus_clock) / (16 * 19200);

    UART2->BDH = (baud_divisor >> 8) & UART_BDH_SBR_MASK;
    UART2->BDL = baud_divisor & UART_BDL_SBR_MASK;

    UART2->C1 = 0x00;
    UART2->C3 = 0x00;
    UART2->S2 = 0x00;

    // Enable UART2 receiver and transmitter
    UART2->C2 |= UART_C2_RE_MASK | UART_C2_TE_MASK;

    // Enable UART2 interrupts in NVIC
    NVIC_SetPriority(UART2_IRQn, 128); 
    NVIC_ClearPendingIRQ(UART2_IRQn); 
    NVIC_EnableIRQ(UART2_IRQn);

    // Enable receive interrupt for UART2
    UART2->C2 |= UART_C2_RIE_MASK;

    // Initialize the receive queue
    Q_Init(&RxQ);
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
	//PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK; //CLEAR MUX BITS FOR RED LED
	//PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1); //SET MUX TO 001 FOR RED LED
	//PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK; 
	//PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1); 
    // Set Data Direction Registers for PortB and PortD
	//PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));;
	//set pins to high
	//PTB->PDOR |= (MASK(RED_LED) | MASK(GREEN_LED));	

    // Configure MUX to make all 6 pins GPIO
	PORTC->PCR[GLED1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GLED1] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK; // pull up resistor

	PORTC->PCR[GLED2] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GLED2] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK; // pull up resistor

	PORTC->PCR[GLED3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GLED3] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK; // pull up resistor

	PORTC->PCR[RLED1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RLED1] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK; // pull up resistor

	PORTC->PCR[RLED2] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RLED2] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK; // pull up resistor;

	PORTC->PCR[RLED3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RLED3] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK; // pull up resistor;

	// Set Data Direction Registers to make output
	PTC->PDDR |= (MASK(GLED1) | MASK(GLED2) | MASK(GLED3) | MASK(RLED1) | MASK(RLED2) | MASK(RLED3));
}

void initGPIO(void) {
    //Enable Port Clocks
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; //enable port B previously in Aaron code is SIM_SCGC5
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; //enable port D
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; //enable port C
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK; //enable UART2
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; //enable port E
    initTPM(); //initialize TPM for everything
    initBuzzer(); //PTB Pin 0 and TPM1 Channel 0
    initMotor(); //PTD Pin 0,1,2,3 and TPM0 Channel 0,1,2,3
    initLED(); //PTB Pin 18,19. GPIO Out, Init=High.
    initUART2();
}


/*---Functions---*/
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

/*------------------------*
    THREADS
*------------------------*/
__NO_RETURN void GLedSwitch_thread(void *arguments) {
    /* Turns on green LED one by one when moving */
	for (;;) {

		// wait for led_flag to be 1 for MOVING
		osEventFlagsWait(ledFlag, 0x0001, osFlagsNoClear, osWaitForever); // not sure if it is WaitAny

		/* checks which LED is on and turn on the next LED */
		if (PTC->PDOR & MASK(GLED1)) {
			PTC->PCOR = (MASK(GLED1)); // off GLED1
			PTC->PCOR = (MASK(GLED3)); // off GLED3
			PTC->PSOR = (MASK(GLED2)); // on GLED2
			osDelay(2000);
		} else if (PTC->PDOR & MASK(GLED2)) {
			PTC->PCOR = (MASK(GLED2)); // off GLED2
			PTC->PCOR = (MASK(GLED1)); // off GLED2
			PTC->PSOR = (MASK(GLED3)); // on GLED3
			osDelay(2000);
		} else {
			PTC->PCOR = (MASK(GLED1)); // off GLED3
			PTC->PCOR = (MASK(GLED2)); // off GLED2
			PTC->PSOR = (MASK(GLED3)); // on GLED 1
			osDelay(2000);
		}
	}
}

__NO_RETURN void GLedAll_thread(void *arguments) {
    /* Keep ALL green LED on when stationary */
	for (;;) {
		// wait for led_flag to be 2 for STOP
		osEventFlagsWait(ledFlag, 0x0002, osFlagsNoClear, osWaitForever);
		PTC->PSOR = (MASK(GLED1)); // on GLED1
		PTC->PSOR = (MASK(GLED3)); // on GLED3
		PTC->PSOR = (MASK(GLED2)); // on GLED2
	}
}

__NO_RETURN void RLedMove_thread(void *arguments) {
    /* Flash red LEDs at 500ms when moving */
	for (;;) {
		// wait for led_flag to be 1 for MOVING
		osEventFlagsWait(ledFlag, 0x0001, osFlagsNoClear, osWaitForever);
		PTC->PSOR = (MASK(RLED1)); // on RLED1
		PTC->PSOR = (MASK(RLED2)); // on RLED2
		PTC->PSOR = (MASK(RLED3)); // on RLED3
		osDelay(MOVE);
		PTC->PCOR = (MASK(RLED1)); // off RLED1
		PTC->PCOR = (MASK(RLED2)); // off RLED2
		PTC->PCOR = (MASK(RLED3)); // off RLED3
		osDelay(MOVE);
	}
} 

__NO_RETURN void RLedStop_thread(void *arguments) {
    /* Flash red LEDs at 250ms when stationary */
	for (;;) {
		// wait for led_flag to be 2 for STOP
		osEventFlagsWait(ledFlag, 0x0002, osFlagsNoClear, osWaitForever);
		PTC->PSOR = (MASK(RLED1)); // on RLED1
		PTC->PSOR = (MASK(RLED2)); // on RLED2
		PTC->PSOR = (MASK(RLED3)); // on RLED3
		osDelay(STOP);
		PTC->PCOR = (MASK(RLED1)); // off RLED1
		PTC->PCOR = (MASK(RLED2)); // off RLED2
		PTC->PCOR = (MASK(RLED3)); // off RLED3
		osDelay(STOP);
	}
}

__NO_RETURN void runningTune_thread(void* arguments) {	
	for(;;) {
		PlayC();
		osDelay(24000000);
		Stop();
		osDelay(24000000);
		
		// onLED(1, GREEN_LED);
		// osDelay(1000000);
		// offLED(1, GREEN_LED);
		// osDelay(1000000);
		// osEventFlagsSet(finishFlag, 1);
	}
}

__NO_RETURN void finishTune_thread(void* arguments) { 
	for(;;) {	
		osEventFlagsWait(finishFlag, 1, osFlagsWaitAny, 	osWaitForever); //wait for the finish flag to be set
		PlayG();
		osDelay(12000000);
		PlayA();
		osDelay(12000000);
		PlayB();
		osDelay(12000000);
		Stop();
		osDelay(12000000);
		
		// onLED(1, RED_LED);
		// osDelay(1000000);
		// offLED(1, RED_LED);
		// osDelay(1000000);
        
        //
	}
}

__NO_RETURN void motor_thread (void *argument) {
    message_t myXYData;
    while(1) {
        //say the input -3,...,3 received from uart is in var X and Y
        osMessageQueueGet(xyMessage, &myXYData, NULL, osWaitForever);
        
        int Yval = myXYData.y_data * 1000;
        if (Yval == 1000)
            Yval = 2000;
        else if (Yval == 2000)
            Yval = 2500;
        else if (Yval == -1000)
            Yval = -2000;
        else if (Yval == -2000)
            Yval = -2500;
        
        int Xval = myXYData.x_data * 1000;
        if (Xval == 1000)
            Xval = 2000;
        else if (Xval == 2000)
            Xval = 2500;
        else if (Xval == -1000)
            Xval = -2000;
        else if (Xval == -2000)
            Xval = -2500;
        
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
                osEventFlagsClear(ledFlag, 1);
                osEventFlagsSet(ledFlag, 2);
        }
        else {
            osEventFlagsClear(ledFlag, 2);
            osEventFlagsSet(ledFlag, 1);
        }
        // Adding a delay to avoid hogging the CPU
        osDelay(1);
    }
}

__NO_RETURN void brain_thread(void *argument) {
    message_t myXYData;
    uart_message_t myUARTData;
    while (1) {
        osMessageQueueGet(uartMessage, &myUARTData, NULL, osWaitForever);
        //parse new data
        int8_t stop_data = myUARTData.data & (0b00000001);
        myXYData.x_data = ( myUARTData.data & (0b00000110) ) >> 1;
        if (myUARTData.data & (0b00001000)) {
            myXYData.x_data = -myXYData.x_data;
        }
        myXYData.y_data = ( myUARTData.data & (0b00110000) ) >> 4;
        if (myUARTData.data & (0b01000000)) {
            myXYData.y_data = -myXYData.y_data;
        }

        if (stop_data == 1) {
            osEventFlagsSet(finishFlag, 1);
        }
        osMessageQueuePut(xyMessage, &myXYData, NULL, 0);
        // Adding a delay to avoid hogging the CPU
        osDelay(1);
    }
}

void initEventFlags(void) {
    finishFlag = osEventFlagsNew(NULL);
    ledFlag = osEventFlagsNew(NULL); // need to set led_flag to 0x0001 when MOVING and to 0x0002 when STOP
}

void initMessageQueue(void) {
    xyMessage = osMessageQueueNew(3, sizeof(message_t), NULL);
    uartMessage = osMessageQueueNew(3, sizeof(uart_message_t), NULL);
}

__NO_RETURN void app_main(void *argument) {

    initEventFlags();

    initMessageQueue();

    runningTune_handle   = osThreadNew(runningTune_thread, NULL, &runningTuneThreadAttr); //create thread for running tune
    finishTune_handle    = osThreadNew(finishTune_thread, NULL, &finishTuneThreadAttr); //finish thread set to higher priority to cut in when flag is set
    motor_handle         = osThreadNew(motor_thread, NULL, &motorThreadAttr);
    brain_handle         = osThreadNew(brain_thread, NULL, &brainAttr);
    GLedSwitch_handle      = osThreadNew(GLedSwitch_thread, NULL, &GLedSwitchAttr);
    GLedAll_handle        = osThreadNew(GLedAll_thread, NULL, &GLedAllAttr);
    RLedMove_handle       = osThreadNew(RLedMove_thread, NULL, &RLedMoveAttr);
    RLedStop_handle       = osThreadNew(RLedStop_thread, NULL, &RLedStopAttr);
    for(;;) {}
}

int main (void) {
    // System Initialization
    SystemCoreClockUpdate();
    initGPIO(); //initialize everything: buzzer, motor, LED, UART

    osKernelInitialize();	
    osThreadNew(app_main, NULL, NULL);
    osKernelStart();
}
