#include "MKL25Z4.h"
#include "cmsis_os2.h" // Include RTX/CMSIS RTOS header for threading

#define MASK(x) (1 << (x))
#define UART2_RX_PIN 23 // UART2 RX Pin (PTE23)
#define UART2_TX_PIN 22 // UART2 TX Pin (PTE22)
#define Q_SIZE 32

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

// Function Prototypes
void UART2_init(void);
void UART2_Transmit_Poll(char data);
void UART2_IRQHandler(void);
void uart_thread(void *argument);  // UART thread function

// Define the thread ID
osThreadId_t uartThreadId;

// Thread attributes for UART thread
const osThreadAttr_t uartThreadAttr = {
    .name = "UARTThread",
    .priority = osPriorityNormal
};

int main(void) {
    // Initialize CMSIS-RTOS
    osKernelInitialize();

    // Initialize UART
    UART2_init();

    // Create UART thread
    uartThreadId = osThreadNew(uart_thread, NULL, &uartThreadAttr);

    // Start RTOS Kernel
    osKernelStart();

    // Should never reach here
    while (1) {}
}

// UART Thread Function
void uart_thread(void *argument) {
    while (1) {
        // Check if data is available in the Rx queue
        if (!Q_Empty(&RxQ)) {
            char received = Q_Dequeue(&RxQ);  // Dequeue received character
            UART2_Transmit_Poll(received);    // Transmit received character back

            // Add any additional processing here (e.g., LED control)
        }

        // Adding a delay to avoid hogging the CPU
        osDelay(1);
    }
}

// UART Initialization
void UART2_init(void) {
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

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

// UART Interrupt Handler
void UART2_IRQHandler(void) {
    // Check if receive data register is full
    NVIC_ClearPendingIRQ(UART2_IRQn);
    if (UART2->S1 & UART_S1_RDRF_MASK) {
        // Received a character
        if (!Q_Full(&RxQ)) {
            Q_Enqueue(&RxQ, UART2->D);
        } else {
            // Error - queue full, data lost
        }
    }
}

// UART Polling Transmit
void UART2_Transmit_Poll(char data) {
    while (!(UART2->S1 & UART_S1_TDRE_MASK));
    UART2->D = data;  // Send back the received data
}

// Delay Function (for debugging purposes)
static void delay(volatile uint32_t nof) {
    while (nof != 0) {
        __asm("NOP");
        nof--;
    }
}
