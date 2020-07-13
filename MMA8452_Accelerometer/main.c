/*********************************************************************************************
 * MAIN file for MMA8452_Accelerometer
 * Works with Sparkfun MMA8452 breakout board, ACCELEROMETER_ID = 0x1D, I2C Bus #1 
 * Microcontroller: PIC32MX795F512X 
 * Compiler: XC32 V1.30 IDE: MPLABX V5.15
 * 
 * Code adapted from Arduino MMA8452 demo .
 * 7-13-20 Adapted from code written for PIC 32MX220. Works with SnadPIC board.
 * NO INTERRUPTS in this version. 
 * Detects motion using MMA8452, transmits XYZ position data to UART 2 at 19200 baud.
 **********************************************************************************************************/
#include <plib.h>
#include <string.h>
#include <ctype.h>

#include "MMA8452.h"
#include "Delay.h"

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR 

#define true TRUE
#define false FALSE

// Used SNAD PIC BOARD with 8 Mhz crystal, this should yield 80 Mhz system & peripheral clock
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx1      // ICE/ICD Comm Channel Select

static void InitializeSystem(void);

#define MIN_CHANGE 128
#define OpenI2C OpenI2C1

int main(void) 
{
    short rawVectx, rawVecty, rawVectz;
    short PreviousX, PreviousY, PreviousZ;
    unsigned char accelerometerBuffer[MAX_I2C_REGISTERS];
    
    InitializeSystem();
    DelayMs(100);
    
    printf("\r\r>Project: MMA8452_Accelerometer");
    printf("\rTesting Sparkfun MMA8452 accelerometer...");
    printf("\rInitializing I2C BUS #1");
    
    #define OpenI2C OpenI2C1    
    OpenI2C(I2C_EN, 299);  
    
    printf("\rInitializing MMA8452...");
    if (initMMA8452()) 
        printf("\rMMA8452 Initialized OK!");
    else
    {
        printf("\r\rERROR: MMA8452 failed init\r\r");
        while(1);
    }
    printf("\rMIN CHANGE = %d", MIN_CHANGE);
    DelayMs(100);
        
    printf("\rTesting MMA8452, no interrupts\r\r");
        
    while(1)
    {                 
        DelayMs(10);
        readRegisters(ACCELEROMETER_ID, 0x01, MAX_I2C_REGISTERS, accelerometerBuffer);
        rawVectx = convertValue(accelerometerBuffer[0], accelerometerBuffer[1]);
        rawVectz = convertValue(accelerometerBuffer[2], accelerometerBuffer[3]);
        rawVecty = convertValue(accelerometerBuffer[4], accelerometerBuffer[5]);
        
        if (abs(PreviousX - rawVectx) > MIN_CHANGE || abs(PreviousY - rawVecty) > MIN_CHANGE || abs(PreviousZ - rawVectz) > MIN_CHANGE)
        {
            printf("\rX: %d, Y: %d, Z: %d", rawVectx, rawVecty, rawVectz);
            PreviousX = rawVectx;
            PreviousY = rawVecty;
            PreviousZ = rawVectz;
        }
    }//end while
}//end main



static void InitializeSystem(void) {
    SYSTEMConfigPerformance(80000000); // was 60000000

    mJTAGPortEnable(false);
    
    // Set up main UART ONLY TX USED IN THIS DEMO
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define HOSTuart UART2
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 19200);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    // INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);        

    // NO INTERRUPTS
    // INTEnableSystemMultiVectoredInt();
    
}//end UserInit


