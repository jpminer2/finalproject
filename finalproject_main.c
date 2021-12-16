//#############################################################################
// FILE:   labstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define RADIUS      (4.675/2.0/12.0) //radius of wheel in feet
#define WIDTH       (6.75/12.0)//width of robot
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);

void setupSpib(void);
void setupSpic(void);
void getBlocks(uint16_t sgn);

void serialRXA(serial_t *s, char data);
void serialRXC(serial_t *s, char data);
void processNum(void);

void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
void changeInputs(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

uint16_t sync1 = 0;
uint16_t sync2 = 0;
uint16_t type = 0;
uint16_t length = 0;
uint16_t checksum1 = 0;
uint16_t checksum2 = 0;
uint16_t hwv1 = 0;
uint16_t hwv2 = 0;
uint16_t fwv1 = 0;
uint16_t fwv2 = 0;
uint16_t fwb1 = 0;
uint16_t fwb2 = 0;
uint16_t fwt = 0;
uint16_t temp = 0;

uint16_t gb1 = 0;
uint16_t gb2 = 0;
uint16_t gb3 = 0;
uint16_t gb4 = 0;
uint16_t gb5 = 0;
uint16_t gb6 = 0;
uint16_t gb7 = 0;
uint16_t gb8 = 0;
uint16_t gb9 = 0;
uint16_t gb10 = 0;
uint16_t gb11 = 0;
uint16_t gb12 = 0;
uint16_t gb13 = 0;
uint16_t gbFlag = 0;
uint16_t blockX[7];
uint16_t blockY[7];
uint16_t blockWidth[7];
uint16_t blockHeight[7];

uint8_t sng1Flag = 0;
uint8_t sng2Flag = 0;
uint16_t sng1Count = 0;
uint8_t autoFlag = 0;
uint16_t area[7];
uint16_t area1[7];

uint16_t flag = 0;
uint16_t count = 0;
char str[11];
uint16_t num = 0;
uint16_t a = 0;
uint16_t b = 0;
uint16_t one = 0;
uint16_t two = 0;
uint16_t minus = 0;
uint16_t plus = 0;
uint16_t home = 0;
int16_t updown = 0;
int16_t leftright = 0;

uint16_t a1 = 0;
uint16_t b1 = 0;
uint16_t one1 = 0;
uint16_t two1 = 0;
uint16_t minus1 = 0;
uint16_t plus1 = 0;
uint16_t home1 = 0;
int16_t plusminus = 0;
int16_t plusminus1 = 0;

int16_t accelx = 0;
int16_t accely = 0;
int16_t accelz = 0;
int16_t gyrox = 0;
int16_t gyroy = 0;
int16_t gyroz = 0;
float gyroxReading = 0;
float gyroyReading = 0;
float gyrozReading = 0;
float accelxReading = 0;
float accelyReading = 0;
float accelzReading = 0;

float LeftWheel = 0;
float RightWheel = 0;

float LeftWheel1 = 0;
float RightWheel1 = 0;

float leftdistance = 0;
float rightdistance = 0;

float uLeft = 5.0;
float uRight = 5.0;

float leftdistance1 = 0;
float rightdistance1 = 0;

float leftvel = 0;
float rightvel = 0;

float kp = 3;
float ki = 25;

//reference velocities
float vref = 0;

//integrator terms right
float ikr = 0;
float ikr1 = 0;

//error terms right
float ekr = 0;
float ekr1 = 0;

//integrator terms left
float ikl = 0;
float ikl1 = 0;

//integrator terms left
float ekl = 0;
float ekl1 = 0;

//turning components
float eturn = 0;
float kturn = 3;

//turn setpoint
float turn = 0;

float x = 0;
float y = 0;
float phi = 0;
float xdot = 0;
float xdot1 = 0;
float ydot = 0;
float ydot1 = 0;

uint32_t numRXC = 0;


//tracking variables for the song
uint16_t songflag = 0;
uint16_t songcount = 0;

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;
    // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;
    // This is needed to disable write to EALLOW protected registers

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 300000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    setupSpib();
    setupSpic();

    init_serial(&SerialA, 115200, serialRXA);
    //raspberry pi serial
    init_serial(&SerialC, 115200, serialRXC);
//    init_serial(&SerialD,115200,serialRXD);

    init_eQEPs();

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTR = 0x0;
    EPwm2Regs.TBPRD = 2500;
    EPwm2Regs.CMPA.bit.CMPA = 1250;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.CMPB.bit.CMPB = 1250;
    EPwm2Regs.AQCTLB.bit.CBU = 1;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;
    EPwm2Regs.TBPHS.bit.TBPHS = 0;

    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);
    EPwm8Regs.TBCTL.bit.CTRMODE = 0;
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm8Regs.TBCTL.bit.PHSEN = 0;
    EPwm8Regs.TBCTL.bit.CLKDIV = 0;
    EPwm8Regs.TBCTR = 0x0;
    EPwm8Regs.TBPRD = 2500;
    EPwm8Regs.CMPA.bit.CMPA = 0;
    EPwm8Regs.AQCTLA.bit.CAU = 1;
    EPwm8Regs.AQCTLA.bit.ZRO = 2;
    EPwm8Regs.CMPB.bit.CMPB = 0;
    EPwm8Regs.AQCTLB.bit.CBU = 1;
    EPwm8Regs.AQCTLB.bit.ZRO = 2;
    EPwm8Regs.TBPHS.bit.TBPHS = 0;

    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5);
    EPwm9Regs.TBCTL.bit.CTRMODE = 0;
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm9Regs.TBCTL.bit.PHSEN = 0;
    EPwm9Regs.TBCTL.bit.CLKDIV = 1;
    EPwm9Regs.TBCTR = 0x0;
    EPwm9Regs.TBPRD = 1;
    EPwm9Regs.AQCTLA.bit.CAU = 0;
    EPwm9Regs.AQCTLA.bit.ZRO = 3;
    EPwm9Regs.TBPHS.bit.TBPHS = 0;

    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;    // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;    // For EPWM8B
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1;    // For EPWM9A
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;    // For EPWM12A

    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6; //SPI

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //Enable Interrupt for SPI
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM

    // IDLE loop. Just sit and loop forever (optional):
    while (1)
    {
        if (UARTPrint == 1)
        {
            //prints the coordinates
            serial_printf(&SerialA,
                          "%s x = %.2f, y = %.2f bearing = %.2f\n\r\n\r", str,
                          x, y, phi);
            UARTPrint = 0;
        }
    }
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void)
{

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");
    // Wait one cycle
    EINT;
    // Clear INTM to enable interrupts

    // Insert SWI ISR Code here.......

    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = 0xBA00; // reading from 3A
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;

    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    //prints the led array for each of the numbers in the class
    if ((numTimer0calls % 500) == 0)
    {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF)
        {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    //variables for average angular velocity
    float thetadotr = 0;
    float thetadotl = 0;
    float thetadot = 0;

    CpuTimer1.InterruptCount++;

    getBlocks((CpuTimer1.InterruptCount%3)+1); //will get the first 3 signatures dependent upon modulo


    //autonomous with auto flag and if it is not being steered
    if (autoFlag ==1 && updown == 0 && leftright == 0) {

        //computes areas of stop sign and turn
       area[0] = blockWidth[0] * blockHeight[0];
       area[1] = blockWidth[1] * blockHeight[1];

       //if yield sign detected
       if(blockX[2] != 257){
           plusminus = -6;
       }
       //if yield was detected and is not now
       else if(plusminus == -6){
           plusminus = 0;
       }

       //if stop sign timer counting up
       if ((sng1Flag == 1) && (sng1Count < 500))  {
          sng1Count++;
          vref = 0;
          turn = 0;

       }
       //if stop sign detected
       else if((sng1Flag == 0) && (area[0] >2500)) {
                    turn = 0.0;
                    vref = 0.0;
                    sng1Flag = 1;
                }
       //if follow sign is detected
       else if((sng2Flag==0) && (blockX[1] != 257)){
           //if it is to the left
           if(blockX[1] >200) {
               turn = 0.4;
               vref = 0.0;

           }
           //if it is to the right
           else if(blockX[1] < 150) {
               turn = -0.4;
               vref = 0.0;
           }
           //if it is centered
           else {
               turn = 0.0;
               vref = 0.5 + plusminus*0.06;
           }
           //if it is done following
           if (area[1] > 10000) {
               vref = 0;
               sng2Flag = 1;
           }
           //was used to detect if approaching
           //area1[1] = area[1];
       }
       //reset or default case
       else {
           //if follow sign not detected
           if (blockX[1] ==257){
               sng2Flag = 0;
               turn = 0;
               vref == 0;
           }
           //area1[1] = 0;

           //if stop sign not detected
           if(area[0] < 2000){
               sng1Flag = 0;
               sng1Count = 0;
           }
       }

    }

    //reading the angle from the encoders
    LeftWheel = readEncLeft();
    RightWheel = -1 * readEncRight();

    //computing angular velocity using a small time approximation
    thetadotl = (LeftWheel - LeftWheel1) / 0.004;
    thetadotr = (RightWheel - RightWheel1) / 0.004;
    thetadot = 0.5 * (thetadotl + thetadotr);

    //increments angular velocity
    LeftWheel1 = LeftWheel;
    RightWheel1 = RightWheel;

    //sets the distance traveled
    leftdistance = LeftWheel * RADIUS;
    rightdistance = RightWheel * RADIUS;

    //finds the bearing
    phi = RADIUS / WIDTH * (RightWheel - LeftWheel);

    //finds speed based on bearing and average angular velocity
    xdot = RADIUS * thetadot * cos(phi);
    ydot = RADIUS * thetadot * sin(phi);

    //integrates the velocities to find distances
    x += 0.004 * (xdot + xdot1) / 2;
    y += 0.004 * (ydot + ydot1) / 2;

    //increments velocity
    xdot1 = xdot;
    ydot1 = ydot;

    //computes velocity
    leftvel = (leftdistance - leftdistance1) / 0.004;
    rightvel = (rightdistance - rightdistance1) / 0.004;

    //increments distance
    leftdistance1 = leftdistance;
    rightdistance1 = rightdistance;

    eturn = turn + (leftvel - rightvel);
    //compution of transfer function
    ekl = vref - leftvel - kturn * eturn;

    //checks for saturation
    if (fabs(uLeft) < 10)
        ikl = ikl1 + 0.004 * (ekl + ekl1) / 2;

    //all in control law from lab
    uLeft = kp * ekl + ki * ikl;
    ekl1 = ekl;
    ikl1 = ikl;

    ekr = vref - rightvel + kturn * eturn;
    if (fabs(uRight) < 10)
        ikr = ikr1 + 0.004 * (ekr + ekr1) / 2;
    uRight = kp * ekr + ki * ikr;
    ekr1 = ekr;
    ikr1 = ikr;

    //setting motors to calculated value
    setEPWM2B(-1 * uLeft);
    setEPWM2A(uRight);
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    //detecting if the song flag is high from pressing a button
    if (songflag == 1)
    {
        //preventing out of bounds error and stopping song
        if (songcount == SONG_LENGTH)
        {
            songcount = 0;
            GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0);
            GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;

        }
        //incrementing
        else
        {
            EPwm9Regs.TBPRD = songarray[songcount];
            songcount++;
        }
    }
    //default or when song is to not be playing
    if (songflag == 0)
    {
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5);
        EPwm9Regs.TBPRD = 0;
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
        songcount = 0;
    }
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
}
__interrupt void SPIB_isr(void)
{
    uint16_t temp = 0;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    accelx = SpibRegs.SPIRXBUF;
    accely = SpibRegs.SPIRXBUF;
    accelz = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    gyrox = SpibRegs.SPIRXBUF;
    gyroy = SpibRegs.SPIRXBUF;
    gyroz = SpibRegs.SPIRXBUF;

    gyroxReading = gyrox * 250.0 / 32767.0;
    gyroyReading = gyroy * 250.0 / 32767.0;
    gyrozReading = gyroz * 250.0 / 32767.0;
    accelxReading = accelx / 32767.0 * 4.0;
    accelyReading = accely / 32767.0 * 4.0;
    accelzReading = accelz / 32767.0 * 4.0;
    //Dan Code
    /*spivalue1 = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO.
     spivalue2 = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO. First ADC Value
     spivalue3 = SpibRegs.SPIRXBUF; //Reading from DAN chip Second ADC Value

     //yvolts = toVolts(spivalue2);
     //xvolts = toVolts(spivalue3);

     GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO 9 high to end Slave Select.  Now to Scope. Later to deselect DAN28027
     // Later when actually communicating with the DAN28027 do something with the data.  Now do nothing.*/

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;  // Acknowledge INT6 PIE interrupt

}

// This function is called each time a char is received over UARTA.
void serialRXA(serial_t *s, char data)
{
    numRXA++;

}

// This function is called each time a char is received over UARTC.
void serialRXC(serial_t *s, char data)
{
    //counts num of times it is called
    numRXC++;

    //detects start code
    if (data == '!')
    {
        //tells to receive data
        flag = 1;
        //empties previous code
        strcpy(str, "");
    }
    //take in data case
    else if (flag == 1)
    {
        //detects stop code
        if (data == '\n')
        {
            //puts count at 0, empties array, resets flag, and processes num
            count = 0;
            num = strtol(str, NULL, 2);
            flag = 0;
            processNum();
        }
        else
        {
            //takes in data and increments
            str[count] = data;
            count++;
        }
    }

    // serial_printf(&SerialA, "%c%n", data);

}

void processNum(void)
{
    //increments variables into previous tracker
    a1 = a;
    b1 = b;
    one1 = one;
    two1 = two;
    minus1 = minus;
    plus1 = plus;
    home1 = home;

    //processes string from each bit
    a = num % 2;
    b = (num >> 1) % 2;
    one = (num >> 2) % 2;
    two = (num >> 3) % 2;
    minus = (num >> 4) % 2;
    plus = (num >> 5) % 2;
    home = (num >> 6) % 2;
    updown = (num >> 7) % 4;
    leftright = (num >> 9) % 4;

    //formats updown variable to be either 1 or -1
    if (updown == 2)
        updown = -1;

    //formats leftright variable to be either 1 or -1
    if (leftright == 2)
        leftright = -1;

    //switches direction so it can be output into motor
    updown *= -1;
    leftright *= -1;

    //prints number if it is nonzero
    if (num != 0)
        UARTPrint = 1;

    //calls the changeinputs in order to reset the vref, turn, songcount, etc...
    changeInputs();
}

//converts analog to digital volts
float toVolts(int adc)
{
    return adc / 4095.0 * 3.3;
}

void setupSpib(void)
{
    int16_t temp = 0;
    SpibRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in Reset

    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;  //The MPU-9250,  Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;  // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1;  // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1;  // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0;  // Disables the SPI interrupt

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period.  SPI base clock is
                                           // 50MHZ.  And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason

    SpibRegs.SPIFFTX.bit.SPIRST = 1; // Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1;    // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt.  RXFFST >= RXFFIL

    SpibRegs.SPIFFCT.bit.TXDLY = 0; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip

    SpibRegs.SPICCR.bit.SPISWRESET = 1;    // Pull the SPI out of reset

    SpibRegs.SPIFFTX.bit.TXFIFO = 1;    // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt.  !! I don’t think this is needed.  Need to Test

    SpibRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes interrupt.  This is just the initial setting for the register.  Will be changed below

    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected

    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15);  //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15);  //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15);  //Set GPIO65 pin to SPICLKB

    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F.  Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    // To address 00x13 write 0x00
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00

    SpibRegs.SPITXBUF = 0x1300;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0013;
    SpibRegs.SPITXBUF = 0x0200;
    SpibRegs.SPITXBUF = 0x0806;
    SpibRegs.SPITXBUF = 0x0000;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while (SpibRegs.SPIFFRX.bit.RXFFST != 7)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29.  Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A

    SpibRegs.SPITXBUF = 0x2300;
    SpibRegs.SPITXBUF = 0x408C;
    SpibRegs.SPITXBUF = 0x0288;
    SpibRegs.SPITXBUF = 0x0C0A;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while (SpibRegs.SPIFFRX.bit.RXFFST != 4)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = 0x2A81;
    // wait for one byte to be received
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001);  // 0x3800
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001);  // 0x3A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001);  // 0x6400
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020);  // 0x6A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071);  // 0x7500
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00F7); // 0x7700
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x00F8); // 0x7800
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;

    //xOffset of -1028 (F7F8) Justin
    //xOffset of -2993 (E89E) Luke

    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00E6); // 0x7A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x00C4); // 0x7B00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;

    //yOffset of -3230 (E6C4) Justin
    //yOffset 924 (181C) Luke
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x001F); // 0x7D00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x002E); // 0x7E00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;

    //zOffset of 4264 (1F2E) Justin
    //zOffset of 3835 (1DF6) Luke
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;  // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

}


//setup code for SPIC that is used for Pixy comm

void setupSpic(void)
{
    int16_t temp = 0;
    SpicRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in Reset
    SpicRegs.SPICTL.bit.CLK_PHASE = 1; //Pixy uses Mode 03 where Phase =1 and Polarity =0
    SpicRegs.SPICCR.bit.CLKPOLARITY = 0;  //
    SpicRegs.SPICTL.bit.MASTER_SLAVE = 1;  // Set to SPI Master
    //SpicRegs.SPICCR.bit.SPICHAR = 7; // Set to transmit and receive 8 bits each write to SPITXBUF
    SpicRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpicRegs.SPICTL.bit.TALK = 1;  // Enable transmission
    SpicRegs.SPIPRI.bit.FREE = 1;  // Free run, continue SPI operation
    SpicRegs.SPICTL.bit.SPIINTENA = 0;  // Disables the SPI interrupt

    SpicRegs.SPIBRR.bit.SPI_BIT_RATE = 24; // Set SCLK bit rate to 1 MHz so 1us period.  SPI base clock is
                                           // 50MHZ.  And this setting divides that base clock to create SCLKs period 2Mhz Pixy com at 2Mb
    SpicRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason

    SpicRegs.SPIFFTX.bit.SPIRST = 1; // Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpicRegs.SPIFFTX.bit.SPIFFENA = 1;    // Enable SPI FIFO enhancements
    SpicRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpicRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpicRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpicRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpicRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpicRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt.  RXFFST >= RXFFIL

    SpicRegs.SPIFFCT.bit.TXDLY = 0; //Set delay between transmits to 16 spi clocks.

    SpicRegs.SPICCR.bit.SPISWRESET = 1;    // Pull the SPI out of reset

    SpicRegs.SPIFFTX.bit.TXFIFO = 1;    // Release transmit FIFO from reset.
    SpicRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpicRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt.  !! not sure this is needed.  Need to Test

    SpicRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes interrupt.  This is just the initial setting for the register.  Will be changed below

    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0); // Set as GPIO125 and used as Pixy SS
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO125 an Output Pin
    GpioDataRegs.GPDSET.bit.GPIO125 = 1; //Initially Set GPIO66/SS High so Pixy is not selected

    GPIO_SetupPinMux(122, GPIO_MUX_CPU1, 6);  //Set GPIO122 pin to SPISIMOC
    GPIO_SetupPinMux(123, GPIO_MUX_CPU1, 6);  //Set GPIO123 pin to SPISOMIC
    GPIO_SetupPinMux(124, GPIO_MUX_CPU1, 6);  //Set GPIO124 pin to SPICLKC

    // perform a multiple 16 bit transfer to initialize   Use only one SS low to high for all these writes

    /*GpioDataRegs.GPDCLEAR.bit.GPIO125 = 1;  // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.


    SpicRegs.SPITXBUF = 0xaec1;
    SpicRegs.SPITXBUF = 0x0e00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while (SpicRegs.SPIFFRX.bit.RXFFST != 14)
        ;
    temp = SpicRegs.SPIRXBUF;
    temp = SpicRegs.SPIRXBUF;
    temp = SpicRegs.SPIRXBUF;
    temp = SpicRegs.SPIRXBUF;
    temp = SpicRegs.SPIRXBUF;
    temp = SpicRegs.SPIRXBUF;
    sync1 = SpicRegs.SPIRXBUF;
    sync2 = SpicRegs.SPIRXBUF; //and type
    //type = SpicRegs.SPIRXBUF;
    length = SpicRegs.SPIRXBUF; //and checksum1
    checksum1 = SpicRegs.SPIRXBUF;
    checksum2 = SpicRegs.SPIRXBUF;
    hwv1 = SpicRegs.SPIRXBUF;
    hwv2 = SpicRegs.SPIRXBUF;
    fwv1 = SpicRegs.SPIRXBUF;
    fwv2 = SpicRegs.SPIRXBUF;
    fwb1 = SpicRegs.SPIRXBUF;
    fwb2 = SpicRegs.SPIRXBUF;
    fwt = SpicRegs.SPIRXBUF;

    GpioDataRegs.GPDSET.bit.GPIO125 = 1; // Slave Select High

    DELAY_US(10);*/ // Delay 10us to allow time to get ready for next transfer.


    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpicRegs.SPIFFRX.bit.RXFFOVFCLR = 1;  // Clear Overflow flag
    SpicRegs.SPIFFRX.bit.RXFFINTCLR = 1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

}

void init_eQEPs(void)
{

    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;    // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;    // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2;   // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0;    // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0;   // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0;      // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0;      // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0;        // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF;   // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1;    // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;    // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;    // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2;   // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0;   // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0;  // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0;     // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0;     // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0;       // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF;  // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1;   // Enable EQep
}

float readEncLeft(void)
{
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U

    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue / 2)
        raw -= QEP_maxvalue;  // I don't think this is needed and never true

    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev // of the DC motor's back shaft.  Then the gear motor's gear ratio is 30:1.
    return (raw * (1 / 30.0 / 20.0 * 2 * PI));
}

float readEncRight(void)
{

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U  -1 32bit signed int

    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue / 2)
        raw -= QEP_maxvalue;  // I don't think this is needed and never true

    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev // of the DC motor's back shaft.  Then the gear motor's gear ratio is 30:1.
    return (-1 * raw * (1 / 30.0 / 20.0 * 2 * PI));
}

void setEPWM2A(float controleffort)
{
    if (controleffort > 10)
    {
        controleffort = 10;

    }
    if (controleffort < -10)
    {
        controleffort = -10;

    }

    EPwm2Regs.CMPA.bit.CMPA = (int) (EPwm2Regs.TBPRD * (controleffort + 10)
            / 20.0);

}
void setEPWM2B(float controleffort)
{
    if (controleffort > 10)
    {
        controleffort = 10;

    }
    if (controleffort < -10)
    {
        controleffort = -10;
    }

    EPwm2Regs.CMPB.bit.CMPB = (int) (EPwm2Regs.TBPRD * (controleffort + 10)
            / 20.0);
}

void changeInputs(void)
{
    //if home button is pressed, reset speed controller
    if(home == 1){
        plusminus = 0;
    }
    //forwards
    if (updown == 1)
        vref = 2.0 + plusminus * .2;

    //backwards
    else if (updown == -1)
        vref = -2.0 - plusminus * .2;

    //not pressed
    else
        vref = 0;

    //right
    if (leftright == 1)
        turn = 1 + plusminus * .08;

    //left
    else if (leftright == -1)
        turn = -1 - plusminus * .08;

    //not pressed
    else
        turn = 0;

    //checks whether plus was just pressed and increments with saturation
    if (plus == 1 && plus1 == 0)
        if(plusminus<=5)
            plusminus++;

    //checks whether minus was just pressed and decrements with saturation
    if (minus == 1 && minus1 == 0){
        if(plusminus>=-9)
            plusminus--;
    }

    //checks if a was just pressed
    if (a == 1 && a1 == 0)
    {
        //stops song if it was started
        if (songflag == 1)
            songflag = 0;

        //starts song
        else if (songflag == 0)
            songflag = 1;

        //resets song count either way
        songcount = 0;
    }

    //checks if 1 was pressed and turns on auto mode
    if (one==1) {
        autoFlag = 1;
        sng2Flag = 0;
    }

    //checks if 2 was pressed and turns off auto mode
    if (two==1) {
        autoFlag = 0;
    }

    //if any of the autonomous flags are 1 and it is still counting then it can't move
    if((sng1Flag == 1) && (sng1Count < 1000)){
        turn = 0;
        vref = 0;
    }
}

//pixy getblocks function
void getBlocks(uint16_t sgn)
{
   GpioDataRegs.GPDCLEAR.bit.GPIO125 = 1; //Slave Select low
    SpicRegs.SPITXBUF = 0xaec1;
    SpicRegs.SPITXBUF = 0x2002;
    SpicRegs.SPITXBUF = ((1<<(sgn+7))+ 0x0001);
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;

    while (SpicRegs.SPIFFRX.bit.RXFFST != 16);
    temp = SpicRegs.SPIRXBUF;
    temp = SpicRegs.SPIRXBUF;
    temp = SpicRegs.SPIRXBUF;
    temp = SpicRegs.SPIRXBUF;
    temp = SpicRegs.SPIRXBUF;
    temp = SpicRegs.SPIRXBUF;
    gb1 = SpicRegs.SPIRXBUF;
    gb2 = SpicRegs.SPIRXBUF;
    gb3 = SpicRegs.SPIRXBUF;
    gb4 = SpicRegs.SPIRXBUF;
    gb5 = SpicRegs.SPIRXBUF;
    gb6 = SpicRegs.SPIRXBUF;
    gb7 = SpicRegs.SPIRXBUF;
    gb8 = SpicRegs.SPIRXBUF;
    gb9 = SpicRegs.SPIRXBUF;
    gb10 = SpicRegs.SPIRXBUF;

    DELAY_US(10); // Delay 10us to allow time to get ready for next transfer.

    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    SpicRegs.SPITXBUF = 0x00;
    while (SpicRegs.SPIFFRX.bit.RXFFST != 3);
    gb11 = SpicRegs.SPIRXBUF;
    gb12 = SpicRegs.SPIRXBUF;
    gb13 = SpicRegs.SPIRXBUF;

    if ((gb2 & 0x00FF) == (0x0021)) {
            gbFlag = 1;
            blockX[sgn-1] = (gb6 & 0xFF00) | (gb5 & 0x00FF);
            blockY[sgn-1] = (gb7 & 0xFF00) | (gb6 & 0x00FF);
            blockWidth[sgn-1] = (gb8 & 0xFF00) | (gb7 & 0x00FF);
            blockHeight[sgn-1] = (gb9 & 0xFF00) | (gb8 & 0x00FF);

    }
    GpioDataRegs.GPDSET.bit.GPIO125 = 1; // Slave Select High

}
