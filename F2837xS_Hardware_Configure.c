#include "F28x_Project.h"
#include "F28377S_MyDefinition.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////     HARDWARE CONFIG    ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//  scia_echoback_init - Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F,
//                       default, 1 STOP bit, no parity
//
/*******************************************************************************
 * Função de inicialização do SCIA (que é o dispositivo de comunicação da porta
 * Serial. É feita a configuração dos registradores  necessários.
 */
void scia_init(void){  // Note: Clocks were turned on to the SCIA peripheral  in the InitSysCtrl() function

SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback// No parity,8 char bits, async mode, idle-line protocol
SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
SciaRegs.SCICTL2.all = 0x0003;
SciaRegs.SCICTL2.bit.TXINTENA = 0;
SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
//SciaRegs.SCIHBAUD.all = 0x0002;  //38400
//SciaRegs.SCILBAUD.all = 0x008A;
//SciaRegs.SCIHBAUD.all = 0x0000; //115200
//SciaRegs.SCILBAUD.all = 0x00D8;
//SciaRegs.SCIHBAUD.all = 0x0000; //230400
//SciaRegs.SCILBAUD.all = 0x006C;
SciaRegs.SCIHBAUD.all = 0x0000; //460800
SciaRegs.SCILBAUD.all = 0x0036;
//SciaRegs.SCIHBAUD.all = 0x0000;   //921600
//SciaRegs.SCILBAUD.all = 0x001B;
//SciaRegs.SCIHBAUD.all = 0x0000; //1820000
//SciaRegs.SCILBAUD.all = 0x000D;
SciaRegs.SCIFFTX.all = 0x8000;
SciaRegs.SCIFFRX.all = 0x0000;
SciaRegs.SCIFFCT.all = 0x00;
SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;

}
/*******************************************************************************
 * Configura as GPIOs necessárias para o funcionamento do sistema
 */
void Gpioconfig(void){
EALLOW;
////////////////////////////////////////////// SCI PINS ////////////////////////
GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 1;
GpioCtrlRegs.GPCGMUX2.bit.GPIO84 = 1;
GpioCtrlRegs.GPCQSEL2.bit.GPIO84 = 3;
GpioCtrlRegs.GPCDIR.bit.GPIO84 = 1;
GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 1;
GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 1;
GpioCtrlRegs.GPCQSEL2.bit.GPIO85 = 3;
GpioCtrlRegs.GPCDIR.bit.GPIO85 = 0;
////////////////////////////////////////////// SETTINGS ////////////////////////
GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 0; //DB14 AD1
GpioCtrlRegs.GPCDIR.bit.GPIO71 = 1;  //DB14 AD1
GpioCtrlRegs.GPCMUX2.bit.GPIO90 = 0; //DB15 AD1
GpioCtrlRegs.GPCDIR.bit.GPIO90 = 1;  //DB15 AD1
GpioCtrlRegs.GPCMUX2.bit.GPIO89 = 0; //DB14 AD2
GpioCtrlRegs.GPCDIR.bit.GPIO89 = 1;  //DB14 AD2
GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0; //DB15 AD2
GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1;  //DB15 AD2
////////////////////////////////////////////// CONVERS 1 2  ////////////////////
GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
////////////////////////////////////////////// OSC 0 1 2    ////////////////////
GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 0;
GpioCtrlRegs.GPCDIR.bit.GPIO72 = 1;
GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 0;
GpioCtrlRegs.GPCDIR.bit.GPIO73 = 1;
GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 0;
GpioCtrlRegs.GPCDIR.bit.GPIO78 = 1;
////////////////////////////////////////////// PAR/SER/BYTE     ////////////////
GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;
GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;
GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;
GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;
////////////////////////////////////////////// TESTE ///////////////////////////
GpioCtrlRegs.GPCMUX2.bit.GPIO92 = 0;
GpioCtrlRegs.GPCDIR.bit.GPIO92 = 1;
////////////////////////////////////////////// RESET ///////////////////////////
GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0;
GpioCtrlRegs.GPBDIR.bit.GPIO62 = 1;
//////////////////////////////////////////////  RANGE //////////////////////////
GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;
GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
//////////////////////////////////////   LEDs   ////////////////////////////////
GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0;
GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;
GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0;
GpioCtrlRegs.GPBDIR.bit.GPIO43 = 1;
////////////////////////////////////// interrupts   ////////////////////////////
GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;         // GPIO
GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;          // input
GpioCtrlRegs.GPAQSEL1.bit.GPIO2 = 0;        // XINT1 Synch to SYSCLKOUT only
GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;         // GPIO
GpioCtrlRegs.GPADIR.bit.GPIO3 = 0;          // input
GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 0;        // XINT2 Synch to SYSCLKOUT only
GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0x00;   // Each sampling window
////////////////////////////////////////////////////////////////////////////////
EDIS;
}
/*******************************************************************************
 *
 */
void external_interrupt(void){// Configure XINT1 and XINT2

     GPIO_SetupXINT1Gpio(3);
     GPIO_SetupXINT2Gpio(2);

XintRegs.XINT1CR.bit.POLARITY = 0;          // Falling edge interrupt
XintRegs.XINT2CR.bit.POLARITY = 0;          // Rising edge interrupt
XintRegs.XINT1CR.bit.ENABLE = 1;            // Enable XINT1
XintRegs.XINT2CR.bit.ENABLE = 1;            // Enable XINT2
}
///////////////////////////////////////////////////////////////////////////////
void timer_config(Uint16 timerzero_t_us, Uint16 timerum_t_us, Uint16 timerdois_t_us){
EALLOW;
PieVectTable.TIMER0_INT = &cpu_timer0_isr;
PieVectTable.TIMER1_INT = &cpu_timer1_isr;
PieVectTable.TIMER2_INT = &cpu_timer2_isr;
EDIS;

    InitCpuTimers();

    ConfigCpuTimer(&CpuTimer0, 20000, timerzero_t_us);
    ConfigCpuTimer(&CpuTimer1, 2, timerum_t_us);
    ConfigCpuTimer(&CpuTimer2, 200, timerdois_t_us);

CpuTimer0Regs.TCR.all = 0x4000;       // Use write-only instruction to set TSS bit = 0
CpuTimer1Regs.TCR.all = 0x4000;       // Use write-only instruction to set TSS bit = 0
CpuTimer2Regs.TCR.all = 0x4000;       // Use write-only instruction to set TSS bit = 0
}
///////////////////////////////////////////////////////////////////////////////
void spi_fifo_init(void){
// Initialize SPI FIFO registers A
SpiaRegs.SPIFFTX.all = 0x8000;    // Enable FIFOs, set TX FIFO level to 0
SpiaRegs.SPIFFRX.all = 0x0000;    // Set RX FIFO level to 0
SpiaRegs.SPIFFCT.all = 0x00;
SpiaRegs.SPIFFTX.bit.TXFIFO=1;
SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
// Initialize SPI FIFO registers B
SpibRegs.SPIFFTX.all = 0x8000;    // Enable FIFOs, set TX FIFO level to 0
SpibRegs.SPIFFRX.all = 0x0000;    // Set RX FIFO level to 0
SpibRegs.SPIFFCT.all = 0x00;
SpibRegs.SPIFFTX.bit.TXFIFO=1;
SpibRegs.SPIFFRX.bit.RXFIFORESET=1;

    InitSpi();
}
//////////////////////////////////////////////////////////////////////////////////////
void mcbsp_a_init_dlb(){
// RESET MCBSP
McbspaRegs.SPCR2.bit.FRST = 0; // Frame Sync generator reset
McbspaRegs.SPCR2.bit.GRST = 0; // Sample Rate generator Reset
McbspaRegs.SPCR2.bit.XRST = 0; // Transmitter reset
McbspaRegs.SPCR1.bit.RRST = 0; // Receiver reset
// Initialize McBSP Registers // McBSP register settings for Digital loop back
McbspaRegs.SPCR2.all=0x0000;        // Reset FS generator, sample rate, generator & transmitter
McbspaRegs.SPCR1.all=0x0000;        // Reset Receiver, Right justify word
McbspaRegs.SPCR1.bit.DLB = 0;       // disable DLB mode. Comment out for non-DLB mode.
McbspaRegs.SPCR1.bit.CLKSTP = 10;   // Enable DLB mode. Comment out for non-DLB mode.
McbspaRegs.RCR2.bit.RDATDLY = 1;    // RX data delay is 1 bit
McbspaRegs.RCR1.all = 0x0;
McbspaRegs.XCR2.bit.XDATDLY = 1;    // TX data delay is 1 bit
McbspaRegs.XCR1.all = 0x0;
McbspaRegs.SRGR2.bit.GSYNC = 1;     // No clock sync for CLKG
McbspaRegs.SRGR2.bit.FPER = 80;    // Frame-synchronization period
McbspaRegs.SRGR2.bit.FSGM = 1;      // Frame-synchronization pulses from  the sample rate generator
McbspaRegs.SRGR2.bit.CLKSM = 1;     // Sample rate generator input clock
McbspaRegs.SRGR1.bit.CLKGDV = 30;    // Divide-down value for CLKG
    delay_loop();
McbspaRegs.SRGR1.bit.FWID = 1;      // Frame-synchronization pulse width                                // is LSPCLK
McbspaRegs.MCR2.all = 0x0;
McbspaRegs.MCR1.all = 0x0;
McbspaRegs.PCR.bit.SCLKME = 0;
McbspaRegs.PCR.bit.CLKXP = 0;
McbspaRegs.PCR.bit.CLKXM = 1;       // CLKX generated internally, CLKR derived from an external source
McbspaRegs.PCR.bit.FSXM = 1;        // FSX generated internally, FSR derived from an external source
McbspaRegs.PCR.bit.FSXP = 1;        // FSX generated internally, FSR derived from an external source
McbspaRegs.PCR.bit.CLKRP = 1;
McbspaRegs.PCR.bit.CLKXP = 1;
    InitMcbspa16bit();
// Enable Sample rate generator and wait at least 2 CLKG clock cycles
McbspaRegs.SPCR2.bit.GRST = 1;
    delay_loop();
// Release from reset RX, TX and frame sync generator
McbspaRegs.SPCR2.bit.XRST = 1;
McbspaRegs.SPCR1.bit.RRST = 1;
McbspaRegs.SPCR2.bit.FRST = 1;
}
//////////////////////////////////////////////////////////////////////////////////////
void mcbsp_b_init_dlb(){
// RESET MCBSP
McbspbRegs.SPCR2.bit.FRST = 0; // Frame Sync generator reset
McbspbRegs.SPCR2.bit.GRST = 0; // Sample Rate generator Reset
McbspbRegs.SPCR2.bit.XRST = 0; // Transmitter reset
McbspbRegs.SPCR1.bit.RRST = 0; // Receiver reset
// Initialize McBSP Registers McBSP register settings for Digital loop back
McbspbRegs.SPCR2.all=0x0000;        // Reset FS generator, sample rate, generator & transmitter
McbspbRegs.SPCR1.all=0x0000;        // Reset Receiver, Right justify word
McbspbRegs.SPCR1.bit.DLB = 0;       // disable DLB mode. Comment out for non-DLB mode.
McbspbRegs.SPCR1.bit.CLKSTP = 10;   // Enable DLB mode. Comment out for non-DLB mode.
McbspbRegs.RCR2.bit.RDATDLY = 1;    // RX data delay is 1 bit
McbspbRegs.RCR1.all = 0x0;
McbspbRegs.XCR2.bit.XDATDLY = 1;    // TX data delay is 1 bit
McbspbRegs.XCR1.all = 0x0;
McbspbRegs.SRGR2.bit.GSYNC = 1;     // No clock sync for CLKG
McbspbRegs.SRGR2.bit.FPER = 80;    // Frame-synchronization period
McbspbRegs.SRGR2.bit.FSGM = 1;      // Frame-synchronization pulses from  the sample rate generator
McbspbRegs.SRGR2.bit.CLKSM = 1;     // Sample rate generator input clock
McbspbRegs.SRGR1.bit.CLKGDV = 30;    // Divide-down value for CLKG
    delay_loop();
McbspbRegs.SRGR1.bit.FWID = 1;      // Frame-synchronization pulse width                                // is LSPCLK
McbspbRegs.MCR2.all = 0x0;
McbspbRegs.MCR1.all = 0x0;
McbspbRegs.PCR.bit.SCLKME = 0;
McbspbRegs.PCR.bit.CLKXP = 0;
McbspbRegs.PCR.bit.CLKXM = 1;       // CLKX generated internally, CLKR derived from an external source
McbspbRegs.PCR.bit.FSXM = 1;        // FSX generated internally, FSR derived from an external source
McbspbRegs.PCR.bit.FSXP = 1;        // FSX generated internally, FSR derived from an external source
McbspbRegs.PCR.bit.CLKRP = 1;
McbspbRegs.PCR.bit.CLKXP = 1;
    InitMcbspb16bit();
// Enable Sample rate generator and wait at least 2 CLKG clock cycles
McbspbRegs.SPCR2.bit.GRST = 1;
    delay_loop();
// Release from reset RX, TX and frame sync generator
McbspbRegs.SPCR2.bit.XRST = 1;
McbspbRegs.SPCR1.bit.RRST = 1;
McbspbRegs.SPCR2.bit.FRST = 1;
}
//////////////////////////////////////////////////////////////////////////////////////
