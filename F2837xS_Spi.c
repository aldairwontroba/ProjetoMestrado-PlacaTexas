//###########################################################################
//
// FILE:   F2837xS_Spi.c
//
// TITLE:  F2837xS SPI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xS Support Library v3.02.00.00 $
// $Release Date: Sat Sep 16 15:30:24 CDT 2017 $
// $Copyright:
// Copyright (C) 2014-2017 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "F2837xS_device.h"
#include "F2837xS_Examples.h"

//
// Calculate BRR: 7-bit baud rate register value
// SPI CLK freq = 500 kHz
// LSPCLK freq  = CPU freq / 4  (by default)
// BRR          = (LSPCLK freq / SPI CLK freq) - 1
//
#if CPU_FRQ_200MHZ
#define SPI_BRR        ((200E6 / 4) / 50E6) - 1
#endif

#if CPU_FRQ_150MHZ
#define SPI_BRR        ((150E6 / 4) / 10E6) - 1
#endif

#if CPU_FRQ_120MHZ
#define SPI_BRR        ((120E6 / 4) / 10E6) - 1
#endif

//
// InitSPI - This function initializes the SPI to a known state
//
void InitSpi(void)
{
    // Initialize SPI-A

    // Set reset low before configuration changes
    // Clock polarity (0 == rising, 1 == falling)
    // 16-bit character
    // Enable loop-back
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;
    SpiaRegs.SPICCR.bit.SPICHAR = (16-1);
    SpiaRegs.SPICCR.bit.SPILBK = 0;
    SpiaRegs.SPICCR.bit.HS_MODE = 1;
    // Enable master (0 == slave, 1 == master)
    // Enable transmission (Talk)
    // Clock phase (0 == normal, 1 == delayed)
    // SPI interrupts are disabled
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 0;
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;
    // Set the baud rate
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 0x9;
    // Set FREE bit
    // Halting on a breakpoint will not halt the SPI
    SpiaRegs.SPIPRI.bit.FREE = 1;
    // Release the SPI from reset
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;


    // Initialize SPI-B

     // Set reset low before configuration changes
     // Clock polarity (0 == rising, 1 == falling)
     // 16-bit character
     // Enable loop-back
     SpibRegs.SPICCR.bit.SPISWRESET = 0;
     SpibRegs.SPICCR.bit.CLKPOLARITY = 1;
     SpibRegs.SPICCR.bit.SPICHAR = (16-1);
     SpibRegs.SPICCR.bit.SPILBK = 0;
     SpibRegs.SPICCR.bit.HS_MODE = 1;
     // Enable master (0 == slave, 1 == master)
     // Enable transmission (Talk)
     // Clock phase (0 == normal, 1 == delayed)
     // SPI interrupts are disabled
     SpibRegs.SPICTL.bit.MASTER_SLAVE = 0;
     SpibRegs.SPICTL.bit.TALK = 1;
     SpibRegs.SPICTL.bit.CLK_PHASE = 0;
     SpibRegs.SPICTL.bit.SPIINTENA = 0;
     // Set the baud rate
     SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 0x9;
     // Set FREE bit
     // Halting on a breakpoint will not halt the SPI
     SpibRegs.SPIPRI.bit.FREE = 1;
     // Release the SPI from reset
     SpibRegs.SPICCR.bit.SPISWRESET = 1;
}

//
// InitSpiGpio - This function initializes GPIO pins to function as SPI pins.
//               Each GPIO pin can be configured as a GPIO pin or up to 3
//               different peripheral functional pins. By default all pins come
//               up as GPIO inputs after reset.
//
//               Caution:
//               For each SPI peripheral
//               Only one GPIO pin should be enabled for SPISOMO operation.
//               Only one GPIO pin should be enabled for SPISOMI operation.
//               Only one GPIO pin should be enabled for SPICLK  operation.
//               Only one GPIO pin should be enabled for SPISTE  operation.
//               Comment out other unwanted lines.
//
void InitSpiGpio()
{
   InitSpiaGpio();
}

//
// InitSpiaGpio - Initialize SPIA GPIOs
//
void InitSpiaGpio()
{
   EALLOW;

    //
    // Enable internal pull-up for the selected pins
    //
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
//    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;  // Enable pull-up on GPIO16 (SPISIMOA)
////  GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;   // Enable pull-up on GPIO5 (SPISIMOA)
//    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;  // Enable pull-up on GPIO17 (SPISOMIA)
////  GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;   // Enable pull-up on GPIO3 (SPISOMIA)
//    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;  // Enable pull-up on GPIO18 (SPICLKA)
//    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;  // Enable pull-up on GPIO19 (SPISTEA)

//    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;  // Enable pull-up on GPIO16 (SPISIMOA)
////  GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;   // Enable pull-up on GPIO5 (SPISIMOA)
//    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;  // Enable pull-up on GPIO17 (SPISOMIA)
////  GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;   // Enable pull-up on GPIO3 (SPISOMIA)
//    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;  // Enable pull-up on GPIO18 (SPICLKA)
//    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;  // Enable pull-up on GPIO19 (SPISTEA)


    // Set qualification for selected pins to asynch only
    //
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.
    //
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input GPIO16 (SPISIMOA)
////  GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 3;  // Asynch input GPIO5 (SPISIMOA)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
////  GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 3;  // Asynch input GPIO3 (SPISOMIA)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch input GPIO18 (SPICLKA)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // Asynch input GPIO19 (SPISTEA)

   // corre�ao de erro na placa
   GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO59 = 0;


// SPI-A
    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3; // Asynch input GPIO16 (SPISIMOA)
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3; // Asynch input GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3; // Asynch input GPIO18 (SPICLKA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3; // Asynch input GPIO19 (SPISTEA)

    //
    //Configure SPI-A pins using GPIO regs
    //
    // This specifies which of the possible GPIO pins will be SPI functional
    // pins.
    // Comment out other unwanted lines.
    //
//    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
////  GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 2;  // Configure GPIO5 as SPISIMOA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
////  GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 2;  // Configure GPIO3 as SPISOMIA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // Configure GPIO19 as SPISTEA

// SPI-A
    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3; // Configure GPIO58 as SPISIMOA
//    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3; // Configure GPIO59 as SPISOMIA
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3; // Configure GPIO60 as SPICLKA
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 3; // Configure GPIO61 as SPISTEA

    GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 3; // Configure GPIO58 as SPISIMOA
//    GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 3; // Configure GPIO59 as SPISOMIA
    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 3; // Configure GPIO60 as SPICLKA
    GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 3; // Configure GPIO61 as SPISTEA


 // SPI-B
     GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Asynch input GPIO63 (SPISIMOB)
     GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Asynch input GPIO64 (SPISOMIB)
     GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Asynch input GPIO65 (SPICLKB)
     GpioCtrlRegs.GPCQSEL1.bit.GPIO66 = 3; // Asynch input GPIO66 (SPISTEB)

 // SPI-B
     GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3; // Configure GPIO63 as SPISIMOB
     GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3; // Configure GPIO64 as SPISOMIB
     GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3; // Configure GPIO65 as SPICLKB
     GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3; // Configure GPIO66 as SPISTEB

     GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 3; // Configure GPIO63 as SPISIMOB
     GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 3; // Configure GPIO64 as SPISOMIB
     GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 3; // Configure GPIO65 as SPICLKB
     GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 3; // Configure GPIO66 as SPISTEB

     EDIS;
}
//
// End of file
//
