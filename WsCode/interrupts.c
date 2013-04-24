/*******************************************************************************
*
* TITLE:    interrupts.c
*
* VERSION:  0.4 (Beta)
*
* DATE:   03-Jan-2008
*
* AUTHOR:   R. Kevin Watson
*           kevinw@jpl.nasa.gov
*
* COMMENTS: This file contains template interrupt initialization & handling
*           code for the IFI FRC robot controller.
*
*               This file best viewed with tabs set to four.
*
*           You are free to use this source code for any non-commercial
*           use. Please do not make copies of this source code, modified
*           or un-modified, publicly available on the internet or elsewhere
*           without permission. Thanks.
*
*           Copyright ©2004-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
* Change log:
*
* DATE         REV  DESCRIPTION
* -----------  ---  ----------------------------------------------------------
* 22-Dec-2003  0.1  RKW Original
* 25-Feb-2004  0.2  RKW - Added the ability to clear the interrupt flag before
*                   enabling the interrupt.
* 24-Dec-2007  0.3  RKW - Modified code to be compatible with version 3.0+
*                   of the Microchip C18 compiler.
* 03-Jan-2008  0.4  RKW - Renamed all ISRs for consistancy across all
*                   modules of the new robot controller code.
*
*******************************************************************************/

#include <p18cxxx.h>
#include "interrupts.h"

/*******************************************************************************
*
* FUNCTION:   Initialize_Int_1()
*
* PURPOSE:    Initializes interrupt 1
*
* CALLED FROM:  teleop.c/Initialization()
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_1_ISR
void Initialize_Int_1(void)
{
  // initialize external interrupt 1 (INT2 on user 18F8520/18F8722)

  TRISBbits.TRISB2 = 1;     // make sure the RB2/INT2 pin is configured as an input
                            //
  INTCON3bits.INT2IP = 0;   // 0: interrupt 1 is low priority (leave at 0 for IFI controllers)
                            // 1: interrupt 1 is high priority
                            //
  INTCON2bits.INTEDG2 = 0;  // 0: trigger on the falling-edge
                            // 1: trigger on the rising-edge
                            //
  INTCON3bits.INT2IF = 0;   // 0: external interrupt 1 hasn't happened (set to 0 before enabling the interrupt)
                            // 1: external interrupt 1 has happened
                            //
  INTCON3bits.INT2IE = 0;   // 0: disable interrupt 1
                            // 1: enable interrupt 1

}
#endif

/*******************************************************************************
*
* FUNCTION:   Initialize_Int_2()
*
* PURPOSE:    Initializes interrupt 2
*
* CALLED FROM:  teleop.c/Initialization()
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_2_ISR
void Initialize_Int_2(void)
{
  // initialize external interrupt 2 (INT3 on user 18F8520/18F8722)
  TRISBbits.TRISB3 = 1;     // make sure the RB3/CCP2/INT3 pin is configured as an input
                            //
  INTCON2bits.INT3IP = 0;   // 0: interrupt 2 is low priority (leave at 0 for IFI controllers)
                            // 1: interrupt 2 is high priority
                            //
  INTCON2bits.INTEDG3 = 0;  // 0: trigger on the falling-edge
                            // 1: trigger on the rising-edge
                            //
  INTCON3bits.INT3IF = 0;   // 0: external interrupt 2 hasn't happened (set to 0 before enabling the interrupt)
                            // 1: external interrupt 2 has happened
                            //
  INTCON3bits.INT3IE = 0;   // 0: disable interrupt 2
                            // 1: enable interrupt 2
}
#endif

/*******************************************************************************
*
* FUNCTION:   Initialize_Int_3_6()
*
* PURPOSE:    Initializes interrupts 3 through 6
*
* CALLED FROM:  teleop.c/Initialization()
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_3_6_ISR
void Initialize_Int_3_6(void)
{
  // initialize external interrupts 3-6 (KBI0 - KBI3 on user 18F8520/18F8722)
  TRISBbits.TRISB4 = 1;   // make sure the RB4/HBI0 pin is configured as an input
  TRISBbits.TRISB5 = 1;   // make sure the RB5/KBI1/PGM pin is configured as an input
  TRISBbits.TRISB6 = 1;   // make sure the RB6/KBI2/PGC pin is configured as an input
  TRISBbits.TRISB7 = 1;   // make sure the RB7/KBI3/PGD pin is configured as an input
                          //
  INTCON2bits.RBIP = 0;   // 0: interrupts 3-6 are low priority (leave at 0 for IFI controllers)
                          // 1: interrupts 3-6 are high priority
                          //
  INTCONbits.RBIF = 0;    // 0: none of the interrupt 3-6 pins has changed state (set to 0 before enabling the interrupts)
                          // 1: at least one of the interrupt 3-6 pins has changed state
                          //
  INTCONbits.RBIE = 0;    // 0: disable interrupts 3-6
                          // 1: enable interrupts 3-6
}
#endif

/*******************************************************************************
*
* FUNCTION:   Int_1_ISR()
*
* PURPOSE:    If enabled, the interrupt 1 handler is called when the
*         interrupt 1/digital input 1 pin changes logic level. The
*         edge that the interrupt 1 pin reacts to is programmable
*         (see comments in the Initialize_Interrupts() function,
*         above).
*
* CALLED FROM:  ifi_frc.c/Interrupt_Handler_Low()
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_1_ISR
#pragma tmpdata low_isr_tmpdata
void Int_1_ISR(void)
{
  // this function will be called when an interrupt 1 occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
* FUNCTION:   Int_2_ISR()
*
* PURPOSE:    If enabled, the interrupt 2 handler is called when the
*         interrupt 2/digital input 2 pin changes logic level. The
*         edge that the interrupt 2 pin reacts to is programmable
*         (see comments in the Initialize_Interrupts() function,
*         above).
*
* CALLED FROM:  ifi_frc.c/Interrupt_Handler_Low()
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_2_ISR
#pragma tmpdata low_isr_tmpdata
void Int_2_ISR(void)
{
  // this function will be called when an interrupt 2 occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
* FUNCTION:   Int_3_ISR()
*
* PURPOSE:    If enabled, the interrupt 3 handler is called when the
*         interrupt 3/digital input 3 pin changes logic level.
*
* CALLED FROM:  ifi_frc.c/Interrupt_Handler_Low()
*
* PARAMETERS:   RB4_State is the current logic level of the
*         interrupt 3 pin.
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_3_6_ISR
#pragma tmpdata low_isr_tmpdata
void Int_3_ISR(unsigned char RB4_State)
{
  // this function will be called when an interrupt 3 occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
* FUNCTION:   Int_4_ISR()
*
* PURPOSE:    If enabled, the interrupt 4 handler is called when the
*         interrupt 4/digital input 4 pin changes logic level.
*
* CALLED FROM:  ifi_frc.c/Interrupt_Handler_Low()
*
* PARAMETERS:   RB5_State is the current logic level of the
*         interrupt 4 pin.
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_3_6_ISR
#pragma tmpdata low_isr_tmpdata
void Int_4_ISR(unsigned char RB5_State)
{
  // this function will be called when an interrupt 4 occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
* FUNCTION:   Int_5_ISR()
*
* PURPOSE:    If enabled, the interrupt 5 handler is called when the
*         interrupt 5/digital input 5 pin changes logic level.
*
* CALLED FROM:  ifi_frc.c/Interrupt_Handler_Low()
*
* PARAMETERS:   RB6_State is the current logic level of the
*         interrupt 5 pin.
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_3_6_ISR
#pragma tmpdata low_isr_tmpdata
void Int_5_ISR(unsigned char RB6_State)
{
  // this function will be called when an interrupt 5 occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
* FUNCTION:   Int_6_ISR()
*
* PURPOSE:    If enabled, the interrupt 6 handler is called when the
*         interrupt 6/digital input 6 pin changes logic level.
*
* CALLED FROM:  ifi_frc.c/Interrupt_Handler_Low()
*
* PARAMETERS:   RB7_State is the current logic level of the
*         interrupt 6 pin.
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_3_6_ISR
#pragma tmpdata low_isr_tmpdata
void Int_6_ISR(unsigned char RB7_State)
{
  // this function will be called when an interrupt 6 occurs
}
#pragma tmpdata
#endif
