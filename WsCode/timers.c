/*******************************************************************************
*
* TITLE:    timers.c
*
* VERSION:  0.2 (Beta)
*
* DATE:   03-Jan-2008
*
* AUTHOR:   R. Kevin Watson
*           kevinw@jpl.nasa.gov
*
* COMMENTS: This file contains template timer initialization & interrupt
*           handling code for the IFI FRC robot controller.
*
*               This file best viewed with tabs set to four.
*
*           You are free to use this source code for any non-commercial
*           use. Please do not make copies of this source code, modified
*           or un-modified, publicly available on the internet or elsewhere
*           without permission. Thanks.
*
*           Copyright ©2007-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
* Change log:
*
* DATE         REV  DESCRIPTION
* -----------  ---  ----------------------------------------------------------
* 24-Dec-2007  0.1  RKW Original
* 03-Jan-2008  0.4  RKW - Renamed all ISRs for consistancy across all
*                   modules of the new robot controller code.
*
*******************************************************************************/

#include <p18cxxx.h>
#include "timers.h"

/*******************************************************************************
*
* FUNCTION:   Initialize_Timer_0()
*
* PURPOSE:    Initializes the timer 0 hardware.
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
#ifdef ENABLE_TIMER_0_ISR
void Initialize_Timer_0(void)
{
  TMR0L = 0x00;     // least significant 8-bits of the timer 0 register (this is readable and writable)
  TMR0H = 0x00;     // most significant 8-bits of the timer 0 register (this is readable and writable)
              //
  T0CONbits.T0PS0 = 0;    // T0PS2 TOPS1 T0PS0
  T0CONbits.T0PS1 = 0;    //   0     0     0    1:2 prescaler (clock=5MHz/each tick=200ns)
  T0CONbits.T0PS2 = 0;    //   0     0     1    1:4 prescaler (clock=2.5MHz/each tick=400ns)
                          //   0     1     0    1:8 prescaler (clock=1.25MHz/each tick=800ns)
                          //   0     1     1    1:16 prescaler (clock=625KHz/each tick=1.6us)
                          //   1     0     0    1:32 prescaler (clock=312.5KHz/each tick=3.2us)
                          //   1     0     1    1:64 prescaler (clock=156.25KHz/each tick=6.4us)
                          //   1     1     0    1:128 prescaler (clock=78.125KHz/each tick=12.8us)
                          //   1     1     1    1:256 prescaler (clock=39.0625 KHz/each tick=25.6us)
                          //
  T0CONbits.PSA = 1;      // 0: use the prescaler to derive the timer clock
                          // 1: don't use the prescaler (clock=10MHz/each tick=100ns)
                          //
  T0CONbits.T0SE = 0;     // 0: when using an external clock, timer increments on the rising-edge
                          // 1: when using an external clock, timer increments on the falling-edge
                          //
  T0CONbits.T0CS = 0;     // 0: use the internal clock (leave at 0)
                          // 1: use an external clock on RA4/T0CKI (don't use - not available on IFI controllers)
                          //
  T0CONbits.T08BIT = 0;   // 0: timer 0 is configured as a 16-bit timer/counter
                          // 1: timer 0 is configured as an 8-bit timer/counter
                          //
  INTCON2bits.TMR0IP = 0; // 0: timer 0 overflow interrupt is low priority (leave at 0 for IFI controllers)
                          // 1: timer 0 overflow interrupt is high priority
                          //
  INTCONbits.TMR0IF = 0;  // 0: timer 0 overflow hasn't happened (set to 0 before enabling the interrupt)
                          // 1: timer 0 overflow has happened
                          //
  INTCONbits.TMR0IE = 0;  // 0: disable timer 0 interrupt on overflow (i.e., a transition from FFFF->0 or FF->0)
                          // 1: enable timer 0 interrupt on overflow (i.e., a transition from FFFF->0 or FF->0)
                          //
  T0CONbits.TMR0ON = 0;   // 0: timer 0 is disabled
                          // 1: timer 0 is enabled (running)
}
#endif

/*******************************************************************************
*
* FUNCTION:   Timer_0_ISR()
*
* PURPOSE:    If enabled, the timer 0 interrupt handler is called when
*         the TMR0 register overflows and rolls over to zero.
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
#ifdef ENABLE_TIMER_0_ISR
#pragma tmpdata low_isr_tmpdata
void Timer_0_ISR(void)
{
  // this function will be called when a timer 0 interrupt occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
* FUNCTION:   Initialize_Timer_1()
*
* PURPOSE:    Initializes the timer 1 hardware.
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
#ifdef ENABLE_TIMER_1_ISR
void Initialize_Timer_1(void)
{
  TMR1L = 0x00;     // least significant 8-bits of the timer 1 register (this is readable and writable)
  TMR1H = 0x00;     // most significant 8-bits of the timer 1 register (this is readable and writable)
                          //
  T1CONbits.T1CKPS0 = 0;  // T1CSP1 T1CSP0
  T1CONbits.T1CKPS1 = 0;  //   0      0   1:1 prescaler (clock=10MHz/each tick=100ns)
                          //   0      1   1:2 prescaler (clock=5MHz/each tick=200ns)
                          //   1      0   1:4 prescaler (clock=2.5MHz/each tick=400ns)
                          //   1      1   1:8 prescaler (clock=1.25MHz/each tick=800ns)
                          //
  T1CONbits.T1OSCEN = 0;  // 0: timer 1 oscillator disabled (leave at 0 to allow the use of an external clock)
                          // 1: timer 1 oscillator enabled (can't be used because of hardware constraints)
                          //
  T1CONbits.TMR1CS = 0;   // 0: use the internal clock
                          // 1: use an external clock on RC0/T1OSO/T13CLK (rc_dig_in14 on robot controller)
                          //
  T1CONbits.RD16 = 1;     // 0: timer 1 register operations are done in two 8-bit accesses
                          // 1: timer 1 register operations are done in one 16-bit access
                          //    In this mode, reading TMR1L will latch a copy of TMR1H into a buffer
                          //    mapped to the TMR1H memory address. Conversely, a write to the buffer
                          //    followed by a write to the TMR1L register will update the entire 16-bit
                          //    timer at once. This solves the problem where the timer may overflow
                          //    between two 8-bit accesses. Here's an example of how to do a 16-bit read:
                          //
                          //    unsigned char Temp_Buf; // 8-bit temporary buffer
                          //    unsigned int Timer_Snapshot; // 16-bit variable
                          //
                          //    Temp_Buf = TMR1L; // TMR1L must be read before TMR1H
                          //    Timer_Snapshot = TMR1H;
                          //    Timer_Snapshot <<= 8; // move TMR1H data to the upper half of the variable
                          //    Timer_Snapshot += Temp_Buf; // we now have all sixteen bits
                          //
  IPR1bits.TMR1IP = 0;    // 0: timer 1 overflow interrupt is low priority (leave at 0 on IFI controllers)
                          // 1: timer 1 overflow interrupt is high priority
                          //
  PIR1bits.TMR1IF = 0;    // 0: timer 1 overflow hasn't happened (set to 0 before enabling the interrupt)
                          // 1: timer 1 overflow has happened
                          //
  PIE1bits.TMR1IE = 0;    // 0: disable timer 1 interrupt on overflow (i.e., a transition from FFFF->0)
                          // 1: enable timer 1 interrupt on overflow (i.e., a transition from FFFF->0)
                          //
  T1CONbits.TMR1ON = 0;   // 0: timer 1 is disabled
                          // 1: timer 1 is enabled (running)
}
#endif

/*******************************************************************************
*
* FUNCTION:   Timer_1_ISR()
*
* PURPOSE:    If enabled, the timer 1 interrupt handler is called when
*         the TMR1 register overflows and rolls over to zero.
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
#ifdef ENABLE_TIMER_1_ISR
#pragma tmpdata low_isr_tmpdata
void Timer_1_ISR(void)
{
  // this function will be called when a timer 1 interrupt occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
* FUNCTION:   Initialize_Timer_2()
*
* PURPOSE:    Initializes the timer 2 hardware.
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
#ifdef ENABLE_TIMER_2_ISR
void Initialize_Timer_2(void)
{
  TMR2 = 0x00;      // 8-bit timer 2 register (this is readable and writable)
                    //
  PR2 = 0xFF;       // timer 2 period register - timer 2 increments to this
                    // value then resets to zero on the next clock and starts
                    // all over again
                    //
  T2CONbits.T2OUTPS0 = 0; // T2OUTPS3 T2OUTPS2 T2OUTPS1 T2OUTPS0
  T2CONbits.T2OUTPS1 = 0; //    0        0        0        0    1:1 postscaler
  T2CONbits.T2OUTPS2 = 0; //    0        0        0        1    1:2 postscaler
  T2CONbits.T2OUTPS3 = 0; //    0        0        1        0    1:3 postscaler
                          //    0        0        1        1    1:4 postscaler
                          //    0        1        0        0    1:5 postscaler
                          //    0        1        0        1    1:6 postscaler
                          //    0        1        1        0    1:7 postscaler
                          //    0        1        1        1    1:8 postscaler
                          //    1        0        0        0    1:9 postscaler
                          //    1        0        0        1    1:10 postscaler
                          //    1        0        1        0    1:11 postscaler
                          //    1        0        1        1    1:12 postscaler
                          //    1        1        0        0    1:13 postscaler
                          //    1        1        0        1    1:14 postscaler
                          //    1        1        1        0    1:15 postscaler
                          //    1        1        1        1    1:16 postscaler
                          //
  T2CONbits.T2CKPS0 = 0;  // T2CKPS1  T2CKPS0
  T2CONbits.T2CKPS1 = 0;  //    0        0  1:1 prescaler (clock = 10MHz/each tick=100ns)
                          //    0        1  1:4 prescaler (clock = 2.5MHz/each tick=400ns)
                          //    1        x  1:16 prescaler (clock = 625KHz/each tick=1.6us) (T2CKPS0 doesn't matter)
                          //
  IPR1bits.TMR2IP = 0;    // 0: timer 2 interrupt is low priority (leave at 0 for IFI controllers)
                          // 1: timer 2 interrupt is high priority
                          //
  PIR1bits.TMR2IF = 0;    // 0: TMR2 to PR2 match hasn't happened (set to 0 before enabling the interrupt)
                          // 1: TMR2 to PR2 match has happened
                          //
  PIE1bits.TMR2IE = 0;    // 0: disable timer 2 interrupt on PR2 match
                          // 1: enable timer 2 interrupt on PR2 match
                          //    if the prescaler is enabled (i.e., greater than 1:1), this
                          //    match will occur n times (where n is the postscaler value)
                          //    before an interrupt will be generated
                          //
  T2CONbits.TMR2ON = 0;   // 0: timer 2 is disabled
                          // 1: timer 2 is enabled (running)
}
#endif

/*******************************************************************************
*
* FUNCTION:   Timer_2_ISR()
*
* PURPOSE:    If enabled, the timer 2 interrupt handler is called when the
*         TMR2 register matches the value stored in the PR2 register.
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
#ifdef ENABLE_TIMER_2_ISR
#pragma tmpdata low_isr_tmpdata
void Timer_2_ISR(void)
{
  // this function will be called when a timer 2 interrupt occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
* FUNCTION:   Initialize_Timer_3()
*
* PURPOSE:    Initializes the timer 3 hardware.
*
* CALLED FROM:  teleop.c/Initialization()
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   Timer 3 is used by the precision PWM software
*         (pwm.c/.h). If you use timer 3 in your application,
*         you'll need to remove pwm.c/.h from your project or
*         modify it to use timer 1.
*
*******************************************************************************/
#ifdef ENABLE_TIMER_3_ISR
void Initialize_Timer_3(void)
{
  TMR3L = 0x00;     // least significant 8-bits of the timer 3 register (this is readable and writable)
  TMR3H = 0x00;     // most significant 8-bits of the timer 3 register (this is readable and writable)
              //
  T3CONbits.T3CKPS0 = 0;  // T3CKPS1 T3CKPS0
  T3CONbits.T3CKPS1 = 0;  //    0       0   1:1 prescaler (clock=10MHz/each tick=100ns)
              //    0       1   1:2 prescaler (clock=5MHz/each tick=200ns)
              //    1       0   1:4 prescaler (clock=2.5MHz/each tick=400ns)
              //    1       1   1:8 prescaler (clock=1.25MHz/each tick=800ns)
              //
  T3CONbits.TMR3CS = 0; // 0: use the internal clock
              // 1: use an external clock on RC0/T1OSO/T13CLK (rc_dig_in14 on robot controller)
              //
  T3CONbits.T3SYNC = 1; // 0: do not synchronize the external clock (this can cause timing problems)
              // 1: synchronize the external clock to the 18F8520/18F8722 internal clock, which is desirable
              //
  T3CONbits.RD16 = 1;   // 0: timer 3 register operations are done in two 8-bit accesses
              // 1: timer 3 register operations are done in one 16-bit access
              //    In this mode, reading TMR3L will latch a copy of TMR3H into a buffer
              //    mapped to the TMR3H memory address. Conversely, a write to the buffer
              //    followed by a write to the TMR3L register will update the entire 16-bit
              //    timer at once. This solves the problem where the timer may overflow
              //    between two 8-bit accesses. Here's an example of how to do a 16-bit read:
              //
              //    unsigned char Temp_Buf; // 8-bit temporary buffer
              //    unsigned int Timer_Snapshot; // 16-bit variable
              //
              //    Temp_Buf = TMR3L; // TMR3L must be read before TMR3H
              //    Timer_Snapshot = TMR3H;
              //    Timer_Snapshot <<= 8; // move TMR3H data to the upper half of the variable
              //    Timer_Snapshot += Temp_Buf; // we now have all sixteen bits
              //
  IPR2bits.TMR3IP = 0;  // 0: timer 3 overflow interrupt is low priority (leave at 0 on IFI controllers)
                        // 1: timer 3 overflow interrupt is high priority
                        //
  PIR2bits.TMR3IF = 0;  // 0: timer 3 overflow hasn't happened (set to 0 before enabling the interrupt)
                        // 1: timer 3 overflow has happened
                        //
  PIE2bits.TMR3IE = 0;  // 0: disable timer 3 interrupt on overflow (i.e., a transition from FFFF->0)
                        // 1: enable timer 3 interrupt on overflow (i.e., a transition from FFFF->0)
                        //
  T3CONbits.TMR3ON = 0; // 0: timer 3 is disabled
                        // 1: timer 3 is enabled (running)
}
#endif

/*******************************************************************************
*
* FUNCTION:   Timer_3_ISR()
*
* PURPOSE:    If enabled, the timer 3 interrupt handler is called when
*         the TMR3 register overflows and rolls over to zero.
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
#ifdef ENABLE_TIMER_3_ISR
#pragma tmpdata low_isr_tmpdata
void Timer_3_ISR(void)
{
  // this function will be called when a timer 3 interrupt occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
* FUNCTION:   Initialize_Timer_4()
*
* PURPOSE:    Initializes the timer 4 hardware.
*
* CALLED FROM:  teleop.c/Initialization()
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   Timer 4 is used by the ADC software (adc.c/.h). If you're
*         using the ADC software, try using timer 2 instead.
*
*******************************************************************************/
#ifdef ENABLE_TIMER_4_ISR
void Initialize_Timer_4(void)
{
  TMR4 = 0x00;      // 8-bit timer 4 register (this is readable and writable)
                    //
  PR4 = 0xFF;       // timer 4 period register - timer 4 increments to this
                    // value then resets to zero on the next clock and starts
                    // all over again
                    //
  T4CONbits.T4OUTPS0 = 0; // T4OUTPS3 T4OUTPS2 T4OUTPS1 T4OUTPS0
  T4CONbits.T4OUTPS1 = 0; //    0        0        0        0    1:1 postscaler
  T4CONbits.T4OUTPS2 = 0; //    0        0        0        1    1:2 postscaler
  T4CONbits.T4OUTPS3 = 0; //    0        0        1        0    1:3 postscaler
                          //    0        0        1        1    1:4 postscaler
                          //    0        1        0        0    1:5 postscaler
                          //    0        1        0        1    1:6 postscaler
                          //    0        1        1        0    1:7 postscaler
                          //    0        1        1        1    1:8 postscaler
                          //    1        0        0        0    1:9 postscaler
                          //    1        0        0        1    1:10 postscaler
                          //    1        0        1        0    1:11 postscaler
                          //    1        0        1        1    1:12 postscaler
                          //    1        1        0        0    1:13 postscaler
                          //    1        1        0        1    1:14 postscaler
                          //    1        1        1        0    1:15 postscaler
                          //    1        1        1        1    1:16 postscaler
                          //
  T4CONbits.T4CKPS0 = 0;  // T4CKPS1  T4CKPS0
  T4CONbits.T4CKPS1 = 0;  //    0        0  1:1 prescaler (clock = 10MHz/each tick=100ns)
                          //    0        1  1:4 prescaler (clock = 2.5MHz/each tick=400ns)
                          //    1        x  1:16 prescaler (clock = 625KHz/each tick=1.6us) (T2CKPS0 doesn't matter)
                          //
  IPR3bits.TMR4IP = 0;    // 0: timer 4 interrupt is low priority (leave at 0 for IFI controllers)
                          // 1: timer 4 interrupt is high priority
                          //
  PIR3bits.TMR4IF = 0;    // 0: TMR4 to PR4 match hasn't happened (set to 0 before enabling the interrupt)
                          // 1: TMR4 to PR4 match has happened
                          //
  PIE3bits.TMR4IE = 0;    // 0: disable timer 4 interrupt on PR4 match
                          // 1: enable timer 4 interrupt on PR4 match
                          //    if the prescaler is enabled (i.e., greater than 1:1), this
                          //    match will occur n times (where n is the postscaler value)
                          //    before an interrupt will be generated
                          //
  T4CONbits.TMR4ON = 0;   // 0: timer 4 is disabled
                          // 1: timer 4 is enabled (running)
}
#endif

/*******************************************************************************
*
* FUNCTION:   Timer_4_ISR()
*
* PURPOSE:    If enabled, the timer 4 interrupt handler is called when the
*         TMR4 register matches the value stored in the PR4 register.
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
#ifdef ENABLE_TIMER_4_ISR
#pragma tmpdata low_isr_tmpdata
void Timer_4_ISR(void)
{
  // this function will be called when a timer 4 interrupt occurs
}
#pragma tmpdata
#endif
