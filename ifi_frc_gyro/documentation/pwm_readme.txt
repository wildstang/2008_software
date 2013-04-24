***************************************************************

You are free to use this source code for any non-commercial
use. Please do not make copies of this source code, modified
or unmodified, publicly available on the internet or
elsewhere without permission. Thanks.

WARNING: This is experimental software and the author cannot
assume any responsibility for its use (or misuse). If you do 
not know how to prevent your robots motors from accidentally 
running away, please do not use this software.

Copyright ©2006-2008 R. Kevin Watson. All rights are reserved.

***************************************************************

PWM() uses the CCP hardware within the robot controller's
CPU to precisely control the PWM pulse width without software
intervention.

In addition to the pulse width, the center point and gain of
each PWM channel is completely programmable with a resolution
of 100 ns. This is a cool feature because it allows you to do
things like map the entire 0 to 255 range to a very narrow
range of servo travel for greater position accuracy, or to a
narrower range of velocities from your motor. 

This source code will work with the PIC18F8520-based FIRST 
Robotics robot controller used in 2004/2005 and the PIC18F8722-
based robot controller used since 2006.


***************************************************************

Here's a description of the functions in pwm.c:

Initialize_PWM()

This function initializes the PWM software and is called
from ifi_frc.c/main().

PWM()

Each time this function is called, one and only one pulse is
generated on each of the four outputs. This function is
called from ifi_frc.c/main() with an update rate of just over
thirty-eight hertz. PWM() can be called from other sections
of your code for higher control rates, but keep in mind that
servos won't handle a rate much higher than that thirty-eight
hertz. On the other hand, IFI's Victors can handle control
rates of at least one-hundred hertz, which is nice for
applications like position control.


Kevin Watson
kevinw@jpl.nasa.gov
