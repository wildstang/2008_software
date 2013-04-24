****************************************************************

You are free to use this source code for any non-commercial
use. Please do not make copies of this source code, modified
or un-modified, publicly available on the internet or
elsewhere without permission. Thanks.

Copyright ©2004-2008 R. Kevin Watson. All rights are reserved.

****************************************************************
The source code in gyro.c/.h contains a driver and supporting 
software to interface a variety of inexpensive gyros to your
robot controller. This software was tested with Analog Devices'
ADXRS401, ADXRS150 and ADXRS300 gyros. Data sheets for these
devices are included. By default, this software is configured 
to work with a ADXRS150 gyro, sampling at 800Hz, downconverting
to an update rate of 50Hz by averaging sixteen samples per 
update. These parameters can be changed by editing gyro.h and/
or adc.h.

Version 0.5 of the gyro interface software has been altered
to use the ADC interface provided by adc.c/.h. This change 
allows the programmer to use additional ADC channels for other 
analog sensors without affecting the performance or operation 
of your gyro. See the documentation included with adc.c/.h for 
information on how to use this new functionality.

Another new feature is the ability to specify a measurement 
deadband centered about the gyro bias. ADC measurements within 
this deadband will not be used to calculate the gyro angle or 
angular rate. This feature can have a significant impact at 
minimizing short-term drift just after a bias calculation has
taken place. To use this feature, follow the instructions 
embedded within gyro.h.

This source code will work with the PIC18F8520-based FIRST 
Robotics robot controller, the PIC18F8722-based FIRST Robotics
robot controller, and the Robovation (A/K/A EDU-RC) robot 
controller.

Wiring-up the ADXRS401EB, ADXRS150EB and ADXRS300EB gyro
evaluation boards is straightforward: Grab a PWM cable and cut
off the male end and strip the three wires back a centimeter 
or so. With a low wattage soldering iron, solder the white wire 
to the RATEOUT pin, solder the black wire to the AGND pin, the 
red wire to the AVCC pin, and finally connect a jumper wire 
between the AVCC and PDD pins. Plug the female end into one of 
the robot controller's analog inputs. These gyro evaluation
boards can be purchased from analog devices (www.analog.com),
and Digi-Key (www.digikey.com). Another great source for
Analog Devices gyros is SparkFun Electronics (www.sparkfun.com).

For optimum performance, you'll need to calibrate the scaling
factor to match that of your gyro. One way to calibrate your 
gyro is to mount it very securely to a hefty, square or 
rectangular object. Mounting the gyro to a hefty object will 
help dampen higher frequency vibrations that can adversely 
effect your measurements. Place the mounted gyro against 
another square object and start the included demonstration 
software. To get good results, the mount must be absolutely 
still when the "Calibrating Gyro Bias..." message appears on 
the terminal screen. After a few seconds, the gyro angular rate 
and angle will be sent to the terminal screen. If the angle 
drifts rapidly while the mounted gyro is motionless, you need 
to restart the software to acquire a new gyro bias measurement. 
Again, gyros are very sensitive and must be still while the bias 
is calculated. Once the gyro is running with little drift, 
rotate the mount 180 degrees and note the reported angle. If the 
angular units are set to tenths of a degree, the ideal reported 
angle is 1800. If set to milliradians, the ideal angle 1s 3142 
(Pi times a thousand). For every tenth of a percent that the 
angle is high, decrease the GYRO_CAL_FACTOR numerator by one.
Conversly, for every tenth of a percent low, increase the 
numerator by one. Repeat until you're satisfied with the 
accuracy.

The included project files were built with MPLAB version 8.00.
If your version of MPLAB complains about the project version, 
the best thing to do is just create a new project with MPLAB's 
project wizard.

Support for this software can be received by posting a help
request in the programming section of the Chief Delphi forums
(http://www.chiefdelphi.com).

****************************************************************

Usage notes:

1) It is important to note that there is a limit to the
range over which this software will return valid data before
an overflow will take place. As there are many parameters
that effect when this will happen, your best bet is to
experiment by seeing how many times you can rotate your gyro
360 degrees before the sign on the heading flips. As an
example, using the default settings with a ADXRS150 gyro,
the valid range is about +/- 32767 milliradians, or just over
+/- five full rotations.

1) A #include statement for the gyro.h header file must be 
included at the beginning of each source file that calls the 
functions in gyro.c. The statement should look like this: 
#include "gyro.h".

2) Process_Gyro_Data() must be called when from the *_spin()
function for each mode you need the gyro to operate.

3) You must select the gyro you're using from a list in gyro.h
and if needed, remove the // in front of its #define. If you're
using the gyro from the 2008 KOP, use the default ADXRS150
gyro.

4) The default angular unit is milliradians. If desired, this
can be changed to tenths of a degree by editing gyro.h

5) A gyro bias calculation must take place using the functions 
Start_Gyro_Bias_Calc() & Stop_Gyro_Bias_Calc() described below.

6) For optimal performance, you'll need to calibrate the gyro 
scaling factor using the instructions above or those included
in gyro.h.

7) Consult the adc_readme.txt and adc.h files for additional
information
  
****************************************************************

Here's a description of the functions in gyro.c:


Initialize_Gyro()
This function initializes the gyro software. It should be called 
from teleop.c/Initialization().
 

Get_Gyro_Rate()
This function returns the current angular rate of change in 
units of milliradians per second. If desired, the angular unit 
can be changed to tenths of a degree per second by modifying
the angular units #define entry in gyro.h


Get_Gyro_Angle()
This function returns the change in heading angle, in 
milliradians, since the software was initialized or 
Reset_Gyro_Angle() was called. If desired, the angular unit
can be changed to tenths of a degree by modifying the angular
units #define entry in gyro


Start_Gyro_Bias_Calc()
Stop_Gyro_Bias_Calc()
These functions start/stop a new gyro bias calculation. For best 
results, Start_Gyro_Bias_Calc() should be called about a second 
after the robot powers-up and while the robot is perfectly still 
with all vibration sources turned off (e.g., compressor). After
at least 64 ADC updates have been acquired (~2 seconds using the
default configuration), call Stop_Gyro_Bias_Calc() to terminate
the calibration. The gyro bias calculation can also be monitored
with the Get_Gyro_Bias_Status() function described below. While a
calibration is taking place, gyro rate and angle are not updated.
Once calibrated, call Reset_Gyro_Angle() to reset the headding 
angle to zero.


Get_Gyro_Bias_Status()
This function can be called to determine the status of your gyro
bias calculation. It returns these values:

GYRO_BIAS_NOT_DONE: This is returned if a gyro bias has not been
calculated.

GYRO_BIAS_IN_PROCESS: This is returned after Start_Gyro_Bias_Calc()
has been called and data is being collected.

GYRO_BIAS_BUFFER_FULL: This is returned if the circular buffer is
full, indicating that it is safe to call Stop_Gyro_Bias_Calc() at
any time.

GYRO_BIAS_READY: This is returned if the gyro bias calculation is
complete and the gyro is returning data.


Get_Gyro_Bias()
This function returns the current gyro bias.


Set_Gyro_Bias()
This function can be called to set the gyro bias.


Reset_Gyro_Angle()
This function resets the integrated gyro heading angle to zero.


Process_Gyro_Data()
This function manages ADC data and does gyro rate integration.
This function must be called when from the *_spin() function 
for each mode you need the gyro to operate.


Kevin Watson
kevinw@jpl.nasa.gov
