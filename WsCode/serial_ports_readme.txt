***************************************************************

You are free to use this source code for any non-commercial
use. Please do not make copies of this source code, modified
or un-modified, publicly available on the internet or
elsewhere without permission. Thanks.

Copyright ©2004-2008 R. Kevin Watson. All rights are reserved.

***************************************************************

The source code in serial_ports.c/.h contains a software 
implementation of a fully buffered, interrupt-driven, full-
duplex serial port driver that can be used with either or both 
on-board serial ports. This software is also specifically
designed to work with the output stream functions included 
with Microchip's C18 C compiler starting with version 2.4  

This source code will work with the PIC18F8520-based FIRST 
Robotics robot controller, the PIC18F8722-based FIRST Robotics
robot controller, and the Robovation (A/K/A EDU-RC) robot 
controller.

Because you can now easily receive data from another computer, 
you can interact with your nifty IFI robot controller in real-
time to change operating parameters on-the-fly using common 
terminal emulation software, or send real telemetry to custom 
applications written with Visual Basic, Visual C++, MATLAB, 
etc... Don't want to drag a long serial cable behind your 'bot? 
Well, check-out the nifty SMiRF radio modem from SparkFun 
Electronics (http://www.sparkfun.com). Would the coolness 
factor of your 'bot be elevated if you had a LCD mounted on 
board to display diagnostics (yes, this is a rhetorical 
question)? How about using one of the serial LCDs that can be 
found on the 'net? I've had success using Scott Edward's 
Electronics (http://www.seetron.com) serial LCDs. The TRM-425L 
will work with the TTL-level serial port two and also includes 
a keypad interface. I've been mostly using the BPP-420L on 
serial port one. To use the above devices you'll need to build 
a simple three or four conductor cable. Disclaimer: Other than 
being a satisfied customer, I have no interest (financially, or 
otherwise) in the companies mentioned above.

By default, serial port one will operate at 115200 baud, which
is compatible with InnovationFIRST's terminal program, and 
serial port two will operate at 9600 baud, which will work with 
the above mentioned peripheral devices.

***************************************************************

Here's a description of the functions in serial_ports.c:


Init_Serial_Port_One()
Init_Serial_Port_Two()

These functions initialize the serial ports. This is where 
you will set the baud rate for each of the serial ports
using one of these predefined values:

BAUD_4800
BAUD_9600
BAUD_14400
BAUD_19200
BAUD_38400
BAUD_57600
BAUD_115200
BAUD_230400

One or both of these functions must be called before any
serial port operations can take place.


Serial_Port_One_Byte_Count()
Serial_Port_Two_Byte_Count()

These functions will return the number of bytes present in
their respective received data queues. Because there might
not be any data in the queues, these functions must be called 
before you can read any data from a serial port.


Read_Serial_Port_One()
Read_Serial_Port_Two()

These functions will return the next byte from the received
data queue. If no data is present in the queue, the function 
will return the number zero, which could cause problems if
your incoming data can also contain a zero. This is why the
Serial_Port_xxx_Byte_Count() functions must be called first.


Write_Serial_Port_One()
Write_Serial_Port_Two()

These functions put a byte of data on the serial port transmit 
queue. If the queue is full, the function will make you wait
until a storage slot becomes available before allowing your
code to execute again.


Rx_1_ISR()
Rx_2_ISR()

When a new byte of data is received by the serial port, the 
microcontroller will automatically call these functions to
get the new data and place it in the received data queue for
you. You shouldn't have to call these functions yourself.


Tx_1_ISR()
Tx_2_ISR()

When the serial port is ready to start sending a new byte of 
data, the microcontroller will automatically call these 
functions to get the next byte of data from the transmission 
queue and give it to the serial port for transmission. You 
shouldn't have to call these functions yourself.


_user_putc()

This function is the "glue" that interfaces the C18 output
stream functions to this serial port driver. If the global
variable stdout  is set to "_H_USER", which is defined in 
stdio.h, the C18 output stream functions will call this
function to send data to a serial port rather than the
library function putc(). Stdout is set to _H_USER within the
serial port initialization functions Init_Serial_Port_One()
and Init_Serial_Port_Two() described above.

Kevin Watson
kevinw@jpl.nasa.gov
