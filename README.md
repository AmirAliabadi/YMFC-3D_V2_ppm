# YMFC-3D_V2 with PPM support 
## This is a fork of Joop Brokkings amazing YMFC-3D_V2 arduino flight controller

http://www.brokking.net for more info.

I will be updating with the following changes:
* Add PPM input instead of the PWM.  Fewer wires
* Update to use a center postion throttle.


## Full Disclamer:
This is a fork from Joop.  Original ReadMe.txt is included.

Thank you for downloading the YMFC-3D V2 software package. 

Version 2.01 - January 22, 2016

Content:
* YMFC-3D_V2_setup.ino
* YMFC-3D_V2_esc_calibrate.ino
* YMFC-3D_V2_Flight_controller.ino
* YMFC-3D_scematic.jpg

Revision update:
* 2.01
** Bug fix in line 119, 130, 142 and 154. Printing of the digital inputs is not correct.
** 0b00000011 is changed in 0b00000111.
** This only involves the printed output during setup and not the flight controller software. (Thanks Freddie for finding this)