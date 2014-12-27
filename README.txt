README:

The BPS project is currently implemented all in 1 .ino file. This one file includes the contents of three other files
just pasted into the .ino file. I did this because Arduino was being incredibly difficult, as it does, with the linking process. The original files are:

BPS_DEF.h --- Includes all of the definitions for the BPS setpoints as well as the I/O pins used on the arduino
BPS.h ------- Header file for BPS.c, includes definition of two structs, BPS and DEF, which are globally shared
BPS.c ------- Implements most of the code in the project. Includes functions that implement high level tasks,
	functions that help tasks, and also low-level functions to access the ADC and I2C
BPS_RTOS.ino- Includes definitions for all of the tasks and calls functions in BPS.c to implement tasks.

This code begins by calling the startup task which initializes everything and then waits for a signal that everything is good before starting the car. The next task to run is the BPScheck task which upon startupt waits for data to come in from all of the sensors before checking if the car is good. At that point, it continuously checks if the parameters are in range and edits status info in the global Structs accordingly.

The Other tasks run periodically and either collect information or report it out. Note that the USB task is the only task that prints from the serial monitor.



To DO:

Add code in function "Iget" to implement current sensing. Function currently just returns 0 current. Needs to call ADCget(pin_Isense) and convert this ADC value to a current in milliamps.

Add code in function "Vget" to implement voltage sensing. Currently returns 3200000 Startup code will need to be added in the BPSinit function to communicate with SPI. Another function will need to be written to handle charge-balancing.


Edit "BPScheck" so that the temperature limit while charging is different than the temperature limit while discharging

Edit "BPScheck" so that the voltage measured is compensated for current

Add options for DIP switches to effect operation like disabling current sensing or moving to an alternate current sensor

Add options for a user button to control things such as what the sevenseg screen shows.

Correct potential atomic code problems .... readers / writers should be mutually excluded.
	This may not be an issue depending on how NiLRTOS is implemented

  
