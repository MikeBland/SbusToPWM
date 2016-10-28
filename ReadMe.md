Arduino used as SBUS to 16-channels PPM decoder

Analog input A4 is used to set the failsafe times to the current received pulses.
Connect a switch to this and connect to ground to set the failsafe.
Failsafe is entered if no SBUS signal is being received.

Analog input A5 is an option, if grounded, the outputs for channels 1-8 and 9-16
are swapped.

The Tx pin is another option link. If left open, all works as 16 channels.
If the Tx pin is connected to ground, via a 1K resistor, then it switches 
to 8 channel only mode, but with a servo update rate of 9mS.

Connections:
<img src="Docs/Wiring.jpg"/>

