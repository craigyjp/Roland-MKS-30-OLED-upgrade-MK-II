An improved version of my earlier MKS-30 upgrade, now I'm using a single Teensy 3.6 to drive two SPI displays at the same time I can actually translate the LED signals into meaningful messages for the OLEDs. So when editing you can see each parameter etc and its real value rather than some symbols of what it could possible be.

This was inspired by the poor condition of the LED display of the Roland MKS-30, rather than just replace the LEDs with new more moderen and slightly larger versions, I thought it would be a good idea to use a pair of 0.96" OLED displays for each pair of characters and using the code by Aaron Williams that renders 7 segement displays onto OLED displays I came up with this design.

I enlarged the characters from the original code to better fill the display windows and added a shift register to capture the segment status directly before the LED display drivers. I also picked up the select lines for the 4 segments all from connector 1 of the MKS-30 display board using interrupts to get the correct timing.

+5V Power can be picked up from connector 3 of the display board also.

I've 3D printed some windows to hide the fact that the displays are about 1mm too small for the MKS-30 windows. These are not shown.
