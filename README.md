SwitchControl
=============

Monitors a network connection and reset a switch (or any other device) if that fails.

I created this sketch because some cheap switches in our office tend to faif after
some days of operation and need to be reset manually.

The sketch was tested on an Arduino Meda 2560, an ethernet shield, a relay connected
to PIN 12 and a HC-05 bluetooth PBC connected to the default serial port.
After the connection timed out several times, the relay will disconnect the power line
to the switches for 2 seconds.

To test the connection, the uC sends a ping command to a remote IP and awaits the repsonse
in a timely fashion. 

The sketch will accept adminitrator commands on the serial port (thus on bluetooth) to
modify the target IP, the timeout and the retry count and store them opermanantly in EEProm.


