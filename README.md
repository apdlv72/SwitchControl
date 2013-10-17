SwitchControl
=============

Monitors a network connection and reset a switch (or any other device) if that fails.

I created this sketch because some cheap switches in our office tend to faif after
some days of operation and need to be reset manually.

The sketch was tested on an Arduino Meda 2560, an ethernet shield, a relay connected
to PIN 12 and a HC-05 bluetooth PBC connected to the default serial port.
After the connection timed out several times, the relay will disconnect the power line
to the switches for 2 seconds.

To test the connection, the uC can send a ping command to a remote IP and awaits the repsonse
in a timely fashion or it can do DHCP requests on a regular basis and check if these succeed.

When configured to synchronize its time with a NTP time server, office hours can be defined
during which resets of the controller switch should be suppresed.

The sketch will accept adminitrator commands on the serial port (thus on bluetooth) to
modify the target IP, the timeout and the retry count and store them opermanantly in EEProm:

	I=ip   set the IP address to ping to to check if the network works
	J=mac  set the mac address of the control
	N=ip   set the IP address of the time server to query
	F=1|0  fetch time from time server (1) or do not (0)
	T=4    timeout for ping requests (seconds)
	W=10   wait time between checks (seconds)
	Z=2    timezone difference in hours (2=CEST)
	R=10   set number of retries before a network outage is assumed
	P=1|0  pause on|off
	M=1|2  set monitor mode: 1=ping requests, 2=dhcp requests
	S      set office start hour
	E      set office end
	Y      same for fridays
	C      show config
	B=4711 reboot then micro controller
	X=4711 factory reset the micro controller
	H      help
