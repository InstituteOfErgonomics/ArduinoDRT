ArduinoDRT
==========

Arduino Detection Response Task

for more information and hardware description please visit http://www.lfe.mw.tum.de/en/arduinodrt

----------

Detection response tasks (DRT) are methods which try to get objective values for the mental workload of a task or a combination of tasks (e.g. to drive a car and navigate through an infotainment system). The task (or task combination) is executed by test subjects in an experiment. The test subjects do the task (combination) under test, while trying to operate a DRT. The values from the DRT are interpreted as measurement for the tasks under test.

In a DRT the test subjects has to detect a stimulus which comes up randomly, typically between 3 and 5 seconds and press quickly a response button. The stimulus is typically visual (red LED) and mounted to the test subjects head (head mounted DRT) or tactile (TDRT) by a vibrating tactor.

If the task (or task combination) under test needs cognitive effort from the test subjects the reaction times to the stimulus gets slower. Sometimes test subject under heavy cognitive stress even miss to detect the stimulus or do not response by a button press. The described hardware and software gathers these reaction data with an Arduino.
The DRTs are currently in a standardization process (ISO), the values presented here are not necessarely reflect 'standardized' values or values in discussion by a working group.

----------

First we introduce the Plain-Arduino-DRT on an Arduino Uno. It just do a DRT experimental protocol and saves the results on a SD card. The experiment can be controlled via serial commands (like start/stop) or manually with a start/stop button and the results can also be transmitted via USB/COM.

Than we expand this to the Ethernet-Arduino-DRT with an Arduino Uno and an Ethernet shield. It does the same, controlled via Ethernet and can also transmits its results via the network. Therefor, it can be connected, e.g. to a driving simulator.

At last we expand the Ethernet-Arduino-DRT to the Mega-Arduino-DRT on an Arduino Mega and an Ethernet shield. Thus, we have more storage for program code and more hardware pins for connections. These freedom is used in an example to implement a setup with up to eight stimulus LEDs. Additionally we include a real time clock, DHCP and NTP functions.

----------

Always remember, the sources are intentionally open source (GPL), thus you can adjust the projects to your needs. No matter, if you want to transfer it to an Arduino Nano, connect a GPS module or LCD, implement another transfer protocol which is better suited for your driving simulator, etc.

Just one note: Please report bugs via the github repositories or mail to krause@tum.de

If you connect wires and equipment to a subject, please do it with caution and obey e.g., electric safety.

If setups are used while driving, special care must be taken. E.g. no cables or parts should interfere with the safe driving task. Think about what can fail while preparing the experiment (something gets loose, gets stuck to other parts,...).
