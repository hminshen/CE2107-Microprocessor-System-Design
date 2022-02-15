# CE2107-Microprocessor-System-Design

The coursework program was adapted from the TI-RSLK University Program curriculum, with the breakdown for labs and courseworks being as follows:

Lab 1: Introduction about Cortex M4F, ARM assembly, Embedded C programming, TI RSLK robot kit.

Lab 2: Introduction about GPIO and Interrupts in Cortex M4F.

Lab 3: Introduction about Timer Compare interrupt.

Lab 4: Introduction about Timer Capture interrup and ADC.

Lab 5: Assessment with Self-test program of the robot.

Lab 5 Self test program includes the following additional features:
1. Remote control movement of the robot using keyboard direction button press (using UART)
2. Obstacle Avoidance using IR sensors and bump sensor switches, which are triggered by edge interrupts
3. Line following using FSM implementation
4. High and low speed settings for robot movement using keyboard button press

Notes:
- Lab5_UARTmain.c in the Lab5_UARTComms folder is used for the Lab 5 self-test program
- The other C files that are outside are mainly used to implement the individual functionalities for the labs 1-4 and the self-test program
