EESchema Schematic File Version 2
LIBS:ArduBooster-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ArduBooster-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 6
Title "Output isolator"
Date "Dom 15 Fev 2015"
Rev "1.0"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L LM358-RESCUE-ArduBooster U2
U 1 1 54D7D0D1
P 5650 2800
F 0 "U2" H 5600 3000 60  0000 L CNN
F 1 "LM358" H 5600 2550 60  0000 L CNN
F 2 "Sockets_DIP:DIP-8__300" H 5650 2800 60  0001 C CNN
F 3 "" H 5650 2800 60  0000 C CNN
	1    5650 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 2900 5050 2900
Wire Wire Line
	5050 2900 5050 3300
Wire Wire Line
	5050 3300 6250 3300
Wire Wire Line
	6250 3300 6250 2800
Wire Wire Line
	5550 3500 5550 3200
Wire Wire Line
	5550 2150 5550 2400
$Comp
L LM358-RESCUE-ArduBooster U2
U 2 1 54D7D175
P 5650 4800
F 0 "U2" H 5600 5000 60  0000 L CNN
F 1 "LM358" H 5600 4550 60  0000 L CNN
F 2 "Sockets_DIP:DIP-8__300" H 5650 4800 60  0001 C CNN
F 3 "" H 5650 4800 60  0000 C CNN
	2    5650 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 5500 5550 5200
Wire Wire Line
	5150 4900 5050 4900
Wire Wire Line
	5050 4900 5050 5300
Wire Wire Line
	5050 5300 6250 5300
Wire Wire Line
	6250 5300 6250 4800
Wire Wire Line
	6150 4800 6500 4800
Wire Wire Line
	5550 4200 5550 4400
Wire Wire Line
	4400 2700 5150 2700
Connection ~ 6250 2800
Connection ~ 6250 4800
Wire Wire Line
	6150 2800 6500 2800
Wire Wire Line
	5150 4700 4400 4700
Text HLabel 5550 2150 1    60   Output ~ 0
+9V
Text HLabel 6500 2800 2    60   Output ~ 0
CH0_OUT
Text HLabel 5550 3500 3    60   Output ~ 0
-5V
Text HLabel 4400 2700 0    60   Output ~ 0
CH0_MUXDEMUX_OUT
Text HLabel 4400 4700 0    60   Output ~ 0
CH1_MUXDEMUX_OUT
Text HLabel 5550 4200 1    60   Output ~ 0
+9V
Text HLabel 6500 4800 2    60   Output ~ 0
CH1_OUT
Text HLabel 5550 5500 3    60   Output ~ 0
-5V
$EndSCHEMATC
