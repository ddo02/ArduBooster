EESchema Schematic File Version 2
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
Sheet 5 6
Title "Mux/Demux source selector"
Date "Dom 15 Fev 2015"
Rev "1.0"
Comp ""
Comment1 "If OUTPUT_SELECTION is \"0\", pedal signal is used"
Comment2 "If OUTPUT_SELECTION is \"1\", Arduino signal is used"
Comment3 "Used as workaround to Arduino boot dalay"
Comment4 "Select output source"
$EndDescr
$Comp
L 4052 U3
U 1 1 54D7D70E
P 5800 3800
F 0 "U3" H 5900 3800 60  0000 C CNN
F 1 "4052" H 5900 3600 60  0000 C CNN
F 2 "Sockets_DIP:DIP-16__300" H 5800 3800 60  0001 C CNN
F 3 "" H 5800 3800 60  0000 C CNN
	1    5800 3800
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR011
U 1 1 54D7D759
P 5800 4700
F 0 "#PWR011" H 5800 4450 60  0001 C CNN
F 1 "GNDREF" H 5800 4550 60  0000 C CNN
F 2 "" H 5800 4700 60  0000 C CNN
F 3 "" H 5800 4700 60  0000 C CNN
	1    5800 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 4300 6600 4300
Wire Wire Line
	6600 4300 6600 4650
Wire Wire Line
	6600 4650 4700 4650
Wire Wire Line
	5800 4450 5800 4700
Wire Wire Line
	5100 4100 4950 4100
Wire Wire Line
	4950 4100 4950 4650
Connection ~ 5800 4650
Wire Wire Line
	5100 4400 4950 4400
Connection ~ 4950 4400
$Comp
L R R1
U 1 1 54D7D7B4
P 4700 4350
F 0 "R1" V 4780 4350 50  0000 C CNN
F 1 "10k" V 4707 4351 50  0000 C CNN
F 2 "Discret:R3" V 4630 4350 30  0001 C CNN
F 3 "" H 4700 4350 30  0000 C CNN
	1    4700 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 4300 4850 4300
Wire Wire Line
	4850 4300 4850 4050
Wire Wire Line
	4850 4050 4200 4050
Wire Wire Line
	4700 4050 4700 4200
Wire Wire Line
	4700 4650 4700 4500
Connection ~ 4950 4650
Connection ~ 4700 4050
Wire Wire Line
	6800 3200 6500 3200
Wire Wire Line
	6800 3600 6500 3600
Wire Wire Line
	4850 3200 5100 3200
Wire Wire Line
	4850 3300 5100 3300
Wire Wire Line
	4850 3600 5100 3600
Wire Wire Line
	4850 3700 5100 3700
NoConn ~ 5100 3800
NoConn ~ 5100 3900
NoConn ~ 5100 3500
NoConn ~ 5100 3400
$Comp
L +5V #PWR012
U 1 1 54D7DB5C
P 5800 3000
F 0 "#PWR012" H 5800 2850 60  0001 C CNN
F 1 "+5V" H 5800 3140 60  0000 C CNN
F 2 "" H 5800 3000 60  0000 C CNN
F 3 "" H 5800 3000 60  0000 C CNN
	1    5800 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 3000 5800 3150
Text HLabel 4850 3200 0    60   Output ~ 0
CH0_PEDAL_SIG
Text HLabel 4850 3300 0    60   Output ~ 0
CH0_ARDUINO_SIG
Text HLabel 4850 3600 0    60   Output ~ 0
CH1_PEDAL_SIG
Text HLabel 4850 3700 0    60   Output ~ 0
CH1_ARDUINO_SIG
Text HLabel 4200 4050 0    60   Output ~ 0
OUTPUT_SELECTION
Text HLabel 6800 3600 2    60   Output ~ 0
CH1_MUXDEMUX_OUT
Text HLabel 6800 3200 2    60   Output ~ 0
CH0_MUXDEMUX_OUT
$EndSCHEMATC
