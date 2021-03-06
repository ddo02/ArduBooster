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
Sheet 3 6
Title "Doubler/Inverter"
Date "Dom 15 Fev 2015"
Rev "1.0"
Comp ""
Comment1 "Used to ensure that OpAmp output will cover all  possible voltages (0V to 5V)"
Comment2 "Output: -5V/9V"
Comment3 "Input: 0V/5V"
Comment4 "Doubler/Inverter - Power source to OpAmp"
$EndDescr
$Comp
L ICL7660 U1
U 1 1 54D7C56E
P 5250 3700
F 0 "U1" H 5450 4100 70  0000 L CNN
F 1 "ICL7660" H 5300 3250 70  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_LongPads" H 5250 3700 60  0001 C CNN
F 3 "" H 5250 3700 60  0000 C CNN
	1    5250 3700
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 54D7C587
P 3800 3550
F 0 "C1" H 3850 3650 50  0000 L CNN
F 1 "1u" H 3850 3450 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W4.5mm_P5.00mm" H 3838 3400 30  0001 C CNN
F 3 "" H 3800 3550 60  0000 C CNN
	1    3800 3550
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 54D7C5F6
P 5650 4550
F 0 "C2" H 5700 4650 50  0000 L CNN
F 1 "1u" H 5700 4450 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W4.5mm_P5.00mm" H 5688 4400 30  0001 C CNN
F 3 "" H 5650 4550 60  0000 C CNN
	1    5650 4550
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 54D7C60F
P 7200 4950
F 0 "C3" H 7250 5050 50  0000 L CNN
F 1 "1u" H 7250 4850 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W4.5mm_P5.00mm" H 7238 4800 30  0001 C CNN
F 3 "" H 7200 4950 60  0000 C CNN
	1    7200 4950
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 54D7C66C
P 7600 4950
F 0 "C4" H 7650 5050 50  0000 L CNN
F 1 "1u" H 7650 4850 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W4.5mm_P5.00mm" H 7638 4800 30  0001 C CNN
F 3 "" H 7600 4950 60  0000 C CNN
	1    7600 4950
	1    0    0    -1  
$EndComp
$Comp
L D D1
U 1 1 54D7C6AB
P 6400 4150
F 0 "D1" H 6400 4250 50  0000 C CNN
F 1 "1n4007" H 6400 4050 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P2.54mm_Vertical_AnodeUp" H 6400 4150 60  0001 C CNN
F 3 "" H 6400 4150 60  0000 C CNN
	1    6400 4150
	0    -1   -1   0   
$EndComp
$Comp
L D D2
U 1 1 54D7C6F0
P 6800 4550
F 0 "D2" H 6800 4650 50  0000 C CNN
F 1 "1n4007" H 6800 4450 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P2.54mm_Vertical_AnodeUp" H 6800 4550 60  0001 C CNN
F 3 "" H 6800 4550 60  0000 C CNN
	1    6800 4550
	-1   0    0    1   
$EndComp
NoConn ~ 4400 3850
NoConn ~ 6100 3850
$Comp
L GNDREF #PWR09
U 1 1 54D7CA46
P 7400 5350
F 0 "#PWR09" H 7400 5100 60  0001 C CNN
F 1 "GNDREF" H 7400 5200 60  0000 C CNN
F 2 "" H 7400 5350 60  0000 C CNN
F 3 "" H 7400 5350 60  0000 C CNN
	1    7400 5350
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR010
U 1 1 54D7CAA6
P 5200 4700
F 0 "#PWR010" H 5200 4450 60  0001 C CNN
F 1 "GNDREF" H 5200 4550 60  0000 C CNN
F 2 "" H 5200 4700 60  0000 C CNN
F 3 "" H 5200 4700 60  0000 C CNN
	1    5200 4700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR011
U 1 1 54D7CB10
P 5200 2800
F 0 "#PWR011" H 5200 2650 60  0001 C CNN
F 1 "+5V" H 5200 2940 60  0000 C CNN
F 2 "" H 5200 2800 60  0000 C CNN
F 3 "" H 5200 2800 60  0000 C CNN
	1    5200 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 3450 4000 3450
Wire Wire Line
	4000 3450 4000 3250
Wire Wire Line
	4000 3250 3800 3250
Wire Wire Line
	3800 3250 3800 3400
Wire Wire Line
	4400 3650 4000 3650
Wire Wire Line
	4000 3650 4000 3850
Wire Wire Line
	4000 3850 3800 3850
Wire Wire Line
	3800 3850 3800 3700
Wire Wire Line
	4250 4550 5500 4550
Wire Wire Line
	4250 4550 4250 3450
Connection ~ 4250 3450
Wire Wire Line
	5800 4550 6650 4550
Wire Wire Line
	6400 4300 6400 4550
Connection ~ 6400 4550
Wire Wire Line
	6400 2950 6400 4000
Wire Wire Line
	6400 2950 5200 2950
Wire Wire Line
	5200 2800 5200 3050
Wire Wire Line
	6950 4550 7900 4550
Wire Wire Line
	7200 4550 7200 4800
Wire Wire Line
	7600 3550 7600 4800
Wire Wire Line
	6100 3550 7900 3550
Connection ~ 7600 3550
Connection ~ 7200 4550
Wire Wire Line
	7600 5100 7600 5250
Wire Wire Line
	7600 5250 7200 5250
Wire Wire Line
	7200 5250 7200 5100
Wire Wire Line
	7400 5350 7400 5250
Connection ~ 7400 5250
Wire Wire Line
	5200 4700 5200 4350
Connection ~ 5200 2950
$Comp
L PWR_FLAG #FLG012
U 1 1 54D7D3CB
P 7750 4350
F 0 "#FLG012" H 7750 4445 30  0001 C CNN
F 1 "PWR_FLAG" H 7750 4530 30  0000 C CNN
F 2 "" H 7750 4350 60  0000 C CNN
F 3 "" H 7750 4350 60  0000 C CNN
	1    7750 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 4350 7750 4550
Connection ~ 7750 4550
Text HLabel 7900 3550 2    60   Input ~ 0
-5V
Text HLabel 7900 4550 2    60   Output ~ 0
+9V
$EndSCHEMATC
