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
Sheet 6 6
Title "Base Arduino"
Date "Dom 15 Fev 2015"
Rev "1.0"
Comp ""
Comment1 "Basic Arduino setup"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATMEGA328-P IC1
U 1 1 54D7F099
P 3600 3750
F 0 "IC1" H 2850 5000 40  0000 L BNN
F 1 "ATMEGA328-P" H 4000 2350 40  0000 L BNN
F 2 "Housings_DIP:DIP-28_W7.62mm_LongPads" H 3600 3750 30  0001 C CIN
F 3 "" H 3600 3750 60  0000 C CNN
	1    3600 3750
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR015
U 1 1 54D7F6B2
P 2550 5150
F 0 "#PWR015" H 2550 4900 60  0001 C CNN
F 1 "GNDREF" H 2550 5000 60  0000 C CNN
F 2 "" H 2550 5150 60  0000 C CNN
F 3 "" H 2550 5150 60  0000 C CNN
	1    2550 5150
	1    0    0    -1  
$EndComp
NoConn ~ 2700 3250
$Comp
L +5V #PWR016
U 1 1 54D7F7B3
P 2550 2300
F 0 "#PWR016" H 2550 2150 60  0001 C CNN
F 1 "+5V" H 2550 2440 60  0000 C CNN
F 2 "" H 2550 2300 60  0000 C CNN
F 3 "" H 2550 2300 60  0000 C CNN
	1    2550 2300
	1    0    0    -1  
$EndComp
$Comp
L Crystal X1
U 1 1 54D7F8BC
P 6750 3300
F 0 "X1" H 6750 3450 50  0000 C CNN
F 1 "CRYSTAL" H 6750 3150 50  0000 C CNN
F 2 "Crystals:HC-49V" H 6750 3300 60  0001 C CNN
F 3 "" H 6750 3300 60  0000 C CNN
	1    6750 3300
	0    -1   -1   0   
$EndComp
$Comp
L C C5
U 1 1 54D7FA47
P 7200 2950
F 0 "C5" H 7250 3050 50  0000 L CNN
F 1 "22p" H 7250 2850 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L7_W2_P5" H 7238 2800 30  0001 C CNN
F 3 "" H 7200 2950 60  0000 C CNN
	1    7200 2950
	0    1    1    0   
$EndComp
$Comp
L C C6
U 1 1 54D7FAD7
P 7200 3650
F 0 "C6" H 7250 3750 50  0000 L CNN
F 1 "22p" H 7250 3550 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L7_W2_P5" H 7238 3500 30  0001 C CNN
F 3 "" H 7200 3650 60  0000 C CNN
	1    7200 3650
	0    1    1    0   
$EndComp
$Comp
L GNDREF #PWR017
U 1 1 54D7FB6B
P 7500 3800
F 0 "#PWR017" H 7500 3550 60  0001 C CNN
F 1 "GNDREF" H 7500 3650 60  0000 C CNN
F 2 "" H 7500 3800 60  0000 C CNN
F 3 "" H 7500 3800 60  0000 C CNN
	1    7500 3800
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR018
U 1 1 54D7FCBC
P 7700 4600
F 0 "#PWR018" H 7700 4350 60  0001 C CNN
F 1 "GNDREF" H 7700 4450 60  0000 C CNN
F 2 "" H 7700 4600 60  0000 C CNN
F 3 "" H 7700 4600 60  0000 C CNN
	1    7700 4600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR019
U 1 1 54D7FCD0
P 7700 3500
F 0 "#PWR019" H 7700 3350 60  0001 C CNN
F 1 "+5V" H 7700 3640 60  0000 C CNN
F 2 "" H 7700 3500 60  0000 C CNN
F 3 "" H 7700 3500 60  0000 C CNN
	1    7700 3500
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 54D7FCE4
P 7700 4350
F 0 "C7" H 7750 4450 50  0000 L CNN
F 1 "1u" H 7750 4250 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L7_W4.5_P5" H 7738 4200 30  0001 C CNN
F 3 "" H 7700 4350 60  0000 C CNN
	1    7700 4350
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 54D7FD1C
P 7700 3800
F 0 "R4" V 7780 3800 50  0000 C CNN
F 1 "10k" V 7707 3801 50  0000 C CNN
F 2 "Discret:R3" V 7630 3800 30  0001 C CNN
F 3 "" H 7700 3800 30  0000 C CNN
	1    7700 3800
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 54D7FEF8
P 5200 5500
F 0 "R2" V 5280 5500 50  0000 C CNN
F 1 "470" V 5207 5501 50  0000 C CNN
F 2 "Discret:R3" V 5130 5500 30  0001 C CNN
F 3 "" H 5200 5500 30  0000 C CNN
	1    5200 5500
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 54D7FF4D
P 5200 5700
F 0 "R3" V 5280 5700 50  0000 C CNN
F 1 "470" V 5207 5701 50  0000 C CNN
F 2 "Discret:R3" V 5130 5700 30  0001 C CNN
F 3 "" H 5200 5700 30  0000 C CNN
	1    5200 5700
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 54D8030D
P 5050 4750
F 0 "R5" V 5130 4750 50  0000 C CNN
F 1 "10k" V 5057 4751 50  0000 C CNN
F 2 "Discret:R3" V 4980 4750 30  0001 C CNN
F 3 "" H 5050 4750 30  0000 C CNN
	1    5050 4750
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR020
U 1 1 54D8038E
P 5050 5200
F 0 "#PWR020" H 5050 4950 60  0001 C CNN
F 1 "GNDREF" H 5050 5050 60  0000 C CNN
F 2 "" H 5050 5200 60  0000 C CNN
F 3 "" H 5050 5200 60  0000 C CNN
	1    5050 5200
	1    0    0    -1  
$EndComp
NoConn ~ 4600 4750
NoConn ~ 4600 4650
NoConn ~ 4600 4550
NoConn ~ 4600 3900
NoConn ~ 4600 4000
NoConn ~ 4600 2650
$Comp
L R R6
U 1 1 54D809E7
P 4900 2150
F 0 "R6" V 4980 2150 50  0000 C CNN
F 1 "10k" V 4907 2151 50  0000 C CNN
F 2 "Discret:R3" V 4830 2150 30  0001 C CNN
F 3 "" H 4900 2150 30  0000 C CNN
	1    4900 2150
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 54D80AC6
P 5100 2150
F 0 "R7" V 5180 2150 50  0000 C CNN
F 1 "10k" V 5107 2151 50  0000 C CNN
F 2 "Discret:R3" V 5030 2150 30  0001 C CNN
F 3 "" H 5100 2150 30  0000 C CNN
	1    5100 2150
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR021
U 1 1 54D80C81
P 5000 1700
F 0 "#PWR021" H 5000 1550 60  0001 C CNN
F 1 "+5V" H 5000 1840 60  0000 C CNN
F 2 "" H 5000 1700 60  0000 C CNN
F 3 "" H 5000 1700 60  0000 C CNN
	1    5000 1700
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 54D81783
P 6500 1650
F 0 "R8" V 6580 1650 50  0000 C CNN
F 1 "100k" V 6507 1651 50  0000 C CNN
F 2 "Discret:R3" V 6430 1650 30  0001 C CNN
F 3 "" H 6500 1650 30  0000 C CNN
	1    6500 1650
	0    1    1    0   
$EndComp
$Comp
L R R9
U 1 1 54D8179E
P 6500 1900
F 0 "R9" V 6580 1900 50  0000 C CNN
F 1 "100k" V 6507 1901 50  0000 C CNN
F 2 "Discret:R3" V 6430 1900 30  0001 C CNN
F 3 "" H 6500 1900 30  0000 C CNN
	1    6500 1900
	0    1    1    0   
$EndComp
$Comp
L C C8
U 1 1 54D8180B
P 6900 2200
F 0 "C8" H 6950 2300 50  0000 L CNN
F 1 "100n" H 6950 2100 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L7_W2.5_P5" H 6938 2050 30  0001 C CNN
F 3 "" H 6900 2200 60  0000 C CNN
	1    6900 2200
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 54D81846
P 7300 2200
F 0 "C9" H 7350 2300 50  0000 L CNN
F 1 "100n" H 7350 2100 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L7_W2.5_P5" H 7338 2050 30  0001 C CNN
F 3 "" H 7300 2200 60  0000 C CNN
	1    7300 2200
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR022
U 1 1 54D8194E
P 7100 2600
F 0 "#PWR022" H 7100 2350 60  0001 C CNN
F 1 "GNDREF" H 7100 2450 60  0000 C CNN
F 2 "" H 7100 2600 60  0000 C CNN
F 3 "" H 7100 2600 60  0000 C CNN
	1    7100 2600
	1    0    0    -1  
$EndComp
Text HLabel 5250 4450 2    60   Input ~ 0
SWITCH
Text HLabel 5300 3600 2    60   Input ~ 0
CH1_PEDAL_SIG
Text HLabel 5300 3500 2    60   Input ~ 0
CH0_PEDAL_SIG
Text HLabel 5650 5500 2    60   Output ~ 0
LED1
Text HLabel 5650 5700 2    60   Output ~ 0
LED2
Text HLabel 5200 2950 2    60   Output ~ 0
OUTPUT_SELECTION
Text HLabel 7650 1900 2    60   Output ~ 0
CH1_ARDUINO_SIG
Text HLabel 7650 1650 2    60   Output ~ 0
CH0_ARDUINO_SIG
$Comp
L +5V #PWR023
U 1 1 57BC9586
P 1650 3400
F 0 "#PWR023" H 1650 3250 60  0001 C CNN
F 1 "+5V" H 1650 3540 60  0000 C CNN
F 2 "" H 1650 3400 60  0000 C CNN
F 3 "" H 1650 3400 60  0000 C CNN
	1    1650 3400
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR024
U 1 1 57BC95BE
P 1650 3800
F 0 "#PWR024" H 1650 3550 60  0001 C CNN
F 1 "GNDREF" H 1650 3650 60  0000 C CNN
F 2 "" H 1650 3800 60  0000 C CNN
F 3 "" H 1650 3800 60  0000 C CNN
	1    1650 3800
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 57BC9625
P 1650 3600
AR Path="/54D7EFEF/57BC9625" Ref="C10"  Part="1" 
AR Path="/54D7C565/57BC9625" Ref="C?"  Part="1" 
F 0 "C10" H 1675 3700 50  0000 L CNN
F 1 "100nF" H 1675 3500 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L7_W2.5_P5" H 1688 3450 50  0001 C CNN
F 3 "" H 1650 3600 50  0000 C CNN
	1    1650 3600
	1    0    0    -1  
$EndComp
Text HLabel 4200 2000 0    60   Input ~ 0
SCK
Text HLabel 4200 2100 0    60   Input ~ 0
MISO
Text HLabel 4200 2200 0    60   Output ~ 0
MOSI
Wire Wire Line
	2550 4850 2550 5150
Wire Wire Line
	2550 4850 2700 4850
Wire Wire Line
	2700 4950 2550 4950
Connection ~ 2550 4950
Wire Wire Line
	2550 2300 2550 2950
Wire Wire Line
	2550 2650 2700 2650
Wire Wire Line
	2550 2950 2700 2950
Connection ~ 2550 2650
Wire Wire Line
	6200 3650 7050 3650
Wire Wire Line
	6750 3450 6750 3650
Wire Wire Line
	6200 2950 7050 2950
Wire Wire Line
	6750 2950 6750 3150
Connection ~ 6750 3650
Connection ~ 6750 2950
Wire Wire Line
	7500 2950 7500 3800
Wire Wire Line
	7350 2950 7500 2950
Wire Wire Line
	7350 3650 7500 3650
Connection ~ 7500 3650
Wire Wire Line
	7700 4500 7700 4600
Wire Wire Line
	7700 3950 7700 4200
Wire Wire Line
	7700 3500 7700 3650
Wire Wire Line
	4600 4100 7700 4100
Connection ~ 7700 4100
Wire Wire Line
	4600 4950 4650 4950
Wire Wire Line
	4650 4950 4650 5700
Wire Wire Line
	4650 5700 5050 5700
Wire Wire Line
	4600 4850 4750 4850
Wire Wire Line
	4750 4850 4750 5500
Wire Wire Line
	4750 5500 5050 5500
Wire Wire Line
	5350 5500 5650 5500
Wire Wire Line
	5350 5700 5650 5700
Wire Wire Line
	5050 4450 5050 4600
Connection ~ 5050 4450
Wire Wire Line
	5050 4900 5050 5200
Wire Wire Line
	4600 3500 5300 3500
Wire Wire Line
	4600 3600 5300 3600
Wire Wire Line
	4900 1850 4900 2000
Wire Wire Line
	4900 1850 5100 1850
Wire Wire Line
	5100 1850 5100 2000
Wire Wire Line
	5000 1700 5000 1850
Connection ~ 5000 1850
Wire Wire Line
	4600 2950 5200 2950
Wire Wire Line
	6650 1900 7650 1900
Wire Wire Line
	6900 1900 6900 2050
Wire Wire Line
	6650 1650 7650 1650
Wire Wire Line
	6900 2500 7300 2500
Wire Wire Line
	6900 2500 6900 2350
Wire Wire Line
	7100 2600 7100 2500
Connection ~ 7100 2500
Wire Wire Line
	7300 2500 7300 2350
Wire Wire Line
	7300 1650 7300 2050
Wire Wire Line
	4600 2850 6100 2850
Wire Wire Line
	6100 2850 6100 1900
Wire Wire Line
	6100 1900 6350 1900
Wire Wire Line
	4600 2750 6000 2750
Wire Wire Line
	6000 2750 6000 1650
Wire Wire Line
	6000 1650 6350 1650
Connection ~ 6900 1900
Connection ~ 7300 1650
Wire Wire Line
	4600 4450 5250 4450
Wire Wire Line
	1650 3800 1650 3750
Wire Wire Line
	1650 3450 1650 3400
Wire Wire Line
	4200 2200 4650 2200
Wire Wire Line
	4650 2200 4650 2950
Connection ~ 4650 2950
Wire Wire Line
	4200 2100 4700 2100
Wire Wire Line
	4700 2100 4700 3050
Wire Wire Line
	4700 3050 4600 3050
Wire Wire Line
	4200 2000 4750 2000
Wire Wire Line
	4750 2000 4750 3150
Wire Wire Line
	4750 3150 4600 3150
Text HLabel 6500 4300 3    60   Input ~ 0
RESET
Wire Wire Line
	6500 4300 6500 4100
Connection ~ 6500 4100
Text HLabel 4900 4250 2    60   Input ~ 0
RX
Text HLabel 4900 4350 2    60   Output ~ 0
TX
Wire Wire Line
	4900 4250 4600 4250
Wire Wire Line
	4600 4350 4900 4350
Wire Wire Line
	5100 2300 5100 3500
Connection ~ 5100 3500
Wire Wire Line
	4900 3600 4900 2300
Connection ~ 4900 3600
Text HLabel 5300 3700 2    60   Input ~ 0
CH0_OUT
Text HLabel 5300 3800 2    60   Input ~ 0
CH1_OUT
Wire Wire Line
	6200 2950 6200 3250
Wire Wire Line
	6200 3650 6200 3350
Wire Wire Line
	6200 3350 4600 3350
Wire Wire Line
	6200 3250 4600 3250
$Comp
L R R10
U 1 1 57BF3972
P 6350 3300
F 0 "R10" V 6430 3300 50  0000 C CNN
F 1 "1M" V 6350 3300 50  0000 C CNN
F 2 "Discret:R3" V 6280 3300 50  0001 C CNN
F 3 "" H 6350 3300 50  0000 C CNN
	1    6350 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 3150 6350 2950
Connection ~ 6350 2950
Wire Wire Line
	6350 3450 6350 3650
Connection ~ 6350 3650
Wire Wire Line
	4600 3700 5300 3700
Wire Wire Line
	5300 3800 4600 3800
$EndSCHEMATC
