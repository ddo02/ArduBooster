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
Sheet 2 6
Title "Connectors"
Date "Dom 15 Fev 2015"
Rev "1.0"
Comp ""
Comment1 "Board connectors"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_01X05 P3
U 1 1 54D7F0D9
P 8200 4400
F 0 "P3" H 8200 4700 50  0000 C CNN
F 1 "CONTROL" V 8300 4400 50  0000 C CNN
F 2 "Connect:Wafer_Horizontal15x5.8x7RM2.5-5" H 8200 4400 60  0001 C CNN
F 3 "" H 8200 4400 60  0000 C CNN
	1    8200 4400
	1    0    0    1   
$EndComp
$Comp
L GNDREF #PWR01
U 1 1 54D7F247
P 7850 4750
F 0 "#PWR01" H 7850 4500 60  0001 C CNN
F 1 "GNDREF" H 7850 4600 60  0000 C CNN
F 2 "" H 7850 4750 60  0000 C CNN
F 3 "" H 7850 4750 60  0000 C CNN
	1    7850 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 4750 7850 4600
Wire Wire Line
	7850 4600 8000 4600
$Comp
L +5V #PWR02
U 1 1 54D7F26C
P 7850 4050
F 0 "#PWR02" H 7850 3900 60  0001 C CNN
F 1 "+5V" H 7850 4190 60  0000 C CNN
F 2 "" H 7850 4050 60  0000 C CNN
F 3 "" H 7850 4050 60  0000 C CNN
	1    7850 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 4200 7850 4200
Wire Wire Line
	7850 4200 7850 4050
Wire Wire Line
	7700 4500 8000 4500
Wire Wire Line
	7700 4400 8000 4400
Text HLabel 7700 4300 0    60   Input ~ 0
SWITCH
Wire Wire Line
	7700 4300 8000 4300
Text HLabel 7700 4500 0    60   Output ~ 0
LED1
Text HLabel 7700 4400 0    60   Output ~ 0
LED2
Text HLabel 4150 4500 2    60   Output ~ 0
CH1_OUT
Text HLabel 4150 4600 2    60   Output ~ 0
CH0_OUT
Text HLabel 4150 4200 2    60   Input ~ 0
CH1_PEDAL_SIG
Text HLabel 4150 4300 2    60   Input ~ 0
CH0_PEDAL_SIG
$Comp
L CONN_01X06 P4
U 1 1 54E8D924
P 3450 4350
F 0 "P4" H 3450 4800 50  0000 C CNN
F 1 "MAIN_CONN" V 3550 4350 50  0000 C CNN
F 2 "Connectors_Molex:Molex_MiniFit-JR-5556-06A_2x03x4.20mm_Straight" H 3450 4350 60  0001 C CNN
F 3 "" H 3450 4350 60  0000 C CNN
	1    3450 4350
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR03
U 1 1 54E8E644
P 3850 3850
F 0 "#PWR03" H 3850 3700 60  0001 C CNN
F 1 "+5V" H 3850 3990 60  0000 C CNN
F 2 "" H 3850 3850 60  0000 C CNN
F 3 "" H 3850 3850 60  0000 C CNN
	1    3850 3850
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG04
U 1 1 54E8E658
P 4100 3850
F 0 "#FLG04" H 4100 3945 30  0001 C CNN
F 1 "PWR_FLAG" H 4100 4030 30  0000 C CNN
F 2 "" H 4100 3850 60  0000 C CNN
F 3 "" H 4100 3850 60  0000 C CNN
	1    4100 3850
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR05
U 1 1 54E8E6DA
P 3800 4900
F 0 "#PWR05" H 3800 4650 60  0001 C CNN
F 1 "GNDREF" H 3800 4750 60  0000 C CNN
F 2 "" H 3800 4900 60  0000 C CNN
F 3 "" H 3800 4900 60  0000 C CNN
	1    3800 4900
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG06
U 1 1 54E8E6F6
P 4100 4900
F 0 "#FLG06" H 4100 4995 30  0001 C CNN
F 1 "PWR_FLAG" H 4100 5080 30  0000 C CNN
F 2 "" H 4100 4900 60  0000 C CNN
F 3 "" H 4100 4900 60  0000 C CNN
	1    4100 4900
	-1   0    0    1   
$EndComp
Wire Wire Line
	3650 4200 4150 4200
Wire Wire Line
	4150 4500 3650 4500
Wire Wire Line
	4150 4600 3650 4600
Wire Wire Line
	3650 4400 3800 4400
Wire Wire Line
	3800 4400 3800 4900
Wire Wire Line
	4100 4900 4100 4850
Wire Wire Line
	4100 4850 3800 4850
Connection ~ 3800 4850
Wire Wire Line
	3650 4300 4150 4300
Wire Wire Line
	3850 3850 3850 4100
Wire Wire Line
	3850 4100 3650 4100
Wire Wire Line
	4100 3850 4100 3950
Wire Wire Line
	4100 3950 3850 3950
Connection ~ 3850 3950
$Comp
L CONN_01X02 P?
U 1 1 57BCD0AF
P 8050 2650
F 0 "P?" H 8050 2800 50  0000 C CNN
F 1 "DEBUG TX/RX" V 8150 2650 50  0000 C CNN
F 2 "" H 8050 2650 50  0000 C CNN
F 3 "" H 8050 2650 50  0000 C CNN
	1    8050 2650
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X03 P?
U 1 1 57BCD0E6
P 3850 2700
F 0 "P?" H 3850 2900 50  0000 C CNN
F 1 "ISP" H 3850 2500 50  0000 C CNN
F 2 "" H 3850 1500 50  0000 C CNN
F 3 "" H 3850 1500 50  0000 C CNN
	1    3850 2700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 57BCD18D
P 4200 2500
F 0 "#PWR?" H 4200 2350 60  0001 C CNN
F 1 "+5V" H 4200 2640 60  0000 C CNN
F 2 "" H 4200 2500 60  0000 C CNN
F 3 "" H 4200 2500 60  0000 C CNN
	1    4200 2500
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR?
U 1 1 57BCD1A8
P 4200 2900
F 0 "#PWR?" H 4200 2650 60  0001 C CNN
F 1 "GNDREF" H 4200 2750 60  0000 C CNN
F 2 "" H 4200 2900 60  0000 C CNN
F 3 "" H 4200 2900 60  0000 C CNN
	1    4200 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 2800 4200 2800
Wire Wire Line
	4200 2800 4200 2900
Wire Wire Line
	4200 2500 4200 2600
Wire Wire Line
	4200 2600 4100 2600
Text HLabel 3200 2800 0    60   Output ~ 0
RESET
Wire Wire Line
	3600 2800 3200 2800
Text HLabel 3200 2700 0    60   Output ~ 0
SCK
Text HLabel 3200 2600 0    60   Input ~ 0
MISO
Wire Wire Line
	3200 2600 3600 2600
Wire Wire Line
	3200 2700 3600 2700
Text HLabel 4500 2700 2    60   Output ~ 0
MOSI
Wire Wire Line
	4500 2700 4100 2700
Text HLabel 7600 2600 0    60   Input ~ 0
TX
Wire Wire Line
	7600 2600 7850 2600
Text HLabel 7600 2700 0    60   Output ~ 0
RX
Wire Wire Line
	7600 2700 7850 2700
$EndSCHEMATC
