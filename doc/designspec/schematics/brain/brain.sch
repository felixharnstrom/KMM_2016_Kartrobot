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
LIBS:tsea29
LIBS:brain-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Kommunikations och kontrollenhet"
Date "2016-10-11"
Rev "0.1"
Comp "Grupp 1"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L USB_A P2
U 1 1 57FD3DF3
P 2950 4250
F 0 "P2" H 3150 4050 50  0000 C CNN
F 1 "USB_RPi_1" H 2900 4450 50  0000 C CNN
F 2 "" V 2900 4150 50  0000 C CNN
F 3 "" V 2900 4150 50  0000 C CNN
	1    2950 4250
	0    -1   -1   0   
$EndComp
$Comp
L USB_A P1
U 1 1 57FD409A
P 2950 3400
F 0 "P1" H 3150 3200 50  0000 C CNN
F 1 "USB_RPi_2" H 2900 3600 50  0000 C CNN
F 2 "" V 2900 3300 50  0000 C CNN
F 3 "" V 2900 3300 50  0000 C CNN
	1    2950 3400
	0    -1   -1   0   
$EndComp
NoConn ~ 2750 2950
NoConn ~ 2750 2850
NoConn ~ 2750 2750
NoConn ~ 2750 2650
NoConn ~ 2750 2550
NoConn ~ 2750 2450
NoConn ~ 2750 2350
NoConn ~ 2750 2250
NoConn ~ 2750 2150
NoConn ~ 2750 2050
NoConn ~ 2750 1950
NoConn ~ 2750 1850
NoConn ~ 2750 1750
NoConn ~ 2750 1650
NoConn ~ 2750 1550
NoConn ~ 2750 1450
NoConn ~ 2750 1350
NoConn ~ 2750 1250
NoConn ~ 2750 1150
NoConn ~ 3250 1050
NoConn ~ 3250 1150
NoConn ~ 3250 1350
NoConn ~ 3250 1450
NoConn ~ 3250 1550
NoConn ~ 3250 1650
NoConn ~ 3250 1750
NoConn ~ 3250 1850
NoConn ~ 3250 1950
NoConn ~ 3250 2050
NoConn ~ 3250 2150
NoConn ~ 3250 2250
NoConn ~ 3250 2350
NoConn ~ 3250 2450
NoConn ~ 3250 2550
NoConn ~ 3250 2650
$Comp
L +3.3V #PWR02
U 1 1 57FE3030
P 2600 1050
F 0 "#PWR02" H 2600 900 50  0001 C CNN
F 1 "+3.3V" H 2600 1190 50  0000 C CNN
F 2 "" H 2600 1050 50  0000 C CNN
F 3 "" H 2600 1050 50  0000 C CNN
	1    2600 1050
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR03
U 1 1 57FE3059
P 4550 2850
F 0 "#PWR03" H 4550 2700 50  0001 C CNN
F 1 "+3.3V" H 4550 2990 50  0000 C CNN
F 2 "" H 4550 2850 50  0000 C CNN
F 3 "" H 4550 2850 50  0000 C CNN
	1    4550 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 57FE3423
P 3450 1250
F 0 "#PWR04" H 3450 1000 50  0001 C CNN
F 1 "GND" H 3450 1100 50  0000 C CNN
F 2 "" H 3450 1250 50  0000 C CNN
F 3 "" H 3450 1250 50  0000 C CNN
	1    3450 1250
	1    0    0    -1  
$EndComp
$Comp
L RPi_3_GPIO P3
U 1 1 57FE32EC
P 3000 2000
F 0 "P3" H 3000 3050 50  0000 C CNN
F 1 "RPi_3_GPIO" V 3000 2000 50  0000 C CNN
F 2 "" H 3000 1050 50  0000 C CNN
F 3 "" H 3000 1050 50  0000 C CNN
	1    3000 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 2850 3250 2850
Wire Wire Line
	2750 1050 2600 1050
Wire Wire Line
	3250 1250 3450 1250
Wire Wire Line
	3350 2700 3350 2850
Wire Wire Line
	3250 2950 3350 2950
Wire Wire Line
	3350 2950 3350 3000
$Comp
L USBUART U?
U 1 1 58013042
P 3800 3450
F 0 "U?" H 3450 3800 60  0000 C CNN
F 1 "USBUART" H 3800 3450 60  0000 C CNN
F 2 "" H 3800 3450 60  0001 C CNN
F 3 "" H 3800 3450 60  0001 C CNN
	1    3800 3450
	-1   0    0    1   
$EndComp
$Comp
L USBUART U?
U 1 1 58013123
P 3800 4300
F 0 "U?" H 3450 4650 60  0000 C CNN
F 1 "USBUART" H 3800 4300 60  0000 C CNN
F 2 "" H 3800 4300 60  0001 C CNN
F 3 "" H 3800 4300 60  0001 C CNN
	1    3800 4300
	-1   0    0    1   
$EndComp
$Comp
L SW_PUSH SW?
U 1 1 5804F9B6
P 3850 2700
F 0 "SW?" H 4000 2810 50  0000 C CNN
F 1 "SW_PUSH" H 3850 2620 50  0000 C CNN
F 2 "" H 3850 2700 50  0000 C CNN
F 3 "" H 3850 2700 50  0000 C CNN
	1    3850 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 2700 3350 2700
NoConn ~ 3250 2750
Wire Wire Line
	4550 2850 4350 2850
Connection ~ 4350 2850
Wire Wire Line
	4150 2700 4350 2700
Wire Wire Line
	4350 2700 4350 3000
$Comp
L SW_PUSH SW?
U 1 1 5804FFC9
P 3850 3000
F 0 "SW?" H 4000 3110 50  0000 C CNN
F 1 "SW_PUSH" H 3850 2920 50  0000 C CNN
F 2 "" H 3850 3000 50  0000 C CNN
F 3 "" H 3850 3000 50  0000 C CNN
	1    3850 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 3000 4150 3000
Wire Wire Line
	3350 3000 3550 3000
$EndSCHEMATC
