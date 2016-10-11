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
L USB_A P?
U 1 1 57FD3DF3
P 2950 4250
F 0 "P?" H 3150 4050 50  0000 C CNN
F 1 "USB_RPi_1" H 2900 4450 50  0000 C CNN
F 2 "" V 2900 4150 50  0000 C CNN
F 3 "" V 2900 4150 50  0000 C CNN
	1    2950 4250
	0    -1   -1   0   
$EndComp
$Comp
L USB_A P?
U 1 1 57FD409A
P 2950 3400
F 0 "P?" H 3150 3200 50  0000 C CNN
F 1 "USB_RPi_2" H 2900 3600 50  0000 C CNN
F 2 "" V 2900 3300 50  0000 C CNN
F 3 "" V 2900 3300 50  0000 C CNN
	1    2950 3400
	0    -1   -1   0   
$EndComp
$Comp
L CH340G U?
U 1 1 57FD4430
P 3800 4300
F 0 "U?" H 3450 4650 60  0000 C CNN
F 1 "CH340G" H 3800 4300 60  0000 C CNN
F 2 "" H 3800 4300 60  0001 C CNN
F 3 "TODO: Choose component" H 3800 4300 60  0001 C CNN
	1    3800 4300
	-1   0    0    1   
$EndComp
$Comp
L CH340G U?
U 1 1 57FD4518
P 3800 3450
F 0 "U?" H 3450 3800 60  0000 C CNN
F 1 "CH340G" H 3800 3450 60  0000 C CNN
F 2 "" H 3800 3450 60  0001 C CNN
F 3 "TODO: Choose component" H 3800 3450 60  0001 C CNN
	1    3800 3450
	-1   0    0    1   
$EndComp
$Comp
L BTN U?
U 1 1 57FD4666
P 3900 2800
F 0 "U?" H 4050 2550 60  0000 C CNN
F 1 "BTN x2" H 3800 2800 60  0000 C CNN
F 2 "" H 3900 2800 60  0001 C CNN
F 3 "" H 3900 2800 60  0001 C CNN
	1    3900 2800
	-1   0    0    1   
$EndComp
NoConn ~ 4400 4350
NoConn ~ 4400 3500
Wire Wire Line
	4400 4550 4400 4450
Wire Wire Line
	4400 3700 4400 3600
Wire Wire Line
	3350 2650 3500 2650
Wire Wire Line
	3350 2650 3350 2850
Wire Wire Line
	3350 2850 3250 2850
Wire Wire Line
	3500 2850 3450 2850
Wire Wire Line
	3450 2850 3450 2950
Wire Wire Line
	3450 2950 3250 2950
NoConn ~ 3500 2750
NoConn ~ 3500 2950
$Comp
L +5V #PWR?
U 1 1 57FD4995
P 4400 2650
F 0 "#PWR?" H 4400 2500 50  0001 C CNN
F 1 "+5V" H 4400 2790 50  0000 C CNN
F 2 "" H 4400 2650 50  0000 C CNN
F 3 "" H 4400 2650 50  0000 C CNN
	1    4400 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 57FD49B5
P 4400 2950
F 0 "#PWR?" H 4400 2700 50  0001 C CNN
F 1 "GND" H 4400 2800 50  0000 C CNN
F 2 "" H 4400 2950 50  0000 C CNN
F 3 "" H 4400 2950 50  0000 C CNN
	1    4400 2950
	1    0    0    -1  
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
NoConn ~ 2750 1050
NoConn ~ 3250 1050
NoConn ~ 3250 1150
NoConn ~ 3250 1250
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
NoConn ~ 3250 2750
$Comp
L Raspberry_pi_3_gpio P?
U 1 1 57FD5699
P 3000 2000
F 0 "P?" H 3000 3050 50  0000 C CNN
F 1 "Raspberry_pi_3_gpio" V 3000 2000 50  0000 C CNN
F 2 "" H 3000 1050 50  0000 C CNN
F 3 "" H 3000 1050 50  0000 C CNN
	1    3000 2000
	1    0    0    -1  
$EndComp
$EndSCHEMATC
