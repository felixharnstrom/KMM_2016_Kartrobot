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
LIBS:styrenhet-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Label 4750 1650 1    60   ~ 0
LP-Filter
Wire Wire Line
	5750 3200 6100 3200
Wire Wire Line
	5750 3300 6100 3300
Wire Wire Line
	5750 3400 6100 3400
Wire Wire Line
	5750 3500 6100 3500
Wire Wire Line
	4750 1150 4750 1700
Wire Wire Line
	4550 5700 4750 5700
Wire Wire Line
	4750 5700 4750 5850
Connection ~ 4750 5700
$Comp
L ATMEGA1284P-P IC1
U 1 1 57F77162
P 4750 3700
F 0 "IC1" H 3900 5580 50  0000 L BNN
F 1 "ATMEGA1284P-P" H 5150 1750 50  0000 L BNN
F 2 "DIL40" H 4750 3700 50  0000 C CIN
F 3 "https://docs.isy.liu.se/pub/VanHeden/DataSheets/atmega1284p.pdf" H 4750 3700 50  0001 C CNN
	1    4750 3700
	1    0    0    -1  
$EndComp
$Comp
L IQEXO-3 U1
U 1 1 57F79B30
P 2200 2150
F 0 "U1" H 2000 2450 60  0000 C CNN
F 1 "IQEXO-3" V 2200 2150 60  0000 C CNN
F 2 "" H 2300 2150 60  0001 C CNN
F 3 "" H 2300 2150 60  0001 C CNN
	1    2200 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 2100 1450 2100
Wire Wire Line
	1450 2100 1450 2800
Wire Wire Line
	2800 2300 2650 2300
Wire Wire Line
	2800 2100 2800 2200
Wire Wire Line
	2800 2200 2800 2300
Wire Wire Line
	2800 2300 2800 2450
Wire Wire Line
	2650 2200 2800 2200
Wire Wire Line
	2800 2200 3000 2200
Connection ~ 2800 2200
Wire Wire Line
	2650 2100 2800 2100
$Comp
L GND #PWR01
U 1 1 57F79CE6
P 3000 2200
F 0 "#PWR01" H 3000 1950 50  0001 C CNN
F 1 "GND" H 3000 2050 50  0000 C CNN
F 2 "" H 3000 2200 50  0000 C CNN
F 3 "" H 3000 2200 50  0000 C CNN
	1    3000 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2000 3000 2000
$Comp
L +5V #PWR02
U 1 1 57F79D1F
P 3000 2000
F 0 "#PWR02" H 3000 1850 50  0001 C CNN
F 1 "+5V" H 3000 2140 50  0000 C CNN
F 2 "" H 3000 2000 50  0000 C CNN
F 3 "" H 3000 2000 50  0000 C CNN
	1    3000 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 2000 1500 2000
Text Label 1500 2000 0    60   ~ 0
16MHz
Text Label 3400 2800 0    60   ~ 0
8MHz
Wire Wire Line
	1450 2800 3750 2800
Wire Wire Line
	1750 2300 1750 2450
Wire Wire Line
	1750 2450 2800 2450
Connection ~ 2800 2300
$Comp
L +5V #PWR03
U 1 1 57F79EC3
P 1550 2300
F 0 "#PWR03" H 1550 2150 50  0001 C CNN
F 1 "+5V" H 1550 2440 50  0000 C CNN
F 2 "" H 1550 2300 50  0000 C CNN
F 3 "" H 1550 2300 50  0000 C CNN
	1    1550 2300
	1    0    0    -1  
$EndComp
Text GLabel 3750 2000 0    60   Input ~ 0
RESET
NoConn ~ 5750 2000
NoConn ~ 5750 2100
NoConn ~ 5750 2200
$Comp
L +5V #PWR04
U 1 1 57F7A832
P 6100 3600
F 0 "#PWR04" H 6100 3450 50  0001 C CNN
F 1 "+5V" H 6100 3740 50  0000 C CNN
F 2 "" H 6100 3600 50  0000 C CNN
F 3 "" H 6100 3600 50  0000 C CNN
	1    6100 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 57F7A84E
P 6100 3700
F 0 "#PWR05" H 6100 3450 50  0001 C CNN
F 1 "GND" H 6100 3550 50  0000 C CNN
F 2 "" H 6100 3700 50  0000 C CNN
F 3 "" H 6100 3700 50  0000 C CNN
	1    6100 3700
	1    0    0    -1  
$EndComp
NoConn ~ 1750 2000
NoConn ~ 5750 2300
NoConn ~ 5750 2400
NoConn ~ 5750 2500
NoConn ~ 5750 2600
NoConn ~ 5750 2700
NoConn ~ 5750 2900
NoConn ~ 5750 3000
NoConn ~ 5750 3100
NoConn ~ 5750 3600
NoConn ~ 5750 3800
NoConn ~ 5750 3900
NoConn ~ 5750 4000
NoConn ~ 5750 4100
NoConn ~ 5750 4200
NoConn ~ 5750 4300
NoConn ~ 5750 4400
NoConn ~ 5750 4500
NoConn ~ 5750 4900
NoConn ~ 5750 5000
NoConn ~ 5750 5100
NoConn ~ 5750 5200
NoConn ~ 5750 5300
NoConn ~ 5750 5400
$Comp
L GND #PWR06
U 1 1 57F7A9D0
P 4750 5850
F 0 "#PWR06" H 4750 5600 50  0001 C CNN
F 1 "GND" H 4750 5700 50  0000 C CNN
F 2 "" H 4750 5850 50  0000 C CNN
F 3 "" H 4750 5850 50  0000 C CNN
	1    4750 5850
	1    0    0    -1  
$EndComp
NoConn ~ 3750 2400
NoConn ~ 3750 3200
$Comp
L +5V #PWR07
U 1 1 57F7ABE0
P 4750 1150
F 0 "#PWR07" H 4750 1000 50  0001 C CNN
F 1 "+5V" H 4750 1290 50  0000 C CNN
F 2 "" H 4750 1150 50  0000 C CNN
F 3 "" H 4750 1150 50  0000 C CNN
	1    4750 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 57F7AC05
P 4300 1600
F 0 "#PWR08" H 4300 1350 50  0001 C CNN
F 1 "GND" H 4300 1450 50  0000 C CNN
F 2 "" H 4300 1600 50  0000 C CNN
F 3 "" H 4300 1600 50  0000 C CNN
	1    4300 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1600 4550 1600
Wire Wire Line
	4550 1600 4550 1700
Wire Wire Line
	1750 2200 1650 2200
Wire Wire Line
	1650 2200 1650 2350
Wire Wire Line
	1650 2350 1550 2350
Wire Wire Line
	1550 2350 1550 2300
$Comp
L USB-UART U3
U 1 1 57F7998E
P 6350 4750
F 0 "U3" H 6000 4900 60  0000 C CNN
F 1 "USB-UART" H 6350 4750 60  0000 C CNN
F 2 "" H 6350 4750 60  0001 C CNN
F 3 "" H 6350 4750 60  0001 C CNN
	1    6350 4750
	1    0    0    -1  
$EndComp
$Comp
L Terminator-CTRL U2
U 1 1 57F799AB
P 6900 3400
F 0 "U2" H 6350 3800 60  0000 C CNN
F 1 "Terminator-CTRL" V 6900 3350 60  0000 C CNN
F 2 "" H 6900 3400 60  0001 C CNN
F 3 "" H 6900 3400 60  0001 C CNN
	1    6900 3400
	1    0    0    -1  
$EndComp
Text Label 6000 5150 0    60   ~ 0
TODO:SERVO
Wire Wire Line
	5750 3600 5900 3600
Wire Wire Line
	5900 3600 5900 4150
Wire Wire Line
	5900 4150 6300 4150
$Comp
L Servo U?
U 1 1 57FFAFE2
P 6850 4250
F 0 "U?" H 6550 4500 60  0000 C CNN
F 1 "Servo" V 6850 4250 60  0000 C CNN
F 2 "" H 6850 4250 60  0001 C CNN
F 3 "" H 6850 4250 60  0001 C CNN
	1    6850 4250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 57FFB033
P 6300 4250
F 0 "#PWR?" H 6300 4100 50  0001 C CNN
F 1 "+5V" H 6300 4390 50  0000 C CNN
F 2 "" H 6300 4250 50  0000 C CNN
F 3 "" H 6300 4250 50  0000 C CNN
	1    6300 4250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 57FFB05B
P 6300 4350
F 0 "#PWR?" H 6300 4100 50  0001 C CNN
F 1 "GND" H 6300 4200 50  0000 C CNN
F 2 "" H 6300 4350 50  0000 C CNN
F 3 "" H 6300 4350 50  0000 C CNN
	1    6300 4350
	1    0    0    -1  
$EndComp
$EndSCHEMATC
