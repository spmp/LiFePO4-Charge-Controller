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
LIBS:special
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
LIBS:LiFePO4-Charge-Controller
LIBS:LiFePO4-Charge-Controller-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "25 jul 2014"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ARDUINO_MINI U?
U 1 1 53D206F8
P 5800 3350
F 0 "U?" H 6300 2400 70  0000 C CNN
F 1 "ARDUINO_MINI" H 6550 2300 70  0000 C CNN
F 2 "DIL20" H 5800 3300 60  0000 C CNN
F 3 "" H 5800 3350 60  0000 C CNN
	1    5800 3350
	1    0    0    -1  
$EndComp
$Comp
L SI8400 U?
U 1 1 53D22237
P 2950 5050
F 0 "U?" H 2950 4950 50  0000 C CNN
F 1 "SI8400" H 2950 5150 50  0000 C CNN
F 2 "MODULE" H 2950 5050 50  0001 C CNN
F 3 "DOCUMENTATION" H 2950 5050 50  0001 C CNN
	1    2950 5050
	1    0    0    -1  
$EndComp
$Comp
L SI8400 U?
U 1 1 53D22246
P 2950 3600
F 0 "U?" H 2950 3500 50  0000 C CNN
F 1 "SI8400" H 2950 3700 50  0000 C CNN
F 2 "MODULE" H 2950 3600 50  0001 C CNN
F 3 "DOCUMENTATION" H 2950 3600 50  0001 C CNN
	1    2950 3600
	1    0    0    -1  
$EndComp
$Comp
L SI8400 U?
U 1 1 53D22255
P 8000 4100
F 0 "U?" H 8000 4000 50  0000 C CNN
F 1 "SI8400" H 8000 4200 50  0000 C CNN
F 2 "MODULE" H 8000 4100 50  0001 C CNN
F 3 "DOCUMENTATION" H 8000 4100 50  0001 C CNN
	1    8000 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 3550 5100 3550
Wire Wire Line
	3800 3650 5100 3650
Wire Wire Line
	5800 2200 5800 1950
Wire Wire Line
	4500 1950 7350 1950
Wire Wire Line
	4500 1950 4500 5450
Wire Wire Line
	4500 4900 3800 4900
Wire Wire Line
	3800 3450 4500 3450
Connection ~ 4500 3450
Wire Wire Line
	4700 3550 4700 5000
Wire Wire Line
	4700 5000 3800 5000
Connection ~ 4700 3550
Wire Wire Line
	4800 3450 4800 5100
Wire Wire Line
	4800 5100 3800 5100
Connection ~ 4800 3650
Wire Wire Line
	5800 4900 5800 5600
$Comp
L R R?
U 1 1 53D22501
P 4650 3150
F 0 "R?" V 4730 3150 40  0000 C CNN
F 1 "R" V 4657 3151 40  0000 C CNN
F 2 "~" V 4580 3150 30  0000 C CNN
F 3 "~" H 4650 3150 30  0000 C CNN
	1    4650 3150
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 53D22537
P 4800 3200
F 0 "R?" V 4880 3200 40  0000 C CNN
F 1 "R" V 4807 3201 40  0000 C CNN
F 2 "~" V 4730 3200 30  0000 C CNN
F 3 "~" H 4800 3200 30  0000 C CNN
	1    4800 3200
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 53D2254D
P 1950 3050
F 0 "R?" V 2030 3050 40  0000 C CNN
F 1 "R" V 1957 3051 40  0000 C CNN
F 2 "~" V 1880 3050 30  0000 C CNN
F 3 "~" H 1950 3050 30  0000 C CNN
	1    1950 3050
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 53D2255C
P 1800 3150
F 0 "R?" V 1880 3150 40  0000 C CNN
F 1 "R" V 1807 3151 40  0000 C CNN
F 2 "~" V 1730 3150 30  0000 C CNN
F 3 "~" H 1800 3150 30  0000 C CNN
	1    1800 3150
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 53D22571
P 1900 4550
F 0 "R?" V 1980 4550 40  0000 C CNN
F 1 "R" V 1907 4551 40  0000 C CNN
F 2 "~" V 1830 4550 30  0000 C CNN
F 3 "~" H 1900 4550 30  0000 C CNN
	1    1900 4550
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 53D22580
P 1750 4600
F 0 "R?" V 1830 4600 40  0000 C CNN
F 1 "R" V 1757 4601 40  0000 C CNN
F 2 "~" V 1680 4600 30  0000 C CNN
F 3 "~" H 1750 4600 30  0000 C CNN
	1    1750 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 3400 4650 3550
Connection ~ 4650 3550
Wire Wire Line
	4800 2950 4800 2750
Wire Wire Line
	4800 2750 4500 2750
Connection ~ 4500 2750
Wire Wire Line
	4650 2900 4650 2750
Connection ~ 4650 2750
$Comp
L C C?
U 1 1 53D227F6
P 4250 3800
F 0 "C?" H 4250 3900 40  0000 L CNN
F 1 "C" H 4256 3715 40  0000 L CNN
F 2 "~" H 4288 3650 30  0000 C CNN
F 3 "~" H 4250 3800 60  0000 C CNN
	1    4250 3800
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 53D22805
P 4250 4000
F 0 "C?" H 4250 4100 40  0000 L CNN
F 1 "C" H 4256 3915 40  0000 L CNN
F 2 "~" H 4288 3850 30  0000 C CNN
F 3 "~" H 4250 4000 60  0000 C CNN
	1    4250 4000
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 53D22814
P 4200 5250
F 0 "C?" H 4200 5350 40  0000 L CNN
F 1 "C" H 4206 5165 40  0000 L CNN
F 2 "~" H 4238 5100 30  0000 C CNN
F 3 "~" H 4200 5250 60  0000 C CNN
	1    4200 5250
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 53D22823
P 4200 5450
F 0 "C?" H 4200 5550 40  0000 L CNN
F 1 "C" H 4206 5365 40  0000 L CNN
F 2 "~" H 4238 5300 30  0000 C CNN
F 3 "~" H 4200 5450 60  0000 C CNN
	1    4200 5450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4500 5450 4400 5450
Connection ~ 4500 4900
Wire Wire Line
	4400 5250 4500 5250
Connection ~ 4500 5250
Wire Wire Line
	4450 4000 4500 4000
Connection ~ 4500 4000
Wire Wire Line
	4450 3800 4500 3800
Connection ~ 4500 3800
Wire Wire Line
	3900 5600 7800 5600
Wire Wire Line
	3900 5600 3900 3750
Wire Wire Line
	3900 3750 3800 3750
Wire Wire Line
	4050 3800 3900 3800
Connection ~ 3900 3800
Wire Wire Line
	4050 4000 3900 4000
Connection ~ 3900 4000
Wire Wire Line
	3800 5200 3900 5200
Connection ~ 3900 5200
Wire Wire Line
	4000 5250 3900 5250
Connection ~ 3900 5250
Wire Wire Line
	4000 5450 3900 5450
Connection ~ 3900 5450
$Comp
L CONN_4 P?
U 1 1 53D229CF
P 950 5050
F 0 "P?" V 900 5050 50  0000 C CNN
F 1 "CONN_4" V 1000 5050 50  0000 C CNN
F 2 "" H 950 5050 60  0000 C CNN
F 3 "" H 950 5050 60  0000 C CNN
	1    950  5050
	-1   0    0    -1  
$EndComp
$Comp
L CONN_4 P?
U 1 1 53D22A94
P 950 3600
F 0 "P?" V 900 3600 50  0000 C CNN
F 1 "CONN_4" V 1000 3600 50  0000 C CNN
F 2 "" H 950 3600 60  0000 C CNN
F 3 "" H 950 3600 60  0000 C CNN
	1    950  3600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2100 5000 1300 5000
Wire Wire Line
	2100 5100 1300 5100
$Comp
L C C?
U 1 1 53D22BA4
P 1700 5350
F 0 "C?" H 1700 5450 40  0000 L CNN
F 1 "C" H 1706 5265 40  0000 L CNN
F 2 "~" H 1738 5200 30  0000 C CNN
F 3 "~" H 1700 5350 60  0000 C CNN
	1    1700 5350
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 53D22BAA
P 1700 5550
F 0 "C?" H 1700 5650 40  0000 L CNN
F 1 "C" H 1706 5465 40  0000 L CNN
F 2 "~" H 1738 5400 30  0000 C CNN
F 3 "~" H 1700 5550 60  0000 C CNN
	1    1700 5550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2050 5550 1900 5550
Wire Wire Line
	2050 5350 1900 5350
Wire Wire Line
	1500 5350 1400 5350
Wire Wire Line
	1400 5550 1500 5550
$Comp
L C C?
U 1 1 53D22BCD
P 1700 3900
F 0 "C?" H 1700 4000 40  0000 L CNN
F 1 "C" H 1706 3815 40  0000 L CNN
F 2 "~" H 1738 3750 30  0000 C CNN
F 3 "~" H 1700 3900 60  0000 C CNN
	1    1700 3900
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 53D22BD3
P 1700 4100
F 0 "C?" H 1700 4200 40  0000 L CNN
F 1 "C" H 1706 4015 40  0000 L CNN
F 2 "~" H 1738 3950 30  0000 C CNN
F 3 "~" H 1700 4100 60  0000 C CNN
	1    1700 4100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2050 4100 1900 4100
Wire Wire Line
	2050 3900 1900 3900
Wire Wire Line
	1500 3900 1400 3900
Wire Wire Line
	1400 4100 1500 4100
Wire Wire Line
	2100 5200 1300 5200
Wire Wire Line
	2100 4900 1300 4900
Wire Wire Line
	1750 4850 1750 5100
Connection ~ 1750 5100
Wire Wire Line
	1750 4350 1750 4300
Wire Wire Line
	1750 4300 2050 4300
Wire Wire Line
	2050 4300 2050 5550
Connection ~ 2050 4900
Connection ~ 1900 4300
Connection ~ 2050 5350
Wire Wire Line
	1400 5200 1400 5550
Connection ~ 1400 5350
Connection ~ 1400 5200
Wire Wire Line
	2100 3750 1300 3750
Wire Wire Line
	2100 3650 1300 3650
Wire Wire Line
	2100 3550 1300 3550
Wire Wire Line
	2100 3450 1300 3450
Wire Wire Line
	1900 5000 1900 4800
Connection ~ 1900 5000
Wire Wire Line
	1950 3550 1950 3300
Connection ~ 1950 3550
Wire Wire Line
	1800 3650 1800 3400
Connection ~ 1800 3650
Wire Wire Line
	1800 2900 1800 2800
Wire Wire Line
	1800 2800 2050 2800
Wire Wire Line
	2050 2800 2050 4100
Connection ~ 2050 3450
Connection ~ 2050 3900
Connection ~ 1950 2800
Wire Wire Line
	1400 3750 1400 4100
Connection ~ 1400 3750
Connection ~ 1400 3900
Wire Wire Line
	7150 5600 7150 4250
Connection ~ 5800 5600
Wire Wire Line
	7150 4150 6500 4150
Wire Wire Line
	6500 4050 7150 4050
Wire Wire Line
	7050 1950 7050 4500
Wire Wire Line
	7050 3950 7150 3950
Connection ~ 5800 1950
$Comp
L C C?
U 1 1 53D2365C
P 6850 4650
F 0 "C?" H 6850 4750 40  0000 L CNN
F 1 "C" H 6856 4565 40  0000 L CNN
F 2 "~" H 6888 4500 30  0000 C CNN
F 3 "~" H 6850 4650 60  0000 C CNN
	1    6850 4650
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 53D23662
P 6850 4850
F 0 "C?" H 6850 4950 40  0000 L CNN
F 1 "C" H 6856 4765 40  0000 L CNN
F 2 "~" H 6888 4700 30  0000 C CNN
F 3 "~" H 6850 4850 60  0000 C CNN
	1    6850 4850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7150 4850 7050 4850
Wire Wire Line
	7050 4650 7150 4650
Wire Wire Line
	6650 4650 6550 4650
Wire Wire Line
	6550 4850 6650 4850
Wire Wire Line
	7050 4500 6550 4500
Wire Wire Line
	6550 4500 6550 4850
Connection ~ 7050 3950
Connection ~ 6550 4650
Connection ~ 7150 4850
Connection ~ 7150 4650
$Comp
L R R?
U 1 1 53D23ADF
P 7600 2550
F 0 "R?" V 7680 2550 40  0000 C CNN
F 1 "R" V 7607 2551 40  0000 C CNN
F 2 "~" V 7530 2550 30  0000 C CNN
F 3 "~" H 7600 2550 30  0000 C CNN
	1    7600 2550
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 53D23AF8
P 7600 1950
F 0 "R?" V 7680 1950 40  0000 C CNN
F 1 "R" V 7607 1951 40  0000 C CNN
F 2 "~" V 7530 1950 30  0000 C CNN
F 3 "~" H 7600 1950 30  0000 C CNN
	1    7600 1950
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 53D23B07
P 8500 2550
F 0 "R?" V 8580 2550 40  0000 C CNN
F 1 "R" V 8507 2551 40  0000 C CNN
F 2 "~" V 8430 2550 30  0000 C CNN
F 3 "~" H 8500 2550 30  0000 C CNN
	1    8500 2550
	0    -1   -1   0   
$EndComp
$Comp
L NPN Q?
U 1 1 53D23B16
P 8100 2250
F 0 "Q?" H 8100 2100 50  0000 R CNN
F 1 "NPN" H 8100 2400 50  0000 R CNN
F 2 "~" H 8100 2250 60  0000 C CNN
F 3 "~" H 8100 2250 60  0000 C CNN
	1    8100 2250
	1    0    0    -1  
$EndComp
$Comp
L PNP Q?
U 1 1 53D23B25
P 8100 2850
F 0 "Q?" H 8100 2700 60  0000 R CNN
F 1 "PNP" H 8100 3000 60  0000 R CNN
F 2 "~" H 8100 2850 60  0000 C CNN
F 3 "~" H 8100 2850 60  0000 C CNN
	1    8100 2850
	1    0    0    1   
$EndComp
$Comp
L MOS_N Q?
U 1 1 53D23B43
P 9050 2550
F 0 "Q?" H 9060 2720 60  0000 R CNN
F 1 "MOS_N" H 9060 2400 60  0000 R CNN
F 2 "~" H 9050 2550 60  0000 C CNN
F 3 "~" H 9050 2550 60  0000 C CNN
	1    9050 2550
	1    0    0    -1  
$EndComp
Connection ~ 7050 1950
Wire Wire Line
	7850 1950 8200 1950
Wire Wire Line
	8200 1950 8200 2050
Wire Wire Line
	8200 2450 8200 2650
Connection ~ 8200 2550
Wire Wire Line
	8750 2550 8850 2550
Wire Wire Line
	7900 2250 7900 2850
Wire Wire Line
	7850 2550 7900 2550
Connection ~ 7900 2550
Wire Wire Line
	6500 3100 6700 3100
Wire Wire Line
	6700 3100 6700 2550
Wire Wire Line
	6700 2550 7350 2550
Wire Wire Line
	7800 5600 7800 3200
Wire Wire Line
	7800 3200 9150 3200
Wire Wire Line
	9150 3200 9150 2750
Connection ~ 7150 5600
Wire Wire Line
	8200 3050 8200 3200
Connection ~ 8200 3200
$EndSCHEMATC
