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
Date "26 jul 2014"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ARDUINO_MINI U4
U 1 1 53D206F8
P 5800 3350
F 0 "U4" H 6300 2400 70  0000 C CNN
F 1 "ARDUINO_MINI" H 6550 2300 70  0000 C CNN
F 2 "DIL20" H 5800 3300 60  0000 C CNN
F 3 "" H 5800 3350 60  0000 C CNN
	1    5800 3350
	1    0    0    -1  
$EndComp
$Comp
L SI8400 U3
U 1 1 53D22237
P 2950 5050
F 0 "U3" H 2950 4950 50  0000 C CNN
F 1 "SI8400" H 2950 5150 50  0000 C CNN
F 2 "MODULE" H 2950 5050 50  0001 C CNN
F 3 "DOCUMENTATION" H 2950 5050 50  0001 C CNN
	1    2950 5050
	1    0    0    -1  
$EndComp
$Comp
L SI8400 U2
U 1 1 53D22246
P 2950 3600
F 0 "U2" H 2950 3500 50  0000 C CNN
F 1 "SI8400" H 2950 3700 50  0000 C CNN
F 2 "MODULE" H 2950 3600 50  0001 C CNN
F 3 "DOCUMENTATION" H 2950 3600 50  0001 C CNN
	1    2950 3600
	1    0    0    -1  
$EndComp
$Comp
L SI8400 U5
U 1 1 53D22255
P 8000 4100
F 0 "U5" H 8000 4000 50  0000 C CNN
F 1 "SI8400" H 8000 4200 50  0000 C CNN
F 2 "MODULE" H 8000 4100 50  0001 C CNN
F 3 "DOCUMENTATION" H 8000 4100 50  0001 C CNN
	1    8000 4100
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 53D22501
P 4650 3150
F 0 "R5" V 4730 3150 40  0000 C CNN
F 1 "R" V 4657 3151 40  0000 C CNN
F 2 "~" V 4580 3150 30  0000 C CNN
F 3 "~" H 4650 3150 30  0000 C CNN
	1    4650 3150
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 53D22537
P 4800 3200
F 0 "R6" V 4880 3200 40  0000 C CNN
F 1 "R" V 4807 3201 40  0000 C CNN
F 2 "~" V 4730 3200 30  0000 C CNN
F 3 "~" H 4800 3200 30  0000 C CNN
	1    4800 3200
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 53D2254D
P 1950 3050
F 0 "R4" V 2030 3050 40  0000 C CNN
F 1 "R" V 1957 3051 40  0000 C CNN
F 2 "~" V 1880 3050 30  0000 C CNN
F 3 "~" H 1950 3050 30  0000 C CNN
	1    1950 3050
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 53D2255C
P 1800 3150
F 0 "R2" V 1880 3150 40  0000 C CNN
F 1 "R" V 1807 3151 40  0000 C CNN
F 2 "~" V 1730 3150 30  0000 C CNN
F 3 "~" H 1800 3150 30  0000 C CNN
	1    1800 3150
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 53D22571
P 1900 4550
F 0 "R3" V 1980 4550 40  0000 C CNN
F 1 "R" V 1907 4551 40  0000 C CNN
F 2 "~" V 1830 4550 30  0000 C CNN
F 3 "~" H 1900 4550 30  0000 C CNN
	1    1900 4550
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 53D22580
P 1750 4600
F 0 "R1" V 1830 4600 40  0000 C CNN
F 1 "R" V 1757 4601 40  0000 C CNN
F 2 "~" V 1680 4600 30  0000 C CNN
F 3 "~" H 1750 4600 30  0000 C CNN
	1    1750 4600
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 53D227F6
P 4250 3800
F 0 "C8" H 4250 3900 40  0000 L CNN
F 1 "C" H 4256 3715 40  0000 L CNN
F 2 "~" H 4288 3650 30  0000 C CNN
F 3 "~" H 4250 3800 60  0000 C CNN
	1    4250 3800
	0    1    1    0   
$EndComp
$Comp
L C C9
U 1 1 53D22805
P 4250 4000
F 0 "C9" H 4250 4100 40  0000 L CNN
F 1 "C" H 4256 3915 40  0000 L CNN
F 2 "~" H 4288 3850 30  0000 C CNN
F 3 "~" H 4250 4000 60  0000 C CNN
	1    4250 4000
	0    -1   -1   0   
$EndComp
$Comp
L C C6
U 1 1 53D22814
P 4200 5250
F 0 "C6" H 4200 5350 40  0000 L CNN
F 1 "C" H 4206 5165 40  0000 L CNN
F 2 "~" H 4238 5100 30  0000 C CNN
F 3 "~" H 4200 5250 60  0000 C CNN
	1    4200 5250
	0    -1   -1   0   
$EndComp
$Comp
L C C7
U 1 1 53D22823
P 4200 5450
F 0 "C7" H 4200 5550 40  0000 L CNN
F 1 "C" H 4206 5365 40  0000 L CNN
F 2 "~" H 4238 5300 30  0000 C CNN
F 3 "~" H 4200 5450 60  0000 C CNN
	1    4200 5450
	0    -1   -1   0   
$EndComp
$Comp
L CONN_4 P3
U 1 1 53D229CF
P 950 5050
F 0 "P3" V 900 5050 50  0000 C CNN
F 1 "CONN_4" V 1000 5050 50  0000 C CNN
F 2 "" H 950 5050 60  0000 C CNN
F 3 "" H 950 5050 60  0000 C CNN
	1    950  5050
	-1   0    0    -1  
$EndComp
$Comp
L CONN_4 P2
U 1 1 53D22A94
P 950 3600
F 0 "P2" V 900 3600 50  0000 C CNN
F 1 "CONN_4" V 1000 3600 50  0000 C CNN
F 2 "" H 950 3600 60  0000 C CNN
F 3 "" H 950 3600 60  0000 C CNN
	1    950  3600
	-1   0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 53D22BA4
P 1700 5350
F 0 "C3" H 1700 5450 40  0000 L CNN
F 1 "C" H 1706 5265 40  0000 L CNN
F 2 "~" H 1738 5200 30  0000 C CNN
F 3 "~" H 1700 5350 60  0000 C CNN
	1    1700 5350
	0    -1   -1   0   
$EndComp
$Comp
L C C4
U 1 1 53D22BAA
P 1700 5550
F 0 "C4" H 1700 5650 40  0000 L CNN
F 1 "C" H 1706 5465 40  0000 L CNN
F 2 "~" H 1738 5400 30  0000 C CNN
F 3 "~" H 1700 5550 60  0000 C CNN
	1    1700 5550
	0    -1   -1   0   
$EndComp
$Comp
L C C1
U 1 1 53D22BCD
P 1700 3900
F 0 "C1" H 1700 4000 40  0000 L CNN
F 1 "C" H 1706 3815 40  0000 L CNN
F 2 "~" H 1738 3750 30  0000 C CNN
F 3 "~" H 1700 3900 60  0000 C CNN
	1    1700 3900
	0    -1   -1   0   
$EndComp
$Comp
L C C2
U 1 1 53D22BD3
P 1700 4100
F 0 "C2" H 1700 4200 40  0000 L CNN
F 1 "C" H 1706 4015 40  0000 L CNN
F 2 "~" H 1738 3950 30  0000 C CNN
F 3 "~" H 1700 4100 60  0000 C CNN
	1    1700 4100
	0    -1   -1   0   
$EndComp
$Comp
L C C10
U 1 1 53D2365C
P 6850 4650
F 0 "C10" H 6850 4750 40  0000 L CNN
F 1 "C" H 6856 4565 40  0000 L CNN
F 2 "~" H 6888 4500 30  0000 C CNN
F 3 "~" H 6850 4650 60  0000 C CNN
	1    6850 4650
	0    -1   -1   0   
$EndComp
$Comp
L C C11
U 1 1 53D23662
P 6850 4850
F 0 "C11" H 6850 4950 40  0000 L CNN
F 1 "C" H 6856 4765 40  0000 L CNN
F 2 "~" H 6888 4700 30  0000 C CNN
F 3 "~" H 6850 4850 60  0000 C CNN
	1    6850 4850
	0    -1   -1   0   
$EndComp
$Comp
L CONN_4 P5
U 1 1 53D2498F
P 9450 4100
F 0 "P5" V 9400 4100 50  0000 C CNN
F 1 "CONN_4" V 9500 4100 50  0000 C CNN
F 2 "" H 9450 4100 60  0000 C CNN
F 3 "" H 9450 4100 60  0000 C CNN
	1    9450 4100
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 53D24B44
P 7750 1750
F 0 "R8" V 7830 1750 40  0000 C CNN
F 1 "R" V 7757 1751 40  0000 C CNN
F 2 "~" V 7680 1750 30  0000 C CNN
F 3 "~" H 7750 1750 30  0000 C CNN
	1    7750 1750
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 53D24B53
P 7950 1750
F 0 "R10" V 8030 1750 40  0000 C CNN
F 1 "R" V 7957 1751 40  0000 C CNN
F 2 "~" V 7880 1750 30  0000 C CNN
F 3 "~" H 7950 1750 30  0000 C CNN
	1    7950 1750
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 53D24B62
P 7750 2450
F 0 "R9" V 7830 2450 40  0000 C CNN
F 1 "R" V 7757 2451 40  0000 C CNN
F 2 "~" V 7680 2450 30  0000 C CNN
F 3 "~" H 7750 2450 30  0000 C CNN
	1    7750 2450
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 53D24B71
P 7950 2450
F 0 "R11" V 8030 2450 40  0000 C CNN
F 1 "R" V 7957 2451 40  0000 C CNN
F 2 "~" V 7880 2450 30  0000 C CNN
F 3 "~" H 7950 2450 30  0000 C CNN
	1    7950 2450
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 53D24B80
P 7400 3100
F 0 "R7" V 7480 3100 40  0000 C CNN
F 1 "R" V 7407 3101 40  0000 C CNN
F 2 "~" V 7330 3100 30  0000 C CNN
F 3 "~" H 7400 3100 30  0000 C CNN
	1    7400 3100
	0    -1   -1   0   
$EndComp
$Comp
L C C12
U 1 1 53D24B8F
P 7750 3350
F 0 "C12" H 7750 3450 40  0000 L CNN
F 1 "C" H 7756 3265 40  0000 L CNN
F 2 "~" H 7788 3200 30  0000 C CNN
F 3 "~" H 7750 3350 60  0000 C CNN
	1    7750 3350
	1    0    0    -1  
$EndComp
$Comp
L CONN_4 P4
U 1 1 53D258C0
P 8700 3050
F 0 "P4" V 8650 3050 50  0000 C CNN
F 1 "CONN_4" V 8750 3050 50  0000 C CNN
F 2 "" H 8700 3050 60  0000 C CNN
F 3 "" H 8700 3050 60  0000 C CNN
	1    8700 3050
	1    0    0    -1  
$EndComp
$Comp
L AD8236 U1
U 1 1 53D25D9A
P 2850 1700
F 0 "U1" H 2850 1600 50  0000 C CNN
F 1 "AD8236" H 2850 1800 50  0000 C CNN
F 2 "MODULE" H 2850 1700 50  0001 C CNN
F 3 "DOCUMENTATION" H 2850 1700 50  0001 C CNN
	1    2850 1700
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P1
U 1 1 53D25DA9
P 950 1700
F 0 "P1" V 900 1700 40  0000 C CNN
F 1 "CONN_2" V 1000 1700 40  0000 C CNN
F 2 "" H 950 1700 60  0000 C CNN
F 3 "" H 950 1700 60  0000 C CNN
	1    950  1700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3800 3550 5100 3550
Wire Wire Line
	3800 3650 5100 3650
Wire Wire Line
	5800 1550 5800 2200
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
	3900 5600 7150 5600
Wire Wire Line
	3900 1850 3900 5600
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
Wire Wire Line
	2100 5000 1300 5000
Wire Wire Line
	2100 5100 1300 5100
Wire Wire Line
	2050 5550 1900 5550
Wire Wire Line
	2050 5350 1900 5350
Wire Wire Line
	1500 5350 1400 5350
Wire Wire Line
	1400 5550 1500 5550
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
Wire Wire Line
	7150 4850 7050 4850
Wire Wire Line
	7050 4650 7750 4650
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
Connection ~ 7050 1950
Wire Wire Line
	9100 3950 8850 3950
Wire Wire Line
	8850 4050 9100 4050
Wire Wire Line
	9100 4150 8850 4150
Wire Wire Line
	8850 4250 9100 4250
Wire Wire Line
	7750 4650 7750 3550
Wire Wire Line
	5100 3150 5000 3150
Wire Wire Line
	5000 3150 5000 2100
Wire Wire Line
	5000 2100 8150 2100
Wire Wire Line
	7950 2000 7950 2200
Connection ~ 7950 2100
Wire Wire Line
	7750 2000 7750 2200
Connection ~ 7750 2100
Wire Wire Line
	6500 3100 7150 3100
Wire Wire Line
	7650 3100 8350 3100
Wire Wire Line
	7750 3100 7750 3150
Wire Wire Line
	7750 3550 8350 3550
Wire Wire Line
	7900 3550 7900 2700
Wire Wire Line
	7750 2700 7950 2700
Connection ~ 7900 2700
Connection ~ 7750 3100
Wire Wire Line
	8350 3550 8350 3200
Connection ~ 7900 3550
Wire Wire Line
	4500 1950 7050 1950
Wire Wire Line
	5650 2200 5650 1750
Wire Wire Line
	5650 1750 7250 1750
Wire Wire Line
	7250 1750 7250 3000
Wire Wire Line
	7250 3000 8350 3000
Wire Wire Line
	8350 1500 8350 2900
Wire Wire Line
	7750 1500 8350 1500
Connection ~ 7950 1500
$Comp
L POT RV1
U 1 1 53D25EC5
P 1700 1650
F 0 "RV1" H 1700 1550 50  0000 C CNN
F 1 "POT" H 1700 1650 50  0000 C CNN
F 2 "~" H 1700 1650 60  0000 C CNN
F 3 "~" H 1700 1650 60  0000 C CNN
	1    1700 1650
	0    1    1    0   
$EndComp
Wire Wire Line
	1700 1900 1850 1900
Wire Wire Line
	1850 1900 1850 1750
Wire Wire Line
	1850 1750 2100 1750
Wire Wire Line
	1700 1400 1850 1400
Wire Wire Line
	1850 1400 1850 1650
Wire Wire Line
	1850 1650 2100 1650
Wire Wire Line
	2100 1550 1950 1550
Wire Wire Line
	1950 1550 1950 1350
Wire Wire Line
	1950 1350 1500 1350
Wire Wire Line
	1500 1350 1500 1600
Wire Wire Line
	1500 1800 1500 1950
Wire Wire Line
	1500 1950 1950 1950
Wire Wire Line
	1950 1950 1950 1850
Wire Wire Line
	1950 1850 2100 1850
Wire Wire Line
	3600 1750 3700 1750
Wire Wire Line
	3700 1750 3700 1850
Connection ~ 3700 1850
Wire Wire Line
	3600 1650 4150 1650
Wire Wire Line
	4150 1650 4150 3250
Wire Wire Line
	4150 3250 5100 3250
Wire Wire Line
	3600 1550 5800 1550
Wire Wire Line
	3900 1850 3600 1850
Connection ~ 3900 3750
$Comp
L C C5
U 1 1 53D2659C
P 4000 1750
F 0 "C5" H 4000 1850 40  0000 L CNN
F 1 "C" H 4006 1665 40  0000 L CNN
F 2 "~" H 4038 1600 30  0000 C CNN
F 3 "~" H 4000 1750 60  0000 C CNN
	1    4000 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 1950 3900 1950
Connection ~ 3900 1950
Wire Wire Line
	1300 1800 1500 1800
Wire Wire Line
	1500 1600 1300 1600
$Comp
L C C13
U 1 1 53D38BF3
P 8150 2450
F 0 "C13" H 8150 2550 40  0000 L CNN
F 1 "C" H 8156 2365 40  0000 L CNN
F 2 "~" H 8188 2300 30  0000 C CNN
F 3 "~" H 8150 2450 60  0000 C CNN
	1    8150 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 2100 8150 2250
Wire Wire Line
	8150 2650 8150 3550
Connection ~ 8150 3550
$EndSCHEMATC
