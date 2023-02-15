EESchema Schematic File Version 4
EELAYER 30 0
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
$Comp
L Isolator:ADuM1410 U?
U 1 1 619F49CB
P 4800 2150
F 0 "U?" H 4800 2917 50  0000 C CNN
F 1 "ADuM1410" H 4800 2826 50  0000 C CNN
F 2 "Package_SO:SOIC-16W_7.5x10.3mm_P1.27mm" H 4800 1450 50  0001 C CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/ADUM1410_1411_1412.pdf" H 4000 2150 50  0001 C CNN
	1    4800 2150
	1    0    0    -1  
$EndComp
$Comp
L Interface_UART:MAX485E U?
U 1 1 61A02136
P 6900 1950
F 0 "U?" H 6900 2631 50  0000 C CNN
F 1 "MAX485E" H 6900 2540 50  0000 C CNN
F 2 "" H 6900 1250 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX1487E-MAX491E.pdf" H 6900 2000 50  0001 C CNN
	1    6900 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61A04A03
P 6900 3100
F 0 "#PWR?" H 6900 2850 50  0001 C CNN
F 1 "GND" H 6905 2927 50  0000 C CNN
F 2 "" H 6900 3100 50  0001 C CNN
F 3 "" H 6900 3100 50  0001 C CNN
	1    6900 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 2550 6900 3100
Wire Wire Line
	5400 1850 6500 1850
Wire Wire Line
	5400 1950 5950 1950
Wire Wire Line
	5950 1950 5950 2150
Wire Wire Line
	5950 2150 6500 2150
$Comp
L power:GND #PWR?
U 1 1 61A0FEC3
P 5550 3050
F 0 "#PWR?" H 5550 2800 50  0001 C CNN
F 1 "GND" H 5555 2877 50  0000 C CNN
F 2 "" H 5550 3050 50  0001 C CNN
F 3 "" H 5550 3050 50  0001 C CNN
	1    5550 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 2550 5550 2550
Wire Wire Line
	5550 2550 5550 2650
Wire Wire Line
	5400 2650 5550 2650
Connection ~ 5550 2650
Wire Wire Line
	5550 2650 5550 3050
$Comp
L power:GND #PWR?
U 1 1 61A10C99
P 4000 3050
F 0 "#PWR?" H 4000 2800 50  0001 C CNN
F 1 "GND" H 4005 2877 50  0000 C CNN
F 2 "" H 4000 3050 50  0001 C CNN
F 3 "" H 4000 3050 50  0001 C CNN
	1    4000 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 2550 4000 2550
Wire Wire Line
	4000 2550 4000 2650
Wire Wire Line
	4200 2650 4000 2650
Connection ~ 4000 2650
Wire Wire Line
	4000 2650 4000 3050
$Comp
L MCU_Microchip_ATmega:ATmega2560-16AU U?
U 1 1 61A13030
P 2050 4350
F 0 "U?" H 2050 1361 50  0000 C CNN
F 1 "ATmega2560-16AU" H 2050 1270 50  0000 C CNN
F 2 "Package_QFP:TQFP-100_14x14mm_P0.5mm" H 2050 4350 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf" H 2050 4350 50  0001 C CNN
	1    2050 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61A26685
P 2600 7400
F 0 "#PWR?" H 2600 7150 50  0001 C CNN
F 1 "GND" H 2605 7227 50  0000 C CNN
F 2 "" H 2600 7400 50  0001 C CNN
F 3 "" H 2600 7400 50  0001 C CNN
	1    2600 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 7250 2600 7250
Wire Wire Line
	2600 7250 2600 7400
$EndSCHEMATC
