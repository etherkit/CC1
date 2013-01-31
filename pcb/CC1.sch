EESchema Schematic File Version 2  date Thu 31 Jan 2013 03:10:17 PM PST
LIBS:NT7S
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
LIBS:CC1-cache
EELAYER 27 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 3
Title "CC1 Transceiver"
Date "31 jan 2013"
Rev "A"
Comp "Etherkit"
Comment1 ""
Comment2 "Creative Commons Licence CC-BY-SA"
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 2550 2600 1650 1100
U 50DE2D7A
F0 "Receiver" 50
F1 "CC1RX.sch" 50
$EndSheet
$Sheet
S 4900 2600 1700 1100
U 50DE2D98
F0 "Transmitter and Microcontroller" 50
F1 "CC1TXUC.sch" 50
$EndSheet
$Comp
L +5V #PWR01
U 1 1 50E3F58E
P 4000 4750
F 0 "#PWR01" H 4000 4840 20  0001 C CNN
F 1 "+5V" H 4000 4840 30  0000 C CNN
	1    4000 4750
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR02
U 1 1 50E3F59D
P 3650 4750
F 0 "#PWR02" H 3650 4710 30  0001 C CNN
F 1 "+3.3V" H 3650 4860 30  0000 C CNN
	1    3650 4750
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR03
U 1 1 50E3F5AC
P 3350 4750
F 0 "#PWR03" H 3350 4700 20  0001 C CNN
F 1 "+12V" H 3350 4850 30  0000 C CNN
	1    3350 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 50E3F5BB
P 4250 4800
F 0 "#PWR04" H 4250 4800 30  0001 C CNN
F 1 "GND" H 4250 4730 30  0001 C CNN
	1    4250 4800
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG05
U 1 1 50E3F5D4
P 4250 4800
F 0 "#FLG05" H 4250 4895 30  0001 C CNN
F 1 "PWR_FLAG" H 4250 4980 30  0000 C CNN
	1    4250 4800
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG06
U 1 1 50E3F5E3
P 4000 4750
F 0 "#FLG06" H 4000 4845 30  0001 C CNN
F 1 "PWR_FLAG" H 4000 4930 30  0000 C CNN
	1    4000 4750
	1    0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG07
U 1 1 50E3F5F2
P 3650 4750
F 0 "#FLG07" H 3650 4845 30  0001 C CNN
F 1 "PWR_FLAG" H 3650 4930 30  0000 C CNN
	1    3650 4750
	1    0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG08
U 1 1 50E3F601
P 3350 4750
F 0 "#FLG08" H 3350 4845 30  0001 C CNN
F 1 "PWR_FLAG" H 3350 4930 30  0000 C CNN
	1    3350 4750
	1    0    0    1   
$EndComp
$EndSCHEMATC
