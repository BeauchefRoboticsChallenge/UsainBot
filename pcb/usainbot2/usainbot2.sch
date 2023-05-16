EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "UsainBot"
Date "2022-03-03"
Rev "Gonzalo"
Comp "FabLab U. de Chile"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	5200 4150 5100 4150
Wire Wire Line
	5100 4150 5100 4050
$Comp
L power:GND #PWR0101
U 1 1 6221156A
P 5200 4150
F 0 "#PWR0101" H 5200 3900 50  0001 C CNN
F 1 "GND" H 5205 3977 50  0000 C CNN
F 2 "" H 5200 4150 50  0001 C CNN
F 3 "" H 5200 4150 50  0001 C CNN
	1    5200 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 622187BD
P 3500 2350
F 0 "#PWR0102" H 3500 2100 50  0001 C CNN
F 1 "GND" H 3505 2177 50  0000 C CNN
F 2 "" H 3500 2350 50  0001 C CNN
F 3 "" H 3500 2350 50  0001 C CNN
	1    3500 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3050 6950 3050
Wire Wire Line
	5600 3150 6950 3150
Wire Wire Line
	5600 3250 6950 3250
Wire Wire Line
	5600 3350 6950 3350
Wire Wire Line
	5600 3450 6950 3450
Wire Wire Line
	5600 3550 6950 3550
Wire Wire Line
	5600 3650 6950 3650
Wire Wire Line
	5600 3750 6950 3750
$Comp
L Connector:Conn_01x11_Female J4
U 1 1 6245E059
P 7150 3550
F 0 "J4" H 7178 3576 50  0000 L CNN
F 1 "QTR8" H 7178 3485 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x11_P2.54mm_Horizontal" H 7150 3550 50  0001 C CNN
F 3 "~" H 7150 3550 50  0001 C CNN
	1    7150 3550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Female J3
U 1 1 6246566F
P 3200 1750
F 0 "J3" H 3228 1776 50  0000 L CNN
F 1 "QTR1-R" H 3228 1685 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3200 1750 50  0001 C CNN
F 3 "~" H 3200 1750 50  0001 C CNN
	1    3200 1750
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Female J2
U 1 1 62464E3E
P 2250 1750
F 0 "J2" H 2278 1776 50  0000 L CNN
F 1 "QTR1-L" H 2278 1685 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2250 1750 50  0001 C CNN
F 3 "~" H 2250 1750 50  0001 C CNN
	1    2250 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1650 1650 1650
Wire Wire Line
	1650 1650 1650 2150
Wire Wire Line
	1650 2150 2750 2150
Wire Wire Line
	3500 2150 3500 2350
Wire Wire Line
	3000 1650 2750 1650
Wire Wire Line
	2750 1650 2750 2150
Connection ~ 2750 2150
Wire Wire Line
	2750 2150 3500 2150
Wire Wire Line
	2050 1750 1800 1750
Wire Wire Line
	1800 1750 1800 1950
Wire Wire Line
	1800 1950 2850 1950
Wire Wire Line
	3000 1750 2850 1750
Wire Wire Line
	2850 1750 2850 1950
Connection ~ 2850 1950
$Comp
L power:VCC #PWR0103
U 1 1 6246B70D
P 5000 1600
F 0 "#PWR0103" H 5000 1450 50  0001 C CNN
F 1 "VCC" H 5015 1773 50  0000 C CNN
F 2 "" H 5000 1600 50  0001 C CNN
F 3 "" H 5000 1600 50  0001 C CNN
	1    5000 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 1600 5000 2050
Wire Wire Line
	2050 1850 2050 2650
Wire Wire Line
	3000 1850 3000 2750
$Comp
L power:GND #PWR0104
U 1 1 62489AC8
P 2600 5350
F 0 "#PWR0104" H 2600 5100 50  0001 C CNN
F 1 "GND" H 2605 5177 50  0000 C CNN
F 2 "" H 2600 5350 50  0001 C CNN
F 3 "" H 2600 5350 50  0001 C CNN
	1    2600 5350
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0105
U 1 1 6248ABDC
P 3000 3350
F 0 "#PWR0105" H 3000 3200 50  0001 C CNN
F 1 "VCC" H 3015 3523 50  0000 C CNN
F 2 "" H 3000 3350 50  0001 C CNN
F 3 "" H 3000 3350 50  0001 C CNN
	1    3000 3350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 6248E364
P 5300 1600
F 0 "#PWR0106" H 5300 1450 50  0001 C CNN
F 1 "+5V" H 5315 1773 50  0000 C CNN
F 2 "" H 5300 1600 50  0001 C CNN
F 3 "" H 5300 1600 50  0001 C CNN
	1    5300 1600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0107
U 1 1 624919D7
P 2600 3350
F 0 "#PWR0107" H 2600 3200 50  0001 C CNN
F 1 "+5V" H 2615 3523 50  0000 C CNN
F 2 "" H 2600 3350 50  0001 C CNN
F 3 "" H 2600 3350 50  0001 C CNN
	1    2600 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 4050 6850 4050
Wire Wire Line
	6850 4050 6850 4350
Wire Wire Line
	6950 3950 6600 3950
$Comp
L power:+5V #PWR0108
U 1 1 6249DCAF
P 6600 3950
F 0 "#PWR0108" H 6600 3800 50  0001 C CNN
F 1 "+5V" H 6615 4123 50  0000 C CNN
F 2 "" H 6600 3950 50  0001 C CNN
F 3 "" H 6600 3950 50  0001 C CNN
	1    6600 3950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 6249EB6D
P 6850 4350
F 0 "#PWR0109" H 6850 4100 50  0001 C CNN
F 1 "GND" H 6855 4177 50  0000 C CNN
F 2 "" H 6850 4350 50  0001 C CNN
F 3 "" H 6850 4350 50  0001 C CNN
	1    6850 4350
	1    0    0    -1  
$EndComp
$Comp
L Driver_Motor:TB6612FNG U1
U 1 1 6244A2F0
P 2900 4350
F 0 "U1" H 2900 3261 50  0000 C CNN
F 1 "TB6612FNG" H 2900 3170 50  0000 C CNN
F 2 "Package_SO:SSOP-24_5.3x8.2mm_P0.65mm" H 4200 3450 50  0001 C CNN
F 3 "https://toshiba.semicon-storage.com/us/product/linear/motordriver/detail.TB6612FNG.html" H 3350 4950 50  0001 C CNN
	1    2900 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 2950 2000 4150
Wire Wire Line
	2000 4150 2300 4150
Wire Wire Line
	2000 2950 4600 2950
Wire Wire Line
	2100 3050 2100 4250
Wire Wire Line
	2100 4250 2300 4250
Wire Wire Line
	2100 3050 4600 3050
Wire Wire Line
	2300 4450 1950 4450
Wire Wire Line
	1950 4450 1950 5900
Wire Wire Line
	1950 5900 4450 5900
Wire Wire Line
	4450 5900 4450 3750
Wire Wire Line
	4450 3750 4600 3750
Wire Wire Line
	4600 3650 4350 3650
Wire Wire Line
	4350 3650 4350 5800
Wire Wire Line
	4350 5800 2050 5800
Wire Wire Line
	2050 5800 2050 4550
Wire Wire Line
	2050 4550 2300 4550
Wire Wire Line
	2300 4650 2150 4650
Wire Wire Line
	2150 4650 2150 5700
Wire Wire Line
	2150 5700 4250 5700
Wire Wire Line
	4250 5700 4250 3550
Wire Wire Line
	4250 3550 4600 3550
Wire Wire Line
	2300 4750 2250 4750
Wire Wire Line
	2250 4750 2250 5600
Wire Wire Line
	2250 5600 4150 5600
Wire Wire Line
	4150 5600 4150 3450
Wire Wire Line
	4150 3450 4600 3450
$Comp
L power:GNDPWR #PWR0110
U 1 1 6249B24F
P 3000 5350
F 0 "#PWR0110" H 3000 5150 50  0001 C CNN
F 1 "GNDPWR" H 3004 5196 50  0000 C CNN
F 2 "" H 3000 5300 50  0001 C CNN
F 3 "" H 3000 5300 50  0001 C CNN
	1    3000 5350
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR0111
U 1 1 6249BC53
P 3200 5350
F 0 "#PWR0111" H 3200 5150 50  0001 C CNN
F 1 "GNDPWR" H 3204 5196 50  0000 C CNN
F 2 "" H 3200 5300 50  0001 C CNN
F 3 "" H 3200 5300 50  0001 C CNN
	1    3200 5350
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0112
U 1 1 624A088E
P 3100 3350
F 0 "#PWR0112" H 3100 3200 50  0001 C CNN
F 1 "VCC" H 3115 3523 50  0000 C CNN
F 2 "" H 3100 3350 50  0001 C CNN
F 3 "" H 3100 3350 50  0001 C CNN
	1    3100 3350
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0113
U 1 1 624A13C8
P 3200 3350
F 0 "#PWR0113" H 3200 3200 50  0001 C CNN
F 1 "VCC" H 3215 3523 50  0000 C CNN
F 2 "" H 3200 3350 50  0001 C CNN
F 3 "" H 3200 3350 50  0001 C CNN
	1    3200 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3950 2300 2850
Wire Wire Line
	2300 2850 4600 2850
$Comp
L power:GND #PWR0114
U 1 1 624B8FCE
P 7200 2050
F 0 "#PWR0114" H 7200 1800 50  0001 C CNN
F 1 "GND" H 7205 1877 50  0000 C CNN
F 2 "" H 7200 2050 50  0001 C CNN
F 3 "" H 7200 2050 50  0001 C CNN
	1    7200 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 624B979D
P 7200 2750
F 0 "#PWR0115" H 7200 2500 50  0001 C CNN
F 1 "GND" H 7205 2577 50  0000 C CNN
F 2 "" H 7200 2750 50  0001 C CNN
F 3 "" H 7200 2750 50  0001 C CNN
	1    7200 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2600 7200 2600
Wire Wire Line
	7200 2600 7200 2750
Wire Wire Line
	7050 1900 7200 1900
Wire Wire Line
	7200 1900 7200 2050
Wire Wire Line
	6650 1900 6300 1900
Wire Wire Line
	6300 1900 6300 1750
Wire Wire Line
	6650 2600 6300 2600
Wire Wire Line
	6300 2600 6300 2450
Text Label 6300 1750 0    50   ~ 0
BTN1
Text Label 6300 2450 0    50   ~ 0
BTN2
$Comp
L MCU_Module:Arduino_Nano_v3.x ArduinoNano1
U 1 1 6220DB09
P 5100 3050
F 0 "ArduinoNano1" H 5000 1950 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 5100 1870 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 5100 3050 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 5100 3050 50  0001 C CNN
	1    5100 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 4050 5200 4150
Connection ~ 5200 4150
Wire Wire Line
	4600 3150 4150 3150
Wire Wire Line
	4600 3250 4150 3250
Text Label 4150 3150 0    50   ~ 0
BTN1
Text Label 4150 3250 0    50   ~ 0
BTN2
$Comp
L Connector:Conn_01x02_Male M1
U 1 1 624E3711
P 3850 4050
F 0 "M1" H 3822 3932 50  0000 R CNN
F 1 "Conn_01x02_Male" H 3822 4023 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal" H 3850 4050 50  0001 C CNN
F 3 "~" H 3850 4050 50  0001 C CNN
	1    3850 4050
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x02_Male M2
U 1 1 624E49C1
P 3900 4550
F 0 "M2" H 3750 4450 50  0000 L CNN
F 1 "Conn_01x02_Male" H 3200 4550 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal" H 3900 4550 50  0001 C CNN
F 3 "~" H 3900 4550 50  0001 C CNN
	1    3900 4550
	-1   0    0    1   
$EndComp
Wire Wire Line
	3700 4450 3500 4450
Wire Wire Line
	3500 4650 3700 4650
Wire Wire Line
	3700 4650 3700 4550
Wire Wire Line
	3650 4050 3500 4050
Wire Wire Line
	3500 4050 3500 4150
Wire Wire Line
	3500 3950 3650 3950
$Comp
L Connector:Conn_01x02_Male BATTJ6
U 1 1 62509397
P 5400 5000
F 0 "BATTJ6" H 5508 5181 50  0000 C CNN
F 1 "BATT" H 5508 5090 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal" H 5400 5000 50  0001 C CNN
F 3 "~" H 5400 5000 50  0001 C CNN
	1    5400 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 5100 5950 5100
Wire Wire Line
	5950 5100 5950 5250
$Comp
L power:VCC #PWR03
U 1 1 6252143A
P 6550 4750
F 0 "#PWR03" H 6550 4600 50  0001 C CNN
F 1 "VCC" H 6565 4923 50  0000 C CNN
F 2 "" H 6550 4750 50  0001 C CNN
F 3 "" H 6550 4750 50  0001 C CNN
	1    6550 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 4750 6550 4800
$Comp
L SparkFun-Resistors:RESISTOR1206 R0
U 1 1 62527907
P 6150 5250
F 0 "R0" H 6150 5400 45  0000 C CNN
F 1 "RESISTOR1206" H 6150 5300 45  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 6150 5400 20  0001 C CNN
F 3 "" H 6150 5250 60  0001 C CNN
F 4 " " H 6150 5371 60  0000 C CNN "Field4"
	1    6150 5250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 62528BF3
P 6350 5250
F 0 "#PWR02" H 6350 5000 50  0001 C CNN
F 1 "GND" H 6355 5077 50  0000 C CNN
F 2 "" H 6350 5250 50  0001 C CNN
F 3 "" H 6350 5250 50  0001 C CNN
	1    6350 5250
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR01
U 1 1 62528FB6
P 5950 5250
F 0 "#PWR01" H 5950 5050 50  0001 C CNN
F 1 "GNDPWR" H 5954 5096 50  0000 C CNN
F 2 "" H 5950 5200 50  0001 C CNN
F 3 "" H 5950 5200 50  0001 C CNN
	1    5950 5250
	1    0    0    -1  
$EndComp
Connection ~ 5950 5250
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 6253DB9C
P 6550 4800
F 0 "#FLG0101" H 6550 4875 50  0001 C CNN
F 1 "PWR_FLAG" V 6550 4928 50  0000 L CNN
F 2 "" H 6550 4800 50  0001 C CNN
F 3 "~" H 6550 4800 50  0001 C CNN
	1    6550 4800
	0    1    1    0   
$EndComp
Connection ~ 6550 4800
Wire Wire Line
	6550 4800 6550 5000
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 6253E2C8
P 5950 5100
F 0 "#FLG0102" H 5950 5175 50  0001 C CNN
F 1 "PWR_FLAG" V 5950 5228 50  0000 L CNN
F 2 "" H 5950 5100 50  0001 C CNN
F 3 "~" H 5950 5100 50  0001 C CNN
	1    5950 5100
	0    1    1    0   
$EndComp
Connection ~ 5950 5100
NoConn ~ 5200 2050
NoConn ~ 5600 2450
NoConn ~ 5600 2550
NoConn ~ 5600 2850
NoConn ~ 4600 3350
NoConn ~ 6950 3850
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 62560DFD
P 5200 4150
F 0 "#FLG0103" H 5200 4225 50  0001 C CNN
F 1 "PWR_FLAG" V 5200 4278 50  0000 L CNN
F 2 "" H 5200 4150 50  0001 C CNN
F 3 "~" H 5200 4150 50  0001 C CNN
	1    5200 4150
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x03_Female J5
U 1 1 6259D194
P 6200 4700
F 0 "J5" V 6138 4512 50  0000 R CNN
F 1 "SWITCH" V 6047 4512 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6200 4700 50  0001 C CNN
F 3 "~" H 6200 4700 50  0001 C CNN
	1    6200 4700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6300 5000 6550 5000
Wire Wire Line
	6200 4900 6200 5000
Wire Wire Line
	5600 5000 6200 5000
Wire Wire Line
	6300 4900 6300 5000
NoConn ~ 6100 4900
Wire Wire Line
	5300 1600 5300 1800
Wire Wire Line
	2850 1950 5300 1950
Connection ~ 5300 1950
Wire Wire Line
	5300 1950 5300 2050
Wire Wire Line
	2050 2650 4600 2650
Wire Wire Line
	3000 2750 4600 2750
$Comp
L Connector:Conn_01x04_Female J1
U 1 1 625BE861
P 3850 1300
F 0 "J1" V 3788 1012 50  0000 R CNN
F 1 "BT_MOD" V 3697 1012 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3850 1300 50  0001 C CNN
F 3 "~" H 3850 1300 50  0001 C CNN
	1    3850 1300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4050 1500 4050 1800
Wire Wire Line
	4050 1800 5300 1800
Connection ~ 5300 1800
Wire Wire Line
	5300 1800 5300 1950
Wire Wire Line
	3950 1500 3950 2150
Wire Wire Line
	3950 2150 3500 2150
Connection ~ 3500 2150
Wire Wire Line
	3850 1500 3850 2450
Wire Wire Line
	3850 2450 4600 2450
Wire Wire Line
	3750 1500 3750 2550
Wire Wire Line
	3750 2550 4600 2550
$Comp
L SparkFun-Switches:MOMENTARY-SWITCH-SPST-PTH-6.0MM-KIT S1
U 1 1 6244FCC3
P 6850 1900
F 0 "S1" H 6850 2210 45  0000 C CNN
F 1 "BTN1" H 6850 2126 45  0000 C CNN
F 2 "TACTILE_SWITCH_PTH_6.0MM_KIT" H 6850 2100 20  0001 C CNN
F 3 "" H 6850 1900 50  0001 C CNN
F 4 "SWCH-08441" H 6850 2031 60  0000 C CNN "Field4"
	1    6850 1900
	1    0    0    -1  
$EndComp
$Comp
L SparkFun-Switches:MOMENTARY-SWITCH-SPST-PTH-6.0MM-KIT S2
U 1 1 62450E75
P 6850 2600
F 0 "S2" H 6850 2910 45  0000 C CNN
F 1 "BTN2" H 6850 2826 45  0000 C CNN
F 2 "TACTILE_SWITCH_PTH_6.0MM_KIT" H 6850 2800 20  0001 C CNN
F 3 "" H 6850 2600 50  0001 C CNN
F 4 "SWCH-08441" H 6850 2731 60  0000 C CNN "Field4"
	1    6850 2600
	1    0    0    -1  
$EndComp
$Comp
L SparkFun-Switches:MOMENTARY-SWITCH-SPST-PTH-6.0MM S?
U 1 1 62462CCB
P 8450 1700
F 0 "S?" H 8450 2010 45  0000 C CNN
F 1 "MOMENTARY-SWITCH-SPST-PTH-6.0MM" H 8450 1926 45  0000 C CNN
F 2 "TACTILE_SWITCH_PTH_6.0MM" H 8450 1900 20  0001 C CNN
F 3 "" H 8450 1700 50  0001 C CNN
F 4 " SWCH-08441" H 8450 1831 60  0000 C CNN "Field4"
	1    8450 1700
	1    0    0    -1  
$EndComp
NoConn ~ 8250 1700
NoConn ~ 8650 1700
$EndSCHEMATC
