EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "PIC-CNT64"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 "Luke \"Bigbass\" Stadem"
Comment4 "Editors:"
$EndDescr
$Comp
L MCU_Microchip_PIC18:PIC18LF4550-IP U1
U 1 1 5EF14FF2
P 4050 2675
F 0 "U1" H 4500 4125 50  0000 C CNN
F 1 "PIC18LF4550-IP" H 4500 4025 50  0000 C CNN
F 2 "Package_DIP:DIP-40_W15.24mm" H 4050 2875 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/39760d.pdf" H 4050 2425 50  0001 C CNN
	1    4050 2675
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5EF173F6
P 3950 4225
F 0 "#PWR05" H 3950 3975 50  0001 C CNN
F 1 "GND" H 3950 4050 50  0000 C CNN
F 2 "" H 3950 4225 50  0001 C CNN
F 3 "" H 3950 4225 50  0001 C CNN
	1    3950 4225
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR04
U 1 1 5EF18529
P 3950 1125
F 0 "#PWR04" H 3950 975 50  0001 C CNN
F 1 "+3.3V" H 3950 1300 50  0000 C CNN
F 2 "" H 3950 1125 50  0001 C CNN
F 3 "" H 3950 1125 50  0001 C CNN
	1    3950 1125
	1    0    0    -1  
$EndComp
Text GLabel 5250 3675 2    50   Input ~ 0
PIN_A
Wire Wire Line
	5150 3675 5250 3675
Text GLabel 5250 3575 2    50   Input ~ 0
PIN_B
Text GLabel 5250 3475 2    50   Input ~ 0
PIN_Z
Wire Wire Line
	5150 3575 5250 3575
Wire Wire Line
	5150 3475 5250 3475
Text GLabel 5250 3275 2    50   Input ~ 0
STICK_XB
Text GLabel 5250 3175 2    50   Input ~ 0
STICK_YB
Text GLabel 5250 3075 2    50   Input ~ 0
PIN_dU
Text GLabel 5250 2975 2    50   Input ~ 0
PIN_dD
Text GLabel 5250 2875 2    50   Input ~ 0
PIN_dL
Text GLabel 5250 2775 2    50   Input ~ 0
PIN_dR
Text GLabel 5250 2575 2    50   BiDi ~ 0
DATA_IO
Text GLabel 5250 2175 2    50   Input ~ 0
PIN_cU
Text GLabel 5250 2075 2    50   Input ~ 0
PIN_cD
Text GLabel 5250 1975 2    50   Input ~ 0
PIN_cL
Text GLabel 5250 1875 2    50   Input ~ 0
PIN_cR
Text GLabel 5250 1775 2    50   Input ~ 0
PIN_LT
Text GLabel 5250 1675 2    50   Input ~ 0
PIN_RT
Text GLabel 5250 1575 2    50   Input ~ 0
PIN_START
Wire Wire Line
	5150 3275 5250 3275
Wire Wire Line
	5250 3175 5150 3175
Wire Wire Line
	5150 3075 5250 3075
Wire Wire Line
	5250 2975 5150 2975
Wire Wire Line
	5150 2875 5250 2875
Wire Wire Line
	5250 2775 5150 2775
Wire Wire Line
	5150 2575 5250 2575
Wire Wire Line
	5150 2175 5250 2175
Wire Wire Line
	5250 2075 5150 2075
Wire Wire Line
	5150 1975 5250 1975
Wire Wire Line
	5250 1875 5150 1875
Wire Wire Line
	5150 1775 5250 1775
Wire Wire Line
	5250 1675 5150 1675
NoConn ~ 5150 2375
NoConn ~ 5150 2675
Text GLabel 2850 2975 0    50   Input ~ 0
STICK_XA
Text GLabel 2850 2675 0    50   Input ~ 0
STICK_YA
Text GLabel 2850 1675 0    50   Output ~ 0
DEBUG_1
Text GLabel 2850 1575 0    50   Output ~ 0
DEBUG_2
NoConn ~ 2950 2575
NoConn ~ 2950 2875
NoConn ~ 2950 3075
NoConn ~ 2950 3175
NoConn ~ 2950 3275
NoConn ~ 2950 2075
NoConn ~ 2950 1975
NoConn ~ 2950 1875
NoConn ~ 2950 1775
Wire Wire Line
	2850 1575 2950 1575
Wire Wire Line
	2850 1675 2950 1675
Wire Wire Line
	2900 2775 2950 2775
Wire Wire Line
	2850 2675 2900 2675
Wire Wire Line
	2850 2975 2950 2975
Wire Wire Line
	2900 2775 2900 2675
Connection ~ 2900 2675
Wire Wire Line
	2900 2675 2950 2675
NoConn ~ 5150 3775
$Comp
L Connector_Generic:Conn_01x06 J1
U 1 1 5EF40668
P 4025 7025
F 0 "J1" H 4105 7017 50  0000 L CNN
F 1 "Analog Stick" H 4105 6926 50  0000 L CNN
F 2 "Connector_JST:JST_PH_B6B-PH-K_1x06_P2.00mm_Vertical" H 4025 7025 50  0001 C CNN
F 3 "~" H 4025 7025 50  0001 C CNN
	1    4025 7025
	1    0    0    -1  
$EndComp
Text GLabel 3725 6825 0    50   Output ~ 0
STICK_XA
Text GLabel 3725 7125 0    50   Output ~ 0
STICK_XB
Text GLabel 3725 7225 0    50   Output ~ 0
STICK_YB
Text GLabel 3725 7325 0    50   Output ~ 0
STICK_YA
$Comp
L power:GND #PWR03
U 1 1 5EF44A8B
P 3175 7025
F 0 "#PWR03" H 3175 6775 50  0001 C CNN
F 1 "GND" H 3180 6852 50  0000 C CNN
F 2 "" H 3175 7025 50  0001 C CNN
F 3 "" H 3175 7025 50  0001 C CNN
	1    3175 7025
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR02
U 1 1 5EF454EB
P 3175 6925
F 0 "#PWR02" H 3175 6775 50  0001 C CNN
F 1 "+3.3V" H 3190 7098 50  0000 C CNN
F 2 "" H 3175 6925 50  0001 C CNN
F 3 "" H 3175 6925 50  0001 C CNN
	1    3175 6925
	1    0    0    -1  
$EndComp
Wire Wire Line
	3175 7025 3825 7025
Wire Wire Line
	3825 6925 3175 6925
Wire Wire Line
	3725 6825 3825 6825
Wire Wire Line
	3825 7125 3725 7125
Wire Wire Line
	3725 7225 3825 7225
Wire Wire Line
	3825 7325 3725 7325
$Comp
L Connector_Generic:Conn_01x03 J2
U 1 1 5EF4CAE4
P 5800 7050
F 0 "J2" H 5880 7092 50  0000 L CNN
F 1 "Controller Port" H 5880 7001 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical" H 5800 7050 50  0001 C CNN
F 3 "~" H 5800 7050 50  0001 C CNN
	1    5800 7050
	1    0    0    -1  
$EndComp
Text GLabel 5500 7050 0    50   BiDi ~ 0
DATA_IO
$Comp
L power:+3.3V #PWR06
U 1 1 5EF50C66
P 5500 6950
F 0 "#PWR06" H 5500 6800 50  0001 C CNN
F 1 "+3.3V" H 5515 7123 50  0000 C CNN
F 2 "" H 5500 6950 50  0001 C CNN
F 3 "" H 5500 6950 50  0001 C CNN
	1    5500 6950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5EF515EC
P 5500 7150
F 0 "#PWR07" H 5500 6900 50  0001 C CNN
F 1 "GND" H 5505 6977 50  0000 C CNN
F 2 "" H 5500 7150 50  0001 C CNN
F 3 "" H 5500 7150 50  0001 C CNN
	1    5500 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 6950 5600 6950
Wire Wire Line
	5600 7050 5500 7050
Wire Wire Line
	5500 7150 5600 7150
Text GLabel 1350 6750 0    50   Output ~ 0
PIN_START
Text GLabel 1350 6850 0    50   Output ~ 0
PIN_RT
Text GLabel 1350 6950 0    50   Output ~ 0
PIN_LT
Text GLabel 1350 7050 0    50   Output ~ 0
PIN_cR
Text GLabel 1350 7150 0    50   Output ~ 0
PIN_cL
Text GLabel 1350 7250 0    50   Output ~ 0
PIN_cD
Text GLabel 1350 7350 0    50   Output ~ 0
PIN_cU
Text GLabel 2050 7350 2    50   Output ~ 0
PIN_dR
Text GLabel 2050 7250 2    50   Output ~ 0
PIN_dL
Text GLabel 2050 7150 2    50   Output ~ 0
PIN_dD
Text GLabel 2050 7050 2    50   Output ~ 0
PIN_dU
Text GLabel 2050 6950 2    50   Output ~ 0
PIN_Z
Text GLabel 2050 6850 2    50   Output ~ 0
PIN_B
Text GLabel 2050 6750 2    50   Output ~ 0
PIN_A
Wire Wire Line
	1450 6750 1350 6750
Wire Wire Line
	1350 6850 1450 6850
Wire Wire Line
	1450 6950 1350 6950
Wire Wire Line
	1350 7050 1450 7050
Wire Wire Line
	1450 7150 1350 7150
Wire Wire Line
	1350 7250 1450 7250
Wire Wire Line
	1450 7350 1350 7350
Wire Wire Line
	1950 7350 2050 7350
Wire Wire Line
	2050 7250 1950 7250
Wire Wire Line
	1950 7150 2050 7150
Wire Wire Line
	2050 7050 1950 7050
Wire Wire Line
	1950 6950 2050 6950
Wire Wire Line
	2050 6850 1950 6850
Wire Wire Line
	1950 6750 2050 6750
Wire Wire Line
	3950 1125 3950 1200
Wire Wire Line
	3950 4075 3950 4150
Wire Wire Line
	4050 1275 4050 1200
Wire Wire Line
	4050 1200 3950 1200
Connection ~ 3950 1200
Wire Wire Line
	3950 1200 3950 1275
Wire Wire Line
	3950 4150 4050 4150
Wire Wire Line
	4050 4150 4050 4075
Connection ~ 3950 4150
Wire Wire Line
	3950 4150 3950 4225
Wire Wire Line
	5150 1575 5250 1575
Connection ~ 9700 1225
Wire Wire Line
	9700 1225 9750 1225
Wire Wire Line
	9400 1325 9400 1225
Wire Wire Line
	9700 1325 10500 1325
$Comp
L Device:R R15
U 1 1 5EF58188
P 9550 1225
F 0 "R15" V 9600 1325 50  0000 L CNN
F 1 "10k" V 9550 1150 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 1225 50  0001 C CNN
F 3 "~" H 9550 1225 50  0001 C CNN
	1    9550 1225
	0    1    1    0   
$EndComp
$Comp
L Device:C C10
U 1 1 5EF58792
P 9550 1075
F 0 "C10" V 9500 1175 50  0000 C CNN
F 1 "0.1uF" V 9500 925 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 9588 925 50  0001 C CNN
F 3 "~" H 9550 1075 50  0001 C CNN
	1    9550 1075
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 1075 9400 1225
Connection ~ 9400 1225
Wire Wire Line
	10500 1325 10500 1150
$Comp
L Switch:SW_Push SW8
U 1 1 5EF59298
P 9950 1225
F 0 "SW8" H 9950 1510 50  0000 C CNN
F 1 "PIN_dR" H 9950 1419 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 9950 1425 50  0001 C CNN
F 3 "~" H 9950 1425 50  0001 C CNN
	1    9950 1225
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 1225 10275 1225
Wire Wire Line
	9700 1225 9700 1075
Wire Wire Line
	9400 1225 9350 1225
Wire Wire Line
	10275 1225 10275 1150
Text GLabel 7550 1225 0    50   Output ~ 0
PIN_START
$Comp
L power:+3.3V #PWR010
U 1 1 5EF5A2E6
P 10275 1150
F 0 "#PWR010" H 10275 1000 50  0001 C CNN
F 1 "+3.3V" H 10290 1323 50  0000 C CNN
F 2 "" H 10275 1150 50  0001 C CNN
F 3 "" H 10275 1150 50  0001 C CNN
	1    10275 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5EF59C8D
P 10500 1150
F 0 "#PWR011" H 10500 900 50  0001 C CNN
F 1 "GND" H 10505 977 50  0000 C CNN
F 2 "" H 10500 1150 50  0001 C CNN
F 3 "" H 10500 1150 50  0001 C CNN
	1    10500 1150
	-1   0    0    1   
$EndComp
$Comp
L Device:Crystal Y1
U 1 1 5EF1B91D
P 1725 2275
F 0 "Y1" V 1679 2406 50  0000 L CNN
F 1 "8MHz" V 1770 2406 50  0000 L CNN
F 2 "Crystal:Crystal_HC49-4H_Vertical" H 1725 2275 50  0001 C CNN
F 3 "~" H 1725 2275 50  0001 C CNN
	1    1725 2275
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 5EF1DD13
P 1375 2025
F 0 "C1" V 1123 2025 50  0000 C CNN
F 1 "27pF" V 1214 2025 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D3.4mm_W2.1mm_P2.50mm" H 1413 1875 50  0001 C CNN
F 3 "~" H 1375 2025 50  0001 C CNN
	1    1375 2025
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 5EF1E5B5
P 1375 2525
F 0 "C2" V 1627 2525 50  0000 C CNN
F 1 "27pF" V 1536 2525 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D3.4mm_W2.1mm_P2.50mm" H 1413 2375 50  0001 C CNN
F 3 "~" H 1375 2525 50  0001 C CNN
	1    1375 2525
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5EF1ED44
P 1025 2275
F 0 "#PWR01" H 1025 2025 50  0001 C CNN
F 1 "GND" H 1030 2102 50  0000 C CNN
F 2 "" H 1025 2275 50  0001 C CNN
F 3 "" H 1025 2275 50  0001 C CNN
	1    1025 2275
	1    0    0    -1  
$EndComp
Wire Wire Line
	1225 2025 1225 2275
Wire Wire Line
	1725 2025 1725 2125
Wire Wire Line
	1725 2525 1725 2425
Wire Wire Line
	1225 2275 1025 2275
Connection ~ 1225 2275
Wire Wire Line
	1225 2275 1225 2525
Wire Wire Line
	2125 2025 2125 2175
Wire Wire Line
	2125 2375 2125 2525
Wire Wire Line
	1725 2025 1525 2025
Wire Wire Line
	1725 2525 1525 2525
Wire Wire Line
	2125 2525 1725 2525
Connection ~ 1725 2525
Wire Wire Line
	2125 2025 1725 2025
Connection ~ 1725 2025
Wire Notes Line
	2225 1675 2225 2675
Wire Notes Line
	2225 2675 875  2675
Wire Notes Line
	875  2675 875  1675
Wire Notes Line
	875  1675 2225 1675
Text Notes 875  1625 0    50   ~ 0
Crystal Oscillator
Wire Wire Line
	2125 2375 2950 2375
Wire Wire Line
	2125 2175 2950 2175
Connection ~ 7900 3925
Wire Wire Line
	7900 3925 7950 3925
$Comp
L Device:R R14
U 1 1 5F03963A
P 7750 4025
F 0 "R14" V 7700 3875 50  0000 C CNN
F 1 "100k" V 7745 4025 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 4025 50  0001 C CNN
F 3 "~" H 7750 4025 50  0001 C CNN
	1    7750 4025
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7600 4025 7600 3925
$Comp
L Device:R R13
U 1 1 5F039642
P 7750 3925
F 0 "R13" V 7800 4025 50  0000 L CNN
F 1 "10k" V 7750 3850 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 3925 50  0001 C CNN
F 3 "~" H 7750 3925 50  0001 C CNN
	1    7750 3925
	0    1    1    0   
$EndComp
$Comp
L Device:C C9
U 1 1 5F039648
P 7750 3775
F 0 "C9" V 7700 3875 50  0000 C CNN
F 1 "0.1uF" V 7700 3625 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 7788 3625 50  0001 C CNN
F 3 "~" H 7750 3775 50  0001 C CNN
	1    7750 3775
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 3775 7600 3925
Connection ~ 7600 3925
$Comp
L Switch:SW_Push SW7
U 1 1 5F039651
P 8150 3925
F 0 "SW7" H 8150 4210 50  0000 C CNN
F 1 "PIN_cU" H 8150 4119 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 8150 4125 50  0001 C CNN
F 3 "~" H 8150 4125 50  0001 C CNN
	1    8150 3925
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 3925 7900 3775
Wire Wire Line
	7600 3925 7550 3925
Wire Wire Line
	8350 3925 8475 3925
Wire Wire Line
	7900 4025 8700 4025
Connection ~ 9700 1675
Wire Wire Line
	9700 1675 9750 1675
$Comp
L Device:R R18
U 1 1 5F03FAE2
P 9550 1775
F 0 "R18" V 9500 1625 50  0000 C CNN
F 1 "100k" V 9545 1775 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 1775 50  0001 C CNN
F 3 "~" H 9550 1775 50  0001 C CNN
	1    9550 1775
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9400 1775 9400 1675
$Comp
L Device:R R17
U 1 1 5F03FAE9
P 9550 1675
F 0 "R17" V 9600 1775 50  0000 L CNN
F 1 "10k" V 9550 1600 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 1675 50  0001 C CNN
F 3 "~" H 9550 1675 50  0001 C CNN
	1    9550 1675
	0    1    1    0   
$EndComp
$Comp
L Device:C C11
U 1 1 5F03FAEF
P 9550 1525
F 0 "C11" V 9500 1625 50  0000 C CNN
F 1 "0.1uF" V 9500 1375 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 9588 1375 50  0001 C CNN
F 3 "~" H 9550 1525 50  0001 C CNN
	1    9550 1525
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 1525 9400 1675
Connection ~ 9400 1675
$Comp
L Switch:SW_Push SW9
U 1 1 5F03FAF7
P 9950 1675
F 0 "SW9" H 9950 1960 50  0000 C CNN
F 1 "PIN_dL" H 9950 1869 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 9950 1875 50  0001 C CNN
F 3 "~" H 9950 1875 50  0001 C CNN
	1    9950 1675
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 1675 9700 1525
Wire Wire Line
	9400 1675 9350 1675
Wire Wire Line
	10150 1675 10275 1675
Wire Wire Line
	9700 1775 10500 1775
Connection ~ 9700 2125
Wire Wire Line
	9700 2125 9750 2125
$Comp
L Device:R R20
U 1 1 5F043208
P 9550 2225
F 0 "R20" V 9500 2075 50  0000 C CNN
F 1 "100k" V 9545 2225 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 2225 50  0001 C CNN
F 3 "~" H 9550 2225 50  0001 C CNN
	1    9550 2225
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9400 2225 9400 2125
$Comp
L Device:R R19
U 1 1 5F04320F
P 9550 2125
F 0 "R19" V 9600 2225 50  0000 L CNN
F 1 "10k" V 9550 2050 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 2125 50  0001 C CNN
F 3 "~" H 9550 2125 50  0001 C CNN
	1    9550 2125
	0    1    1    0   
$EndComp
$Comp
L Device:C C12
U 1 1 5F043215
P 9550 1975
F 0 "C12" V 9500 2075 50  0000 C CNN
F 1 "0.1uF" V 9500 1825 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 9588 1825 50  0001 C CNN
F 3 "~" H 9550 1975 50  0001 C CNN
	1    9550 1975
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 1975 9400 2125
Connection ~ 9400 2125
$Comp
L Switch:SW_Push SW10
U 1 1 5F04321D
P 9950 2125
F 0 "SW10" H 9950 2410 50  0000 C CNN
F 1 "PIN_dD" H 9950 2319 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 9950 2325 50  0001 C CNN
F 3 "~" H 9950 2325 50  0001 C CNN
	1    9950 2125
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 2125 9700 1975
Wire Wire Line
	9400 2125 9350 2125
Wire Wire Line
	10150 2125 10275 2125
Wire Wire Line
	9700 2225 10500 2225
Connection ~ 9700 2575
Wire Wire Line
	9700 2575 9750 2575
$Comp
L Device:R R22
U 1 1 5F0470FE
P 9550 2675
F 0 "R22" V 9500 2525 50  0000 C CNN
F 1 "100k" V 9545 2675 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 2675 50  0001 C CNN
F 3 "~" H 9550 2675 50  0001 C CNN
	1    9550 2675
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9400 2675 9400 2575
$Comp
L Device:R R21
U 1 1 5F047105
P 9550 2575
F 0 "R21" V 9600 2675 50  0000 L CNN
F 1 "10k" V 9550 2500 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 2575 50  0001 C CNN
F 3 "~" H 9550 2575 50  0001 C CNN
	1    9550 2575
	0    1    1    0   
$EndComp
$Comp
L Device:C C13
U 1 1 5F04710B
P 9550 2425
F 0 "C13" V 9500 2525 50  0000 C CNN
F 1 "0.1uF" V 9500 2275 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 9588 2275 50  0001 C CNN
F 3 "~" H 9550 2425 50  0001 C CNN
	1    9550 2425
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 2425 9400 2575
Connection ~ 9400 2575
$Comp
L Switch:SW_Push SW11
U 1 1 5F047113
P 9950 2575
F 0 "SW11" H 9950 2860 50  0000 C CNN
F 1 "PIN_dU" H 9950 2769 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 9950 2775 50  0001 C CNN
F 3 "~" H 9950 2775 50  0001 C CNN
	1    9950 2575
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 2575 9700 2425
Wire Wire Line
	9400 2575 9350 2575
Wire Wire Line
	10150 2575 10275 2575
Wire Wire Line
	9700 2675 10500 2675
Connection ~ 9700 3025
Wire Wire Line
	9700 3025 9750 3025
$Comp
L Device:R R24
U 1 1 5F04B710
P 9550 3125
F 0 "R24" V 9500 2975 50  0000 C CNN
F 1 "100k" V 9545 3125 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 3125 50  0001 C CNN
F 3 "~" H 9550 3125 50  0001 C CNN
	1    9550 3125
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9400 3125 9400 3025
$Comp
L Device:R R23
U 1 1 5F04B717
P 9550 3025
F 0 "R23" V 9600 3125 50  0000 L CNN
F 1 "10k" V 9550 2950 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 3025 50  0001 C CNN
F 3 "~" H 9550 3025 50  0001 C CNN
	1    9550 3025
	0    1    1    0   
$EndComp
$Comp
L Device:C C14
U 1 1 5F04B71D
P 9550 2875
F 0 "C14" V 9500 2975 50  0000 C CNN
F 1 "0.1uF" V 9500 2725 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 9588 2725 50  0001 C CNN
F 3 "~" H 9550 2875 50  0001 C CNN
	1    9550 2875
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 2875 9400 3025
Connection ~ 9400 3025
$Comp
L Switch:SW_Push SW12
U 1 1 5F04B725
P 9950 3025
F 0 "SW12" H 9950 3310 50  0000 C CNN
F 1 "PIN_Z" H 9950 3219 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 9950 3225 50  0001 C CNN
F 3 "~" H 9950 3225 50  0001 C CNN
	1    9950 3025
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 3025 9700 2875
Wire Wire Line
	9400 3025 9350 3025
Wire Wire Line
	10150 3025 10275 3025
Wire Wire Line
	9700 3125 10500 3125
Connection ~ 9700 3475
Wire Wire Line
	9700 3475 9750 3475
$Comp
L Device:R R26
U 1 1 5F05D45C
P 9550 3575
F 0 "R26" V 9500 3425 50  0000 C CNN
F 1 "100k" V 9545 3575 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 3575 50  0001 C CNN
F 3 "~" H 9550 3575 50  0001 C CNN
	1    9550 3575
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9400 3575 9400 3475
$Comp
L Device:R R25
U 1 1 5F05D463
P 9550 3475
F 0 "R25" V 9600 3575 50  0000 L CNN
F 1 "10k" V 9550 3400 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 3475 50  0001 C CNN
F 3 "~" H 9550 3475 50  0001 C CNN
	1    9550 3475
	0    1    1    0   
$EndComp
$Comp
L Device:C C15
U 1 1 5F05D469
P 9550 3325
F 0 "C15" V 9500 3425 50  0000 C CNN
F 1 "0.1uF" V 9500 3175 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 9588 3175 50  0001 C CNN
F 3 "~" H 9550 3325 50  0001 C CNN
	1    9550 3325
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 3325 9400 3475
Connection ~ 9400 3475
$Comp
L Switch:SW_Push SW13
U 1 1 5F05D471
P 9950 3475
F 0 "SW13" H 9950 3760 50  0000 C CNN
F 1 "PIN_B" H 9950 3669 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 9950 3675 50  0001 C CNN
F 3 "~" H 9950 3675 50  0001 C CNN
	1    9950 3475
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 3475 9700 3325
Wire Wire Line
	9400 3475 9350 3475
Wire Wire Line
	10150 3475 10275 3475
Wire Wire Line
	9700 3575 10500 3575
Connection ~ 9700 3925
Wire Wire Line
	9700 3925 9750 3925
$Comp
L Device:R R28
U 1 1 5F05D47D
P 9550 4025
F 0 "R28" V 9500 3875 50  0000 C CNN
F 1 "100k" V 9545 4025 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 4025 50  0001 C CNN
F 3 "~" H 9550 4025 50  0001 C CNN
	1    9550 4025
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9400 4025 9400 3925
$Comp
L Device:R R27
U 1 1 5F05D484
P 9550 3925
F 0 "R27" V 9600 4025 50  0000 L CNN
F 1 "10k" V 9550 3850 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 3925 50  0001 C CNN
F 3 "~" H 9550 3925 50  0001 C CNN
	1    9550 3925
	0    1    1    0   
$EndComp
$Comp
L Device:C C16
U 1 1 5F05D48A
P 9550 3775
F 0 "C16" V 9500 3875 50  0000 C CNN
F 1 "0.1uF" V 9500 3625 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 9588 3625 50  0001 C CNN
F 3 "~" H 9550 3775 50  0001 C CNN
	1    9550 3775
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 3775 9400 3925
Connection ~ 9400 3925
$Comp
L Switch:SW_Push SW14
U 1 1 5F05D492
P 9950 3925
F 0 "SW14" H 9950 4210 50  0000 C CNN
F 1 "PIN_A" H 9950 4119 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 9950 4125 50  0001 C CNN
F 3 "~" H 9950 4125 50  0001 C CNN
	1    9950 3925
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 3925 9700 3775
Wire Wire Line
	9400 3925 9350 3925
Wire Wire Line
	10150 3925 10275 3925
Wire Wire Line
	9700 4025 10500 4025
Connection ~ 7900 1225
Wire Wire Line
	7900 1225 7950 1225
$Comp
L Device:R R2
U 1 1 5F05D49E
P 7750 1325
F 0 "R2" V 7700 1175 50  0000 C CNN
F 1 "100k" V 7745 1325 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 1325 50  0001 C CNN
F 3 "~" H 7750 1325 50  0001 C CNN
	1    7750 1325
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7600 1325 7600 1225
$Comp
L Device:R R1
U 1 1 5F05D4A5
P 7750 1225
F 0 "R1" V 7800 1325 50  0000 L CNN
F 1 "10k" V 7750 1150 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 1225 50  0001 C CNN
F 3 "~" H 7750 1225 50  0001 C CNN
	1    7750 1225
	0    1    1    0   
$EndComp
$Comp
L Device:C C3
U 1 1 5F05D4AB
P 7750 1075
F 0 "C3" V 7700 1175 50  0000 C CNN
F 1 "0.1uF" V 7700 925 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 7788 925 50  0001 C CNN
F 3 "~" H 7750 1075 50  0001 C CNN
	1    7750 1075
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 1075 7600 1225
Connection ~ 7600 1225
$Comp
L Switch:SW_Push SW1
U 1 1 5F05D4B3
P 8150 1225
F 0 "SW1" H 8150 1510 50  0000 C CNN
F 1 "PIN_START" H 8150 1419 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 8150 1425 50  0001 C CNN
F 3 "~" H 8150 1425 50  0001 C CNN
	1    8150 1225
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 1225 7900 1075
Wire Wire Line
	7600 1225 7550 1225
Wire Wire Line
	8350 1225 8475 1225
Wire Wire Line
	7900 1325 8700 1325
Connection ~ 7900 1675
Wire Wire Line
	7900 1675 7950 1675
$Comp
L Device:R R4
U 1 1 5F05D4BF
P 7750 1775
F 0 "R4" V 7700 1625 50  0000 C CNN
F 1 "100k" V 7745 1775 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 1775 50  0001 C CNN
F 3 "~" H 7750 1775 50  0001 C CNN
	1    7750 1775
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7600 1775 7600 1675
$Comp
L Device:R R3
U 1 1 5F05D4C6
P 7750 1675
F 0 "R3" V 7800 1775 50  0000 L CNN
F 1 "10k" V 7750 1600 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 1675 50  0001 C CNN
F 3 "~" H 7750 1675 50  0001 C CNN
	1    7750 1675
	0    1    1    0   
$EndComp
$Comp
L Device:C C4
U 1 1 5F05D4CC
P 7750 1525
F 0 "C4" V 7700 1625 50  0000 C CNN
F 1 "0.1uF" V 7700 1375 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 7788 1375 50  0001 C CNN
F 3 "~" H 7750 1525 50  0001 C CNN
	1    7750 1525
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 1525 7600 1675
Connection ~ 7600 1675
$Comp
L Switch:SW_Push SW2
U 1 1 5F05D4D4
P 8150 1675
F 0 "SW2" H 8150 1960 50  0000 C CNN
F 1 "PIN_RT" H 8150 1869 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 8150 1875 50  0001 C CNN
F 3 "~" H 8150 1875 50  0001 C CNN
	1    8150 1675
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 1675 7900 1525
Wire Wire Line
	7600 1675 7550 1675
Wire Wire Line
	8350 1675 8475 1675
Wire Wire Line
	7900 1775 8700 1775
Connection ~ 7900 2125
Wire Wire Line
	7900 2125 7950 2125
$Comp
L Device:R R6
U 1 1 5F066F35
P 7750 2225
F 0 "R6" V 7700 2075 50  0000 C CNN
F 1 "100k" V 7745 2225 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 2225 50  0001 C CNN
F 3 "~" H 7750 2225 50  0001 C CNN
	1    7750 2225
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7600 2225 7600 2125
$Comp
L Device:R R5
U 1 1 5F066F3C
P 7750 2125
F 0 "R5" V 7800 2225 50  0000 L CNN
F 1 "10k" V 7750 2050 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 2125 50  0001 C CNN
F 3 "~" H 7750 2125 50  0001 C CNN
	1    7750 2125
	0    1    1    0   
$EndComp
$Comp
L Device:C C5
U 1 1 5F066F42
P 7750 1975
F 0 "C5" V 7700 2075 50  0000 C CNN
F 1 "0.1uF" V 7700 1825 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 7788 1825 50  0001 C CNN
F 3 "~" H 7750 1975 50  0001 C CNN
	1    7750 1975
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 1975 7600 2125
Connection ~ 7600 2125
$Comp
L Switch:SW_Push SW3
U 1 1 5F066F4A
P 8150 2125
F 0 "SW3" H 8150 2410 50  0000 C CNN
F 1 "PIN_LT" H 8150 2319 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 8150 2325 50  0001 C CNN
F 3 "~" H 8150 2325 50  0001 C CNN
	1    8150 2125
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 2125 7900 1975
Wire Wire Line
	7600 2125 7550 2125
Wire Wire Line
	8350 2125 8475 2125
Wire Wire Line
	7900 2225 8700 2225
Connection ~ 7900 2575
Wire Wire Line
	7900 2575 7950 2575
$Comp
L Device:R R8
U 1 1 5F066F56
P 7750 2675
F 0 "R8" V 7700 2525 50  0000 C CNN
F 1 "100k" V 7745 2675 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 2675 50  0001 C CNN
F 3 "~" H 7750 2675 50  0001 C CNN
	1    7750 2675
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7600 2675 7600 2575
$Comp
L Device:R R7
U 1 1 5F066F5D
P 7750 2575
F 0 "R7" V 7800 2675 50  0000 L CNN
F 1 "10k" V 7750 2500 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 2575 50  0001 C CNN
F 3 "~" H 7750 2575 50  0001 C CNN
	1    7750 2575
	0    1    1    0   
$EndComp
$Comp
L Device:C C6
U 1 1 5F066F63
P 7750 2425
F 0 "C6" V 7700 2525 50  0000 C CNN
F 1 "0.1uF" V 7700 2275 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 7788 2275 50  0001 C CNN
F 3 "~" H 7750 2425 50  0001 C CNN
	1    7750 2425
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 2425 7600 2575
Connection ~ 7600 2575
$Comp
L Switch:SW_Push SW4
U 1 1 5F066F6B
P 8150 2575
F 0 "SW4" H 8150 2860 50  0000 C CNN
F 1 "PIN_cR" H 8150 2769 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 8150 2775 50  0001 C CNN
F 3 "~" H 8150 2775 50  0001 C CNN
	1    8150 2575
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 2575 7900 2425
Wire Wire Line
	7600 2575 7550 2575
Wire Wire Line
	8350 2575 8475 2575
Wire Wire Line
	7900 2675 8700 2675
Connection ~ 7900 3025
Wire Wire Line
	7900 3025 7950 3025
$Comp
L Device:R R10
U 1 1 5F066F77
P 7750 3125
F 0 "R10" V 7700 2975 50  0000 C CNN
F 1 "100k" V 7745 3125 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 3125 50  0001 C CNN
F 3 "~" H 7750 3125 50  0001 C CNN
	1    7750 3125
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7600 3125 7600 3025
$Comp
L Device:R R9
U 1 1 5F066F7E
P 7750 3025
F 0 "R9" V 7800 3125 50  0000 L CNN
F 1 "10k" V 7750 2950 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 3025 50  0001 C CNN
F 3 "~" H 7750 3025 50  0001 C CNN
	1    7750 3025
	0    1    1    0   
$EndComp
$Comp
L Device:C C7
U 1 1 5F066F84
P 7750 2875
F 0 "C7" V 7700 2975 50  0000 C CNN
F 1 "0.1uF" V 7700 2725 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 7788 2725 50  0001 C CNN
F 3 "~" H 7750 2875 50  0001 C CNN
	1    7750 2875
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 2875 7600 3025
Connection ~ 7600 3025
$Comp
L Switch:SW_Push SW5
U 1 1 5F066F8C
P 8150 3025
F 0 "SW5" H 8150 3310 50  0000 C CNN
F 1 "PIN_cL" H 8150 3219 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 8150 3225 50  0001 C CNN
F 3 "~" H 8150 3225 50  0001 C CNN
	1    8150 3025
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 3025 7900 2875
Wire Wire Line
	7600 3025 7550 3025
Wire Wire Line
	8350 3025 8475 3025
Wire Wire Line
	7900 3125 8700 3125
Connection ~ 7900 3475
Wire Wire Line
	7900 3475 7950 3475
$Comp
L Device:R R12
U 1 1 5F066F98
P 7750 3575
F 0 "R12" V 7700 3425 50  0000 C CNN
F 1 "100k" V 7745 3575 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 3575 50  0001 C CNN
F 3 "~" H 7750 3575 50  0001 C CNN
	1    7750 3575
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7600 3575 7600 3475
$Comp
L Device:R R11
U 1 1 5F066F9F
P 7750 3475
F 0 "R11" V 7800 3575 50  0000 L CNN
F 1 "10k" V 7750 3400 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 3475 50  0001 C CNN
F 3 "~" H 7750 3475 50  0001 C CNN
	1    7750 3475
	0    1    1    0   
$EndComp
$Comp
L Device:C C8
U 1 1 5F066FA5
P 7750 3325
F 0 "C8" V 7700 3425 50  0000 C CNN
F 1 "0.1uF" V 7700 3175 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D10.5mm_W5.0mm_P5.00mm" H 7788 3175 50  0001 C CNN
F 3 "~" H 7750 3325 50  0001 C CNN
	1    7750 3325
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 3325 7600 3475
Connection ~ 7600 3475
$Comp
L Switch:SW_Push SW6
U 1 1 5F066FAD
P 8150 3475
F 0 "SW6" H 8150 3760 50  0000 C CNN
F 1 "PIN_cD" H 8150 3669 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 8150 3675 50  0001 C CNN
F 3 "~" H 8150 3675 50  0001 C CNN
	1    8150 3475
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 3475 7900 3325
Wire Wire Line
	7600 3475 7550 3475
Wire Wire Line
	8350 3475 8475 3475
Wire Wire Line
	7900 3575 8700 3575
Text GLabel 7550 1675 0    50   Output ~ 0
PIN_RT
Text GLabel 7550 2125 0    50   Output ~ 0
PIN_LT
Text GLabel 7550 2575 0    50   Output ~ 0
PIN_cR
Text GLabel 7550 3025 0    50   Output ~ 0
PIN_cL
Text GLabel 7550 3475 0    50   Output ~ 0
PIN_cD
Text GLabel 7550 3925 0    50   Output ~ 0
PIN_cU
$Comp
L power:+3.3V #PWR08
U 1 1 5F0BF2CC
P 8475 1150
F 0 "#PWR08" H 8475 1000 50  0001 C CNN
F 1 "+3.3V" H 8490 1323 50  0000 C CNN
F 2 "" H 8475 1150 50  0001 C CNN
F 3 "" H 8475 1150 50  0001 C CNN
	1    8475 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5F0C07E2
P 8700 1150
F 0 "#PWR09" H 8700 900 50  0001 C CNN
F 1 "GND" H 8705 977 50  0000 C CNN
F 2 "" H 8700 1150 50  0001 C CNN
F 3 "" H 8700 1150 50  0001 C CNN
	1    8700 1150
	-1   0    0    1   
$EndComp
Wire Wire Line
	8475 1225 8475 1150
Wire Wire Line
	8700 1150 8700 1325
Wire Wire Line
	10500 1325 10500 1775
Connection ~ 10500 1325
Wire Wire Line
	10500 2225 10500 1775
Connection ~ 10500 1775
Wire Wire Line
	10500 2225 10500 2675
Connection ~ 10500 2225
Wire Wire Line
	10500 2675 10500 3125
Connection ~ 10500 2675
Wire Wire Line
	10500 3125 10500 3575
Connection ~ 10500 3125
Wire Wire Line
	10500 4025 10500 3575
Connection ~ 10500 3575
Wire Wire Line
	10275 3925 10275 3475
Wire Wire Line
	10275 3475 10275 3025
Connection ~ 10275 3475
Wire Wire Line
	10275 3025 10275 2575
Connection ~ 10275 3025
Wire Wire Line
	10275 2575 10275 2125
Connection ~ 10275 2575
Wire Wire Line
	10275 2125 10275 1675
Connection ~ 10275 2125
Wire Wire Line
	10275 1675 10275 1225
Connection ~ 10275 1675
Connection ~ 10275 1225
Wire Wire Line
	8700 1325 8700 1775
Connection ~ 8700 1325
Wire Wire Line
	8700 1775 8700 2225
Connection ~ 8700 1775
Wire Wire Line
	8700 2675 8700 2225
Connection ~ 8700 2225
Wire Wire Line
	8700 2675 8700 3125
Connection ~ 8700 2675
Wire Wire Line
	8700 3125 8700 3575
Connection ~ 8700 3125
Wire Wire Line
	8700 3575 8700 4025
Connection ~ 8700 3575
Wire Wire Line
	8475 3925 8475 3475
Wire Wire Line
	8475 3475 8475 3025
Connection ~ 8475 3475
Wire Wire Line
	8475 3025 8475 2575
Connection ~ 8475 3025
Wire Wire Line
	8475 2575 8475 2125
Connection ~ 8475 2575
Wire Wire Line
	8475 2125 8475 1675
Connection ~ 8475 2125
Wire Wire Line
	8475 1675 8475 1225
Connection ~ 8475 1675
Connection ~ 8475 1225
Text GLabel 9350 1225 0    50   Output ~ 0
PIN_dR
Text GLabel 9350 1675 0    50   Output ~ 0
PIN_dL
Text GLabel 9350 2125 0    50   Output ~ 0
PIN_dD
Text GLabel 9350 2575 0    50   Output ~ 0
PIN_dU
Text GLabel 9350 3025 0    50   Output ~ 0
PIN_Z
Text GLabel 9350 3475 0    50   Output ~ 0
PIN_B
Text GLabel 9350 3925 0    50   Output ~ 0
PIN_A
$Comp
L Connector_Generic:Conn_02x07_Counter_Clockwise #J1
U 1 1 5EF55D37
P 1650 7050
F 0 "#J1" H 1700 7567 50  0000 C CNN
F 1 "Button Inputs" H 1700 7476 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x07_P2.54mm_Vertical" H 1650 7050 50  0001 C CNN
F 3 "~" H 1650 7050 50  0001 C CNN
	1    1650 7050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 5EF57A42
P 9550 1325
F 0 "R16" V 9500 1175 50  0000 C CNN
F 1 "100k" V 9545 1325 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 1325 50  0001 C CNN
F 3 "~" H 9550 1325 50  0001 C CNN
	1    9550 1325
	0    -1   -1   0   
$EndComp
Wire Notes Line
	7050 850  10625 850 
Wire Notes Line
	10625 850  10625 4150
Wire Notes Line
	10625 4150 7050 4150
Wire Notes Line
	7050 4150 7050 850 
Text Notes 7050 825  0    50   ~ 0
Push Button Debounce Wiring Example
Wire Notes Line
	3025 6650 6525 6650
Wire Notes Line
	6525 6650 6525 7475
Wire Notes Line
	6525 7475 3025 7475
Wire Notes Line
	3025 7475 3025 6650
Text Notes 3025 6625 0    50   ~ 0
Connector Pin Headers
Wire Notes Line
	2400 850  5775 850 
Wire Notes Line
	5775 850  5775 4500
Wire Notes Line
	5775 4500 2400 4500
Wire Notes Line
	2400 4500 2400 850 
Text Notes 2400 825  0    50   ~ 0
Microcontroller
Wire Notes Line
	850  6450 2425 6450
Wire Notes Line
	2425 6450 2425 7475
Wire Notes Line
	2425 7475 850  7475
Wire Notes Line
	850  7475 850  6450
Text Notes 850  6425 0    50   ~ 0
Alternative Input Pin Headers
$EndSCHEMATC