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
L thmalmeida:HX711_A_module HX711_s1
U 1 1 61841E26
P 7600 1200
F 0 "HX711_s1" H 7600 1747 60  0000 C CNN
F 1 "HX711_A_module" H 7600 1641 60  0000 C CNN
F 2 "thmalmeida:HX711_A_module" H 7600 1700 60  0001 C CNN
F 3 "" H 7600 1050 60  0000 C CNN
	1    7600 1200
	1    0    0    -1  
$EndComp
$Comp
L thmalmeida:HX711_A_module HX711_s2
U 1 1 61844C90
P 7600 2100
F 0 "HX711_s2" H 7600 2647 60  0000 C CNN
F 1 "HX711_A_module" H 7600 2541 60  0000 C CNN
F 2 "thmalmeida:HX711_A_module" H 7600 2600 60  0001 C CNN
F 3 "" H 7600 1950 60  0000 C CNN
	1    7600 2100
	1    0    0    -1  
$EndComp
$Comp
L thmalmeida:HX711_A_module HX711_s3
U 1 1 61845A72
P 7600 3000
F 0 "HX711_s3" H 7600 3547 60  0000 C CNN
F 1 "HX711_A_module" H 7600 3441 60  0000 C CNN
F 2 "thmalmeida:HX711_A_module" H 7600 3500 60  0001 C CNN
F 3 "" H 7600 2850 60  0000 C CNN
	1    7600 3000
	1    0    0    -1  
$EndComp
$Comp
L thmalmeida:HX711_A_module HX711_s4
U 1 1 61845F6D
P 7600 3900
F 0 "HX711_s4" H 7600 4447 60  0000 C CNN
F 1 "HX711_A_module" H 7600 4341 60  0000 C CNN
F 2 "thmalmeida:HX711_A_module" H 7600 4400 60  0001 C CNN
F 3 "" H 7600 3750 60  0000 C CNN
	1    7600 3900
	1    0    0    -1  
$EndComp
$Comp
L thmalmeida:PUSH_BUTTON SW?
U 1 1 6184A26D
P 9750 2550
F 0 "SW?" H 9650 2737 39  0000 C CNN
F 1 "PUSH_BUTTON" H 9650 2662 39  0000 C CNN
F 2 "" H 9750 2550 60  0000 C CNN
F 3 "" H 9750 2550 60  0000 C CNN
	1    9750 2550
	1    0    0    -1  
$EndComp
$Comp
L thmalmeida:PUSH_BUTTON SW?
U 1 1 6184A5BB
P 9500 3100
F 0 "SW?" H 9400 3287 39  0000 C CNN
F 1 "PUSH_BUTTON" H 9400 3212 39  0000 C CNN
F 2 "" H 9500 3100 60  0000 C CNN
F 3 "" H 9500 3100 60  0000 C CNN
	1    9500 3100
	1    0    0    -1  
$EndComp
Text Label 2200 4200 2    50   ~ 0
nokia5110_LED
Wire Wire Line
	7000 1200 6900 1200
Wire Wire Line
	7000 1100 6900 1100
Text Label 6900 1200 2    50   ~ 0
HX711_s1_SCK
Text Label 5400 3300 0    50   ~ 0
HX711_s4_SCK
Text Label 6900 1100 2    50   ~ 0
HX711_s1_SDA
Text Label 5400 3200 0    50   ~ 0
HX711_s4_SDA
Text Label 5400 3100 0    50   ~ 0
HX711_s3_SCK
Text Label 5400 3000 0    50   ~ 0
HX711_s3_SDA
Wire Wire Line
	5400 3000 5300 3000
Wire Wire Line
	5300 3100 5400 3100
Wire Wire Line
	8200 1800 8300 1800
Wire Wire Line
	8200 1900 8300 1900
Wire Wire Line
	8200 2000 8300 2000
Wire Wire Line
	8200 2100 8300 2100
Text Label 6900 3000 2    50   ~ 0
HX711_s3_SCK
Text Label 5400 1800 0    50   ~ 0
HX711_s2_SCK
Wire Wire Line
	5400 1800 5300 1800
Text Label 6900 3900 2    50   ~ 0
HX711_s4_SCK
Text Label 6900 3800 2    50   ~ 0
HX711_s4_SDA
Wire Wire Line
	6900 3900 7000 3900
Wire Wire Line
	6900 3800 7000 3800
Wire Wire Line
	6900 4000 7000 4000
Wire Wire Line
	7000 3700 6900 3700
Wire Wire Line
	7000 3000 6900 3000
Wire Wire Line
	7000 3100 6900 3100
Wire Wire Line
	6900 2900 7000 2900
Wire Wire Line
	7000 2800 6900 2800
Wire Wire Line
	7000 2200 6900 2200
Text Label 6900 2100 2    50   ~ 0
HX711_s2_SCK
Text Label 6900 2000 2    50   ~ 0
HX711_s2_SDA
Wire Wire Line
	6900 2100 7000 2100
Wire Wire Line
	7000 2000 6900 2000
Wire Wire Line
	7000 1900 6900 1900
Text Label 6900 2900 2    50   ~ 0
HX711_s3_SDA
Wire Wire Line
	8200 900  8300 900 
Wire Wire Line
	8200 1000 8300 1000
Wire Wire Line
	8200 1100 8300 1100
Wire Wire Line
	8200 1200 8300 1200
Wire Wire Line
	8200 2700 8300 2700
Wire Wire Line
	8200 2800 8300 2800
Wire Wire Line
	8200 2900 8300 2900
Wire Wire Line
	8200 3000 8300 3000
Wire Wire Line
	8200 3600 8300 3600
Wire Wire Line
	8200 3700 8300 3700
Wire Wire Line
	8200 3800 8300 3800
Wire Wire Line
	8200 3900 8300 3900
Wire Wire Line
	2300 4900 2200 4900
Text Label 2200 4900 2    50   ~ 0
+3.3V
Text Label 6900 4000 2    50   ~ 0
+3.3V
Text Label 6900 3100 2    50   ~ 0
+3.3V
Text Label 6900 2200 2    50   ~ 0
+3.3V
Text Label 6900 1300 2    50   ~ 0
+3.3V
Wire Wire Line
	6900 1300 7000 1300
Text Label 2200 4800 2    50   ~ 0
nokia5110_SCE
Wire Wire Line
	2300 4800 2200 4800
Wire Wire Line
	2200 4200 2300 4200
Text Label 1800 2900 2    50   ~ 0
nokia5110_SCE
$Comp
L thmalmeida:STM32F103_MapleMini IC?
U 1 1 61855071
P 3600 2400
F 0 "IC?" H 3600 3665 50  0000 C CNN
F 1 "STM32F103_MapleMini" H 3600 3574 50  0000 C CNN
F 2 "thmalmeida:STM32F103_MapleMini" H 3650 3600 60  0001 C CNN
F 3 "" H 3350 1650 60  0000 C CNN
F 4 "M" H 3700 3850 60  0001 C CNN "Manufacturer"
F 5 "P" H 3700 3750 60  0001 C CNN "Partnumber"
	1    3600 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 2900 1800 2900
Text Label 1800 2300 2    50   ~ 0
nokia5110_SCLK
Wire Wire Line
	1800 2300 1900 2300
Wire Wire Line
	1800 2500 1900 2500
Text Label 5400 2900 0    50   ~ 0
Beep
Wire Wire Line
	5400 2900 5300 2900
Wire Wire Line
	5300 2200 5400 2200
Text Label 5400 2200 0    50   ~ 0
Tare_switch
Wire Wire Line
	1800 2600 1900 2600
Text Label 1800 2600 2    50   ~ 0
nokia5110_LED
$Comp
L thmalmeida:nokia5110 P?
U 1 1 6186976F
P 2500 4600
F 0 "P?" H 2467 3985 50  0000 C CNN
F 1 "nokia5110" H 2467 4076 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 2550 4950 50  0001 C CNN
F 3 "" H 2550 4950 50  0001 C CNN
	1    2500 4600
	1    0    0    1   
$EndComp
Text Label 2200 4300 2    50   ~ 0
nokia5110_SCLK
Wire Wire Line
	2300 4300 2200 4300
Text Label 1800 2800 2    50   ~ 0
nokia5110_RST
Wire Wire Line
	1800 2800 1900 2800
Text Label 2200 4600 2    50   ~ 0
nokia5110_RST
Text Label 2200 4700 2    50   ~ 0
nokia5110_SCE
Wire Wire Line
	2200 4700 2300 4700
Wire Wire Line
	2300 4600 2200 4600
Text Label 2200 4500 2    50   ~ 0
nokia5110_COM
Text Label 1800 2700 2    50   ~ 0
nokia5110_COM
Wire Wire Line
	1800 2700 1900 2700
Wire Wire Line
	2200 4500 2300 4500
Text Label 1800 2500 2    50   ~ 0
nokia5110_SDA
Text Label 2200 4400 2    50   ~ 0
nokia5110_SDA
Wire Wire Line
	2200 4400 2300 4400
Text Label 1800 2000 2    50   ~ 0
HX711_s1_SCK
Text Label 1800 1900 2    50   ~ 0
HX711_s1_SDA
Wire Wire Line
	1800 2000 1900 2000
Wire Wire Line
	1900 1800 1800 1800
Wire Wire Line
	1800 1900 1900 1900
Text Label 1800 1800 2    50   ~ 0
Beep
Text Label 5400 1700 0    50   ~ 0
HX711_s2_SDA
Wire Wire Line
	5400 1700 5300 1700
Wire Wire Line
	5400 3300 5300 3300
Wire Wire Line
	5300 3200 5400 3200
$EndSCHEMATC
