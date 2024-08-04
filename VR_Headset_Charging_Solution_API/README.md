## Description

This project is an API which contains libaries of the following ICs intended for single cell USB PD charging applications.


## About Firmware
The code has the following features:
1.It can toggle LEDs based on the SOC value read by the Fuel Gauge when MAX32660 on-board switch is pressed.
2.This code consists of API for MAX77958, MAX77986, MAX17262 using MAX32660 for power bank application.
3.PDO, PPS, OTG modes are supported using the code.
4.Fuel Gauge parameter reading can be done
5.MAX77958 Interrupt is connected to MAX32660 which facilates in automatic charger mode selection (Forward charging, Reverse OTG, Disabled mode) based on type of device connected (Source, Sink, Left unconnected).


## Required Connections
Hardware Connections:
1.Connect GND (GND4) of MAX77986AEVKIT to GND of MAX32660EVSYS.
2.Remove jumper 7 of the MAX77986AEVKIT.
3.Connect HDR_VIO of MAX32660EVSYS to Middle pin of J7 of MAX77986AEVKIT.
4.Connect P0.8 and P0.9 pins of MAX32660EVSYS (SCL and SDA) to the test points SCL1 and SDA1 of MAX77986AEVKIT.
5.Connect P0.11 of MAX32660EVSYS to the INTB1 test point of MAX77986AEVKIT.
6.Connect SYSPWR and SYSGND of MAX17262XEVKIT to the BATTP and BATTN points of MAX77986AEVKIT.
7.Connect a single cell battery to the PACK+ and PACK- terminals of MAX17262XEVKIT
8.Connect SCL1 and SDA1 of MAX77986AEVKIT to the SCL and SDA pins of MAX17262XEVKIT
9.Toggle the SW1 switch of MAX77986AEVKIT to the left(towards SCLM and SDAM)
10.Connect RT1- pin of MAX17262XEVKIT to the GND4 hook of MAX77986AEVKIT

LED connections to MCU: P0.1, P0.2, P0.3, P0.4 of MAX32660EVSYS to 4 LEDs

