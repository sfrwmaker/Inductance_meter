# Inductance_meter
The inductance meter on STM32 and atmega328p (I_meter)

01/14/2020 Updated the stm32 schematics.

01/10/2020 Added sketch to debug the hardware configuration (I_meter_debug)
This program should display 4 parameters:
- "LC"  - The period (in timer ticks, 1/16 microseconds) of internal Inductance + Internal capacitor oscillator
- "LCC" - The period of Internal Inductance + Internal capacitor + Calibration capacitor oscillator
- "CAP" - The internal capacitor value, uF
- "IND" - The period of Internal Inductance + External Inductance + Internal Capacitor.
