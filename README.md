Marlin
======

Marlin Firmware with Delta autocalibration and various other updates/fixes

This version has a much improved auto-calibration function which will hopefully be able to cope with many more printer configuration errors than previous versions were capable of resolving.
The auto-calibration can now automatically adjust the following printer configuration variables:

·	Delta Radius
·	Diagonal Rod Length
·	Software Endstop Offsets
·	Z-Height Correction
·	Tower Position Error Correction (independent Radius and Radial position adjustment for each tower)
All of these values can also be changed manually if desired using the M666 command .

The following G-Code commands are have been added to the standard marlin firmware:
G30	This command is used to perform reporting and auto-calibration of a delta printer and has several options, as follows:
G30	Probe bed and produce a report of the current state of the printer, e.g.:
Z-Tower		Endstop Offsets
	-0.0125			X:-3.05 Y:-1.83 Z:-2.69
-0.0000		-0.0000		Tower Position Adjust
	-0.0625			A:-0.04 B:0.05 C:-0.01
-0.0375		-0.0250		I:0.25 J:-1.25 K:-0.37
	-0.0250			Delta Radius: 109.5965
X-Tower	Y-Tower	Diag Rod: 224.5935
This option does not change any settings, but is useful when manually calibrating a printer, using the M666 command to change values.
G30 Xnn Ynn	Probe bed at specified X,Y point and show z-height and delta carriage positions, e.g.:
Bed Z-Height at X:30.00 Y:30.00 = 0.0000
Carriage Positions: [176.40, 207.77, 209.52]
G30 A	Start auto-calibration. This will attempt to calibrate the printer,  adjusting all 
	parameters automatically, and will repeat the bed probing sequence show above 
	several times adjusting each time until calibration is complete. 
	It is recommended that you use M502 to load default values and then M500 to save 
	them prior to starting the auto-calibration.

     	To save the settings after the auto-calibration is complete, use the M500 command.
M666 L		List all current configuration values , e.g.:
Current Delta geometry values:
X (Endstop Adj): -3.05
Y (Endstop Adj): -1.83
Z (Endstop Adj): -2.69
P (Z-Probe Offset): X0.00 Y10.00 Z-5.60
A (Tower A Position Correction): -0.04
B (Tower B Position Correction): 0.05
C (Tower C Position Correction): -0.02
I (Tower A Radius Correction): 0.25
J (Tower B Radius Correction): -1.25
K (Tower C Radius Correction): -0.37
R (Delta Radius): 109.60
D (Diagonal Rod Length): 224.59
H (Z-Height): 255.73
All of these values can also be adjusted using the M666 command, e.g. to set the delta radius to 200mm, use:
M666 R200
Or to change the Z-Height to 350.5 mm:
M666 H350.5
Commands can also be combined, e.g. to set endstop values:
M666 X-2.04 Y-1.02 Z-1.52
All of these values can be saved/loaded to/from EEPROM using standard M500/M501 G-Code commands (to save the settings at any time just type M500). This makes manual configuration of a printer much easier as there is no longer a requirement to edit the configuration.h file and re-upload firmware for each time a change needs to be made.

Configuration.h  now includes the following additional parameters:
Set start and end locations used to deploy the Z-Probe:

#define Z_PROBE_DEPLOY_START_LOCATION {20, 96, 30, 0}
#define Z_PROBE_DEPLOY_END_LOCATION {5, 96, 30, 0}
#define Z_PROBE_RETRACT_START_LOCATION {49, 84, 20, 0}         
#define Z_PROBE_RETRACT_END_LOCATION {49, 84, 1, 0}
Set precision for autocalibration G30 function – calibration will complete when this value is reached – all probed point have to be at 0 +/- 0.015mm (for 0.03 setting below)

#define AUTOCALIBRATION_PRECISION 0.03 // mm  

Set distance to probe bed at for G30 function
 
#define BED_DIAMETER 170 // mm
           
