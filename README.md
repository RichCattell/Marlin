Marlin
======

Marlin Firmware with Delta autocalibration and various other updates/fixes


includes the following updates:


-  Add/Update M666 Command to support the following parameters:

		L		- List all current parameters.
		X/Y/Z	- Endstop offset adjustment (unchanged from previous implementation).
		A/B/C	- Delta tower position adjustment. These values equate to xa/ya/xc parameters from Peter
	    	 	  Hercek's Maxima calibration worksheet (deltabot google group).
		R	- Delta radius (in mm).
		D	- Diagonal rod length (in mm).
		H	- Build height (in mm).   
   
		All of these parameters can now be saved/loaded to EEPROM using M500 / M501 commands.
		M502 will load default values for these parameters from configuration.h

-  Add G30 Bed Report / Autocalibration command, supports the following syntax:

		G30	- Home all carrages the probe bed at 7 points and produce calibration report:

                      Z-Tower                      Endstop Offsets
                      -2.0125                      X: 0.00 Y: 0.00 Z: 0.00
             -2.3250            -2.3750            Tower Position Adjust:
                      -2.0625                      A: 0.00 B: 0.00 C: 0.00                              
             -2.3000            -2.4375            Delta Radius: 108.85
                      -2.8000                      Diagonal Rod: 217.50
           X-Tower                Y-Tower          Build Height: 280.00 
                                                   

		This can be used to perform manual calibration by changing parameters with M666 command and
	        then re-doing G30 to see the result.

   		G30 A	- Perform delta autocalibration - will produce a G30 report for each stage of the calibration.

           
-  Add Z-Probe deploy / retract location variables to configuration.h:

           #define Z_PROBE_DEPLOY_START_LOCATION {20, 96, 30, 0}
           #define Z_PROBE_DEPLOY_END_LOCATION {5, 96, 30, 0}
           #define Z_PROBE_RETRACT_START_LOCATION {49, 84, 20, 0}         
           #define Z_PROBE_RETRACT_END_LOCATION {49, 84, 1, 0}  

-  Changes to configuration.h to support autocalibration function. Added variables:

           #define AUTOCALIBRATION_PRECISION 0.03 // mm  
           #define BED_DIAMETER 170 // mm   - used to calculate raidus to probe bed at for G30 


-  Prevent moves to invalid delta coordinates
           This fix prevents delta carrages from crashing into motor mounts when attempting to move to an
           invalid location that is outside of the delta printable area. Will display an error to console if 
           this is attempted.
