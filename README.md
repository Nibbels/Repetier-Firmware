# Inofficial modification of the RF firmware for RF1000 and RF2000 devices
Based on Repetier-Firmware - the fast and user friendly firmware.

## Disclaimer

THIS VERSION IS AN UNOFFICIAL MODIFICATION OF mhiers UNOFFICIAL MODIFICATION OF THE ORIGINAL FIRMWARE. It is neither endorsed nor
supported by the developers of the RF1000 firmware or by the developer of the original "Z-Offset-Scan".  
USE AT YOUR OWN RISK.

For the official version, have a look at the upstream repository: https://github.com/RF1000/Repetier-Firmware  
For the mhier version, have a look at the upstream repository: https://github.com/mhier/Repetier-Firmware


## Version RF.01.37mod 

See:  
http://www.rf1000.de/viewtopic.php?f=7&t=1504#p14882 (mhier Mod)  
http://www.rf1000.de/viewtopic.php?f=7&t=1504&start=60#p16397 (added feature)  
http://www.rf1000.de/viewtopic.php?f=7&t=1610 (rf1000s developement version thread to 1.35)  
http://www.rf1000.de/viewtopic.php?f=7&t=1665 (rf1000s developement version thread to 1.37)  
https://github.com/Nibbels/Repetier-Firmware/commits/heat_bed_z_offset_scan-%26-heat_bed_decrease (all the commits)

## Main Features of this Mod:
* mhier Z-Offset-Scan
* Nibbels/Wessix SensiblePressure Adjustment

## List of Features and additional G-Codes for RF2000 and RF1000

_by mhier_:  
M3900	- Scan the heatbeds offset and correct the active zMatrix within the RAM of the Printer. See M3901 for Configuration of M3900  
Z_OVERRIDE_MAX = 1.0mm, instead of 0.5mm  

_by StarTony_:  
Fan-Speed-Patch, see http://www.rf1000.de/viewtopic.php?f=7&t=1638  

_by Wessix_:  
M355 Sx - Turn on and off, or switch Port X19 to controll lights etc. (24V-MOSFET @Connector X19)  
M355    - Switch Port X19 to controll lights etc. (24V-MOSFET @Connector X19)  
Idea to implement the SensiblePressure-Function, see M3909  

_by Nibbels_:  
Disabled Milling-Mode! If you need it, activate it within the configuration files.  
M3901 Xn Ym Px Sy	- to configure M3900 (X-Y-Pos, Learning-Factor, linear distance weight.)  
M3902 R1			- to fix a single HeatBeds Hole within the HBS Matrix.  
M3902 Zn.n			- to add an Offset to the Matrix. n.n = {-0.2 .. 0.2} [mm]  
M3902 Sn 			- to save the active Matrix to position n = {1..9}  
M3903				- to configure a very slow heat bed temperature decrease  
Removed all Compilerwarnings and Compilererrors within the original Firmware.  
Included some Fixes the original developers of the branch "repetier/Repetier-Firmware" committed to their firmware.  
Upgraded the Firmware to the latest RF.01.37 (2017-01-20)  

_by Nibbels_:  
M3939 Pn Ex Iy Rm	- to messure a curve of your Filaments velocity over digits -> viscosity.

_by Nibbels/Wessix_:  
M3909 Pn Sm			- See "SensiblePressure"  

## Z-Offset-Scan
* M3900				- Run a Z-Offset Scan at the specified Location.

Configuration Options for M3900:  
* M3901 Xn Ym Sy Px 

[X]/[Y] specifies the Location of the Scan within the original HBS-Locations.  
X = {0..10}  
Y = {0..13}  

[S] specifies the learning rate of M3900  
S= {100}  
the Matrix will reloaded from the EEPROM at the begin of the scan.  
S= {0..99}  
the Matrix in RAM will be adjusted by 0% to 99% of the messured Offset. This is a great feature for multiple little corrections, in case you would not want to start allover or you cannot trust your first values 100% (you want to sum up corrections from different scanning-locations).

[P] ZOS learning linear distance weight  
P = {0..100}  
This is a configuration option to commit a scans offset to its surrounding area only.  
Example: Put P to a high value and messure all corners. Then the matrix is somehow "bended" and not constantly updated. (Might work very well on heat-beds which tend to lift edges within different temperature ranges).

## SensiblePressure  
* M3909 Pn Sm  
P = max. digits = {1...14999} [digits]  
S = max. SenseOffset = {1...200} [um]  

Feature called "SensiblePressure"  
 The printer will automaticly release Pressure inbetween the Nozzle and the HeatBed, whenever the digits rise atop [max.digits] by adjusting the Z-Offset. 
 The adjustment-offset is fixed to positive values (bed does never go closer to the nozzle than without M3909). The adjustment will not violate the "max. SenseOffset" restriction in order to avoid unwanted offsets (not caused by too close nozzle).  
 When choosing "max. SenseOffset" you should already know how much digits your printer normally has (Same Material, same Temperatures, same Speed, same Nozzle) and then add some plus 20% or plus 1000digits of force-tolerance.  
 When the z-Compensation gets deactivated the SensiblePressure-function is deactivated as well.  
 Use "M3909 P0" for manual shutdown of the feature, but normally this is not necessary.  
 You cannot activate the Feature if zCompensation is not active already.  

## SensibleViscosity  
* M3939 Fn St1 Pt2   Ex Iy Rm  
S = Starting Temperature for the Test in [°C]  
P = Final Temperature for the Test in [°C]  
F = max. test digits = 1000 or {1...12000} [digits]  
E = max. test extrusion velocity = 5 or {0.05...maxStartFeedrate} [mm/s]  
optional:  
I = test velocity increment = 0.1 or {0.02...0.4} [mm/s]  
R = max. digit change needed to consider having a filled nozzle. = 800 [digits] 

Feature called "SensibleViscosity"  
 The printer will automaticly extrude into the thin air and messure the resulting force. 
 After every messurement the velocity of extrusion is increased.  
 You will get a CSV-Output within your Console-Window (Repetierserver/Repetierhost/Octoprint/...) which you can easily copy and paste to some text file. Excel can then read the Data and you can put it into some Chart with 2 Clicks.
 You might ask why you need this feature, but in case you know your Filament quite well you can compare these resulting chars to really see how your filament extrusion behaves under different circumstances or when your nozzle is or gets clogged.
 For longer heated/boiled (my) ABS: http://www.rf1000.de/viewtopic.php?f=62&t=1549&p=16617#p16617  
 ![ ](https://downfight.de/picproxy.php?url=http://image.prntscr.com/image/3508699873f7431489d1df9344110e7f.png "Boiled ABS")  
 For the general viscosity of (my) ABS: http://www.rf1000.de/viewtopic.php?f=23&t=1620&p=16613#p16605  
 ![ ](https://downfight.de/picproxy.php?url=http://image.prntscr.com/image/610de7720945474590668b20c05d7652.png "Temperature ABS")  
 That is a clogged nozzle with the same ABS:
 ![ ](https://downfight.de/picproxy.php?url=http://image.prntscr.com/image/c00ac11b3a384994b37ce8bc3cf03bd9.png "Clogged Nozzle ABS")  
 That is red PLA:
 ![ ](https://downfight.de/picproxy.php?url=http://image.prntscr.com/image/2a3253c930794afc81e4fa4d4b2a4261.png "Clogged Nozzle ABS")  

 Have fun finally seeing your Filaments behaviour over time, temp, nozzles ;)  
 
## !! 31.12.2016: -> Compile with Arduino.cc 1.6.5, otherwise the OutputObject-Command is not 100% stable at Commands::waitUntilEndOfAllMoves();
see http://www.rf1000.de/viewtopic.php?f=7&t=1610&p=16082#p16082
## !! 03.02.2017: Project is Work in Progress and untested changes are possible.
