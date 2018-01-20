[<img src="us.png" height="30">](README.md)
[<img src="de.png" height="30">](README.de_DE.md)

# Inofficial modification of the RF firmware for RF1000 and RF2000 devices
Based on Repetier-Firmware - the fast and user friendly firmware.

## Disclaimer

THIS VERSION IS AN UNOFFICIAL MODIFICATION OF mhiers UNOFFICIAL MODIFICATION OF THE ORIGINAL FIRMWARE. It is neither endorsed nor
supported by the developers of the RF1000 firmware.  
USE AT YOUR OWN RISK.

For the official version, have a look at the upstream repository:
https://github.com/RF1000/Repetier-Firmware (see branch development)

## HowTo Install

- Download the Firmware `Branch: community_development` and unzip all the files.  
- Install Arduino.cc 1.6.5 or 1.8.5 and later if it is not installed on your computer already.  
- Edit and save Configuration.h @Line46 and 47 according to your printers model. You have to remove the two **//** in front of the printers name you wish to activate:  
`#define MOTHERBOARD                         DEVICE_TYPE_RF1000` or  
`#define MOTHERBOARD                         DEVICE_TYPE_RF2000`
- Optional: Edit and save the Configuration.h and RF1000.h or RF2000.h to your needs in case you use special specialized hardware.
- Connect your Printer with a USB-cable and switch it on.
- Go to and doubleclick /Repetier/Repetier.ino
- Within the cyan colored Arduino-IDE you have to choose some preconfiguration settings for your RFx000 Board at the menu **Tools->**:  
Board: `Arduino Mega 2560 or Mega ADK`,  
Processor: `ATMega 2560 (Mega 2560)`,  
Port: Your printers Port like `COM3`, if successfully connected.
- **Check** and **Upload** the Firmware to your Printer.

## compiled Hex-Files / automatic build system

You want to get precompiled firmware images with stock settings? Visit this website https://jenkins.beta-centauri.de/view/Repetier/ . You will find two versions, one is "stable" the other one is marked as "development".
Find more information and discussion here: http://www.rf1000.de/viewtopic.php?f=7&t=2057
Using linux you can flash the .hex-files to your printer with this command:
```avrdude -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -D -Uflash:w:Repetier.hex:i``` 

## If you upgrade to this Version from 1.37r or earlier please do a fresh M303 PID-Autotune on all heaters!  
- RF2000/RF1000 Extruder left: Menu -> Configuration -> Temperatures -> Extruder 0 -> PID Pessen-Rule  
- RF2000 Extruder right:       Menu -> Configuration -> Temperatures -> Extruder 1 -> PID Pessen-Rule  
- RF2000/RF1000 Heated Bed:    Menu -> Configuration -> Temperatures -> Heated Bed -> PID Tyreus-Lyben  
Start with an integral limit with EEPROM-values of `PID I drive min = 30` and `PID I drive max = 120`  
- Menu -> Configuration -> Temperatures -> (...) -> I-drive min = 20 .. 30  
- Menu -> Configuration -> Temperatures -> (...) -> I-drive max = 100 .. 130  

## Version RF.01.37mod 

http://www.rf1000.de/viewtopic.php?f=67&t=2043 (Thread to Stable 1.37v8 / 18.10.2017)
http://www.rf1000.de/viewtopic.php?f=74&t=1674 (Nibbels/Wessix SenseOffset-Thread)  
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
M3900               - Scan the heatbeds offset and correct the active zMatrix within the RAM of the Printer. (X-Y-Pos, Learning-Factor, linear distance weight.)  
RF2000: Z_OVERRIDE_MAX = 1.0mm, instead of 0.5mm  
RF1000: Z_OVERRIDE_MAX = 1.0mm, instead of 0.5mm  

_by StarTony_:  
Fan-Speed-Patch, see http://www.rf1000.de/viewtopic.php?f=7&t=1638  

_by Wessix_:  
M355 Sx - Turn on and off, or switch Port X19 to controll lights etc. (24V-MOSFET @Connector X19)  
M355    - Switch Port X19 to controll lights etc. (24V-MOSFET @Connector X19)  
Idea to implement the SensiblePressure-Function, see M3909  
Idea to send the active digits with the printTemperature-Status.

_by Nibbels_:  
M3900 Xn Ym Px Sy   - Scan the heatbeds offset and correct the active zMatrix within the RAM of the Printer. (X-Y-Pos, Learning-Factor, linear distance weight.)  
M3901 Xn Ym Px Sy   - to preconfigure M3900 (X-Y-Pos, Learning-Factor, linear distance weight.)  

M3902 R1            - to fix a single HeatBeds Hole within the HBS Matrix.  
M3902 Zn.n          - to add an Offset to the Matrix. n.n = {-0.2 .. 0.2} [mm]  
M3902 Z0            - to shift your active zOffset to the zMatrix. The M3006 zOffset will be zero afterwards. The zMatrix is altered within Ram and might be saved to EEPROM[n] with M3902 S[n] afterwards.  
M3902 Sn            - to save the active Matrix to position n = {1..9}  
M3902 Z0 S1         - to shift the zOffset to your zMatrix and save the Matrix at position 1. This is an example to show that the options of M3902 can be combined.  
M3911 S6000 P9000 E-30 - to limit extrusion when getting high digits because of overextrusion (Example: -10% per 1000 digits starting from 6000 digits)  
M3911 S6000 P9000 F-30 - to limit speed when getting high digits (Example: -10% per 1000 digits starting from 6000 digits) M3911 is adjustable at runtime within menus Configuration -> DMS Features -> Digit Flow CMP (No EEPROM-support!)  
M3939 Fn St1 Pt2 Ex Iy Rm   - to messure a curve of your Filaments velocity over digits -> viscosity.  

M3920 Sb - to go into or switch back from SilentMode (This will lower your steppers current to an alternative profile)  
Removed all compiler warnings and errors within the original Firmware.  
Included some Fixes the original developers of the branch "repetier/Repetier-Firmware" committed to their firmware.  
Upgraded the Firmware to the latest RF.01.37 (2017-01-20)  
Activated the X35 Temperature-Sensor for RF2000.  
Listed all spare pins for RF1000 or RF2000.  
Fixed the RF2000's Status Message length to 20chars.  
Fix for M3117
Third Z-Scale Mode: G-Code @ Configuration->Z-Configuration->Z-Scale.
(+ massive bug elimination on several places within the firmwares code)

_by Nibbels/Wessix_:  
M3909 Pn Sm         - See "Nibbels/Wessix SenseOffset"  

## mhier Z-Offset-Scan
* M3900 Xn Ym Sy Px             - Run a Z-Offset Scan at the specified Location.

Preconfiguration Options for M3900:  
* M3901 Xn Ym Sy Px 

[X]/[Y] specifies the Location of the Scan within the original HBS-Locations.  
X = {1..9}, {0 = random}
Y = {2..12}, {0 = random}
With two extruders X=1 is allowed otherwise forbidden. Normally X=2 is the lowest possible scan position. This limit is caused by border values within every zMatrix. You cannot set it wrong, the upper and lower limits are limited and autoadjusted by the firmware if set wrong.  

[S] specifies the learning rate of M3900  
S= {100}  
The measured offset will be added/substracted by 100% of its Value from/to the original matrix. The original matrix is the matrix stored in your EEPROM. Thus the matrix will be reloaded from the EEPROM to the RAM prior to the following z-offset scan.
S= {0..99}  
the Matrix in RAM will be adjusted by 0% to 99% of the messured offset. This is a great feature for multiple incremental corrections, in case you would not want to start allover or you cannot trust your first values 100% (you want to sum up corrections from different scanning-locations).

[P] ZOS learning linear distance weight  
P = {0..100}  
This is a configuration option to commit a scans offset to its surrounding area only.  
If you scan the middle of the heated bed and then all the corners with a high distance weight, this feature will behave like autobed leveling of your zMatrix.
Example: Put P to a high value and messure all corners. Then the matrix is somehow "bended" and not constantly updated. (Might work very well on heat-beds which tend to lift edges within different temperature ranges or to correct tilting).

You can activate the mhier Z-Offset-Scan within ->Configuration->Z-Configuration->Z-Offset Scan within your printers menu structure. This will always use the preconfigured or changed (see M3901) settings within your printers RAM. These M3900/M3901-Settings are not stored within the printers EEPROM.  

## Nibbels/Wessix SenseOffset  
* M3909 Pn Sm  
P = max. digits = {1...14999} [digits]  
S = max. SenseOffset = {1...300} std.: 180 [um]  

Feature called "Nibbels/Wessix SenseOffset". This is a digit limiter which decreases the printers digits by lowering the heatbed. It compensates the thermal expansion of some second order printer parts.  
 The printer will automaticly release Pressure inbetween the Nozzle and the HeatBed, whenever the digits rise atop [max.digits] by adjusting the Z-Offset. 
 The adjustment-offset is fixed to positive values (bed does never go closer to the nozzle than without M3909). The adjustment will not violate the "max. SenseOffset" restriction in order to avoid unwanted offsets (not caused by too close nozzle).  
 When choosing "max. SenseOffset" you should already know how much digits your printer normally has (Same Material, same Temperatures, same Speed, same Nozzle) and then add some plus 20% or plus 1000digits of force-tolerance.  
 Senseoffset can have two states. Active controling and idle. It is idle, whenever Z is not within negative and ~0.3mm height. You can tune this height limit by tweaking M3007 (http://www.rf1000.de/wiki/index.php/GCodes#M3007_-_Minimale_H.C3.B6he_f.C3.BCr_Z-Kompensation_festlegen). Whenever there is active control you see a circumflex "^" within the modmenu, instead of the @-Symbol.
 ![ ](https://downfight.de/picproxy.php?url=https://downfight.de/grafiken/dimi/modmenudisplay_eng.png "Angebratenes ABS")  
 When the z-compensation gets deactivated the senseoffset-function is deactivated as well.  
 Use "M3909 P0" for manual shutdown of the feature, but normally this is not necessary.  
 You cannot activate the Feature if zCompensation is not active already.  
 
 While Printing you can adjust or activate this feature with the "Left"- and "Right"-Button, while being in the "Mod-Menu" which is page 2 on the printers display.  

## SensibleViscosity  
* M3939 Fn St1 Pt2   Ex Iy Rm  
S = Starting Temperature for the Test in [°C]  
P = Final Temperature for the Test in [°C]  
F = max. test digits = 10000 or {1...12000} [digits]  
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
 
## Configurable emergency zStop and emergency pause digits within menu and EEPROM  
Menü -> Configuration -> General  

## Configurable motorcurrent within menu and EEPROM  
You can easily change the printers stepper currents within menu for more silence. This features is a replacement to our old silence-feature by gcode. EEPROM support included. The settings are restored after a reboot.
Menü -> Configuration -> Stepper  

## Configurable frequency limit to shift to double stepping / quad stepping within menu and EEPROM  
You can easily improve your printers speed when driving ~43mm/s till 80mm/s. We changed this setting from 12000 = 79mm/s to 6500 = 43mm/s to achive better performance and a more precise interrupt timing due to an earlier shift to doublesteps. We included the configuration option to give full controll to you. Double and Quadstepping works like gear shifting. Pulses are packed and sent in a row to save calculation power at high speeds. The frequency tells the printer when to shift up/down. Quad steps are sent when reaching two times the set frequency and higher speeds.
See http://www.rf1000.de/viewtopic.php?f=70&t=1986 also for more details. EEPROM support included. The settings are restored after a reboot.  
Menü -> Configuration -> Stepper :: DblFq = 6500 or {5000...12000}  

## Configurable temperature settings within menu and EEPROM  
PID-Autotune, other controlparameters and the sensortype is adjustable within the menu.  
Menü -> Configuration -> Temperatures -> Extruder0/Extruder1/Heizbett -> Sensortype: {1=Pico/Reprap, 3=EPCOS G550, 8=104-GT2, 14=3950-100k}  
In case you need other sensor types see RF1000.h / RF2000.h and edit it within EEPROM.

## Dualhead Tip-Down Support (beta)
* M3919 [S]mikrometer - Testfunction for Dip-Down-Hotend @ T1: T1 can now be springloaded and the bed will be adjusted "down" whenever T1 is selected. This is alike Ultimaker 3 does it with the right hotend.   
Example: M3919 T1 Z-0.6 tells the Printer that the right hotend will reach 0.6mm more down than the left hotend. Test it :)
* EEPROM Support for offset

## Feature Digit-Z-Kompensation (beta)
If you apply force to the DMS sensors the bed is adjusted (if Z-Compensation is active). This corrects the error which bending of the sensors apply to your layers. The physical change of the beds level is approximately +0,01mm for +1000 applied force (digits).

## RF2000: Additional Temperature Sensor for heated chamger  
This optional 3rd temperature C is automatically sent out with the other Temperatures and Digits.  
Example: ``` 22:09:24.271: T:21.7 /0 @:0 B:20.6 /0 B@:0 T0:21.7 /0 @0:0 T1:23.2 /0 @1:0 C:21.5 /0 C@:0 F:-173 /0 @:0 ```  
See RESERVE_ANALOG_SENSOR_TYPE in RF2000.h for configuration options.  
See RESERVE_ANALOG_INPUTS to find the involved code within the firmware.  
You will need to install and additional cable and a temperature sensor at X35 on your printers Board. Then you can wire your sensor to some place needed.  
This sensor might keep track of your boards temperature or the air-temperature within your enclosure.  

## Advanced profiles for PID Autotune

Additions to GCode M303:
[J0] Classic PID  
[J1] Pessen Integral Rule (suggested for Hotend)  
[J2] Some Overshoot  
[J3] No Overshoot  
[J4] Tyreus-Luyben (suggested for Heated Bed)  
[Rn] Configurable Autotune cycles  
Easy autotune support via printers menu.  
Siehe auch http://www.rf1000.de/viewtopic.php?f=7&t=1963  

## advanced Pause while printing
Temperature drop while paused.  
Lower motor current on both extruders while paused to prevent unnecessary heat to the extruder and filament.  
Manual extrusion when paused resets automatic retract for continue.

## Wessix`s help video:
[![ScreenShot](https://downfight.de/picproxy.php?url=http://image.prntscr.com/image/d7b7fade0c7343eeb67b680339478894.png)](http://youtu.be/iu9Nft7SXD8)

## !! 12.11.2017: Project is Work in Progress and untested changes are possible.