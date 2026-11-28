[<img src="us.png" height="30">](README.md)
[<img src="de.png" height="30">](README.de_DE.md)

# Inoffizielle Modifikation der RF Firmware für RF1000 und RF2000 / RF2000v2 Geräte

Basierend auf der Repetier-Firmware - die schnelle und benutzerfreundliche Firmware  

Dieser Branch empfängt Druckaufträge über USB.

## Warnhinweis
Diese Firmware-Modifikation stellt eine Weiterentwicklung/Ergänzung von mhier's inoffizieller Firmware-Modifikation der original Conrad-Firmware dar. Sie wird vom Hersteller der RF1000 und RF2000 Geräte nicht unterstützt.  
Für etwaige Schäden, die an Ihrem Gerät entstehen könnten, kann keine Verantwortung übernommen werden.  

BENUTZUNG AUF EIGENE GEFAHR. 

Link zur offiziellen Version der Firmware:
https://github.com/RF1000/Repetier-Firmware (siehe Branch developement. Oder auch auf den Produktseiten auf www.conrad.de)  

## Installationsanleitung

- Diesen Branch (`ai-firmware-review`) herunterladen und entpacken. Dies ist ein Software-Review-Kandidat; die gesammelten Druckertests stehen noch aus.  
- Installiere die Arduino.cc IDE 1.8.12 oder neuer, wenn es nicht bereits installiert ist.  
- Man bearbeitet und speichert `Repetier/Configuration.h` passend zum Druckermodell mit einem Texteditor. Man muss die zwei **//** vor dem Druckermodell entfernen, welches man aktivieren will:  
`#define MOTHERBOARD                         DEVICE_TYPE_RF1000` or  
`#define MOTHERBOARD                         DEVICE_TYPE_RF2000` or  
`#define MOTHERBOARD                         DEVICE_TYPE_RF2000v2`  
- Dieser Stand ist für den 3D-Druck ausgelegt. Der [letzte Stand mit Fräscode](https://github.com/Nibbels/Repetier-Firmware/tree/7e2e08b7f6da7677007dfddd06f8fc5adeb7d709) liegt im Repository-Verlauf.
- Für Umbauten stehen die Einstellungen in `Configuration.h` sowie in `RF1000.h` oder im gemeinsamen `RF2000.h` für RF2000 und RF2000v2. Die [Anschlussübersicht](docs/board-connectors.md) enthält das Beispiel für einen Hotend-Lüfter an X44.
- Der Drucker wird mit dem USB-Kabel angeschlossen und angeschaltet.
- Die Datei /Repetier/Repetier.ino doppelklicken. 
- Innerhalb der türkisfarbenen Arduino-IDE muss man im Menü **Werkzeuge->** die Voreinstellungen für die RFx000-Boards prüfen oder einstellen:  
Board: `Arduino Mega 2560 or Mega ADK`,  
Prozessor: `ATMega 2560 (Mega 2560)`,  
Port: Der Port des Druckers, wie `COM3`, wenn er korrekt verbunden ist.
- **Prüfen** and **Hochladen** der Firmware zum Drucker.

## Vorbereitete Hex-Files / automatisches Build-System

Für diesen Review-Branch erzeugen die [GitHub Actions Firmware-Prüfungen](https://github.com/Nibbels/Repetier-Firmware/actions/workflows/firmware-check.yml) vier Varianten mit Arduino CLI 1.2.2 und AVR Boards 1.8.6. Die Artefakte sind sieben Tage verfügbar. Der [Build- und Cleanup-Audit](coordination/audits/final-cleanup.md) erläutert den geprüften Buildweg und die älteren Alternativen. Historische Community-Builds wurden unter https://jenkins.beta-centauri.de/view/Repetier/ veröffentlicht; sie gehören nicht zu diesem Review-Branch.
Siehe auch http://www.rf1000.de/viewtopic.php?f=7&t=2057
Unter Linux kann man diese Firmwares mit 
```avrdude -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -D -Uflash:w:Repetier.hex:i``` 
in die Drucker einspielen.

## Vorheizprofile

Unter **Configuration → Preheat profiles** stellst du Bett- und Extrudertemperatur für Profil A und B ein. Der Drucker speichert jede Änderung im internen EEPROM. Unter **Extruder** startest du das Profil anhand der angezeigten Temperaturen. Das Bett steht zuerst. Mit 0 °C schaltest du die jeweilige Heizung aus. Die Werkseinstellungen sind 60/180 °C für A und 100/210 °C für B.

## Wenn du von einer älteren Version als 1.37r.Mod oder der offiziellen Firmware bis 1.39 updatest, führe bitte einen neuen M303 PID-Autotune bei allen Heizelementen aus!  
- RF2000/RF1000 Extruder links: Menü -> Configuration -> Temperatures -> Extruder 0 -> PID Pessen-Rule  
- RF2000 Extruder rechts:       Menü -> Configuration -> Temperatures -> Extruder 1 -> PID Pessen-Rule  
- RF2000/RF1000 Heizbett:       Menü -> Configuration -> Temperatures -> Heated Bed -> PID Tyreus-Lyben  
Starte mit I-Anteil limits (EEPROM-Werte) `PID I drive min = 30` and `PID I drive max = 120`  
- Menü -> Configuration -> Temperatures -> (...) -> I-drive min = 20 .. 30  
- Menü -> Configuration -> Temperatures -> (...) -> I-drive max = 100 .. 130  

## Community-Mod Wiki - Für Feature-Beschreibungen GCodes und detailliertere Erklärung.

http://www.rf1000.de/wiki/index.php/Kategorie:CommunityMod_Firmware  

## Version RF 1.45.Mod - wichtige Threads im Forum

http://www.rf1000.de/viewtopic.php?f=67&t=2708 (Thread to Stable 1.45 / 03.01.2020)  
http://www.rf1000.de/viewtopic.php?f=67&t=2403 (Thread to Stable 1.43 / 28.11.2018)  
http://www.rf1000.de/viewtopic.php?f=67&t=2324 (Thread to Stable 1.42 / 28.07.2018)  
http://www.rf1000.de/viewtopic.php?f=67&t=2084 (Thread to Stable 1.37x7 / 26.11.2017)  
http://www.rf1000.de/viewtopic.php?f=67&t=2043 (Thread to Stable 1.37v8 / 18.10.2017)  
http://www.rf1000.de/viewtopic.php?f=74&t=1674 (Nibbels/Wessix SenseOffset-Thread)  
http://www.rf1000.de/viewtopic.php?f=7&t=1504#p14882 (mhier Mod)  
http://www.rf1000.de/viewtopic.php?f=7&t=1504&start=60#p16397 (added feature)  
http://www.rf1000.de/viewtopic.php?f=7&t=1610 (rf1000s developement version thread to 1.35)  
http://www.rf1000.de/viewtopic.php?f=7&t=1665 (rf1000s developement version thread to 1.37)  
https://github.com/Nibbels/Repetier-Firmware/commits/heat_bed_z_offset_scan-%26-heat_bed_decrease (all the commits)  

## Haupt-Funktionen dieser Modifikation:

mhier Z-Offset Scan (zMatrix Höhenkorrekturscan durch Abtasten eines Punktes auf dem Heizbetts bei Betriebstemperatur)  
Nibbels/Wessix SenseOffset (Kompensation der thermischen Nachdehnung im ersten Layer)  

## Liste der Funktionen und zusätzlichen GCODEs für den RF2000 und den RF1000

_von mhier entwickelt_:  
M3900 - Messe den Abstand/Versatz des Heizbetts und korrigiere die aktive zMatrix des Heizbettes und speichere sie im RAM Speicher.   (Angabe von X, Y Wert der zMatrix möglich, Zufallsbestimmung der Messtelle,  Lernfaktor, Lineare Gewichtung in Abhängigkeit von der Entfernung)  
Modell RF2000: Z_OVERRIDE_MAX = 1.0mm, anstelle von 0.5mm.  
Modell RF1000: Z_OVERRIDE_MAX = 1.0mm, anstelle von 0.5mm.   

_von StarTony_:  
Lüfter-Geschwindigkeits-Patch, siehe: http://www.rf1000.de/viewtopic.php?f=7&t=1638  

_von Wessix_:  
**M355 Sx** - Port X19 per Menü oder GCode an und abschalten, um dort angeschlossene Verbraucher wie z.b. die LED Leiste zu steuern (24V MOSFET @Anschluss X19)  
Die grundlegende Idee zum SenseOffset (Kompensation der thermischen Nachdehnung) (Siehe Erklärung des MCODES M3909)  
Idee den aktuellen Digit-Kraftwert zusammen mit der Status-Temperaturabfrage zum RepetierServer/Octoprint zu senden.  

_von Nibbels_:  
**M3900 Xn Ym Px Sy** - Scanne den Abstand des Heizbetts und korrigiere die aktive zMatrix innerhalb des RAM im Drucker. (Angabe von X, Y Wert der zMatrix möglich, Zufallsbestimmung der Messtelle, Lernfaktor, Lineare Gewichtung in Abhängigkeit von der Entfernung)  
**M3901 Xn Ym Px Sy** - Vorkonfigurationsbefehl für M3900, ohne den Scan zu starten.  
**M3902 R1** - um ein einzelnes Loch im Heizbett in der Heatbed-Matrix korrigieren. Der tiefste Matrixpunkt wird durch interpolation der umliegenden Punkte angehoben.  
**M3902 Zn.n** - Addiere einen Abstand/manuelles Offset zMatrix im RAM des Druckers. Die zMatrix wird innerhalb des RAMs verändert und kann danach mit dem Befehl M3902 S [n] an der Position [n] im EEPROM abgelegt werden. z.B. M3901 Z0.1 addiert ein positives Offset in die zMatrix, mit demselben Effekt wie _M3**0**0**6** Z0.1_. M3902 Z0 oder M3902 Z0.0 hat wie nachfolgend erklärt eine Sonderstellung.  
**M3902 Z0** - Verschiebe das aktuell eingestellte Z-Offset in die zMatrix im RAM. Danach ist das Z-Offset=0.00 (Siehe M3006 / Z-Offset im Menü des Druckers), das tatsächliche Druck-Offset hat sich nicht geändert (+-0).
**M3902 Z0 S1** - Kombinationsbefehl: Verschiebe das aktuell eingestellte Z-Offset in die zMatrix im RAM und speichere diese zMatrix an der Position 1 im EEPROM - Dies ist ein Beispiel um zu zeigen, dass die Optionen von M3902 kombiniert werden können.  
**M3902 Sn** - Speichere die aktuell im RAM liegende zMatrix unter der Postion n = {1..9}  
**M3911 S6000 P9000 E-30 - Bei hohen Kraftwerten wegen Überextrusion wird die Filamentförderung zurückgefahren (-10% pro 1000 digits, beginnend bei 6000 digits)  
**M3911 S6000 P9000 F-30 - Bei hohen Kraftwerten wird die Druckgeschwindigkeit eingeschränkt (Example: -10% Geschwindigkeit pro 1000 digits, beginnend bei 6000 digits) M3911 ist im Menü unter Configuration -> DMS Features -> Digit Flow CMP zur Laufzeit einstellbar (Kein EEPROM-Support!)  
**M3920 Sb** - Flüstermodus ein oder ausschalten. (Diese Funktion vermindert den Strom der Steppermotoren auf ein in der Firmware definiertes alternatives Strom-Profil MOTOR\_CURRENT\_SILENT = [110/110/90/90/90] ).  
Es wurden **alle Compilerwarnungen und Compilerfehler eliminiert**.  
Weitere (passende) **Verbesserungen/Bugfixes/Commits der Entwickler der originalen RepetierFirmware** wurden auch in diesen Mod übernommen, obwohl die original RF2000/RF1000 Firmware diese nicht übernommen hatte.  
Mit dem Stand vom 20.01.2017 wurde die Firmware auf den neuesten Firmwarestand der Original **RF.01.37** angehoben.  
Der zusätzliche X35 Temperature-Sensor wurde für den RF2000 aktiviert.  
In der pins.h wurden alle optionalen Pins für RF1000 und RF2000 aufgelistet.  
Korrektur des RF2000 Statuszeilenlimits auf 20 Zeichen.
Korrektur des M3117
Dritter Z-Scale Modus: G-Code unter Configuration->Z-Configuration->Z-Scale.
(+ Massive Bug-Elimination überall im Code)

_von Nibbels und Wessix entwickelt_:  
**M3909 Pn Sm** - siehe unten "SenseOffset" (Kompensation der thermischen Nachdehnung)  

## mhier Z-Offset-Scan

* M3900 **Xn Ym Sy Px** - Mache einen Z-Offset-Scan an einer bestimmten Position. Die Korrektur-Höhe der zMatrix wird im RAM korrigiert.  
**[X]/[Y]** legt die Position des Scans innerhalb der möglichen Heat-Bed-Scan Scan-Positionen fest.  
**X** = {1..9}, {0 = Zufallsposition X}  
**Y** = {2..12}, {0 = Zufallsposition Y}  
Mit zwei Extrudern ist X=1 ein erlaubter Wert, mit nur einem ist normalerweise X=2 die niedrigste zMatrix-Scanposition in X Ausrichtung. Diese Limitierung wird von Begrenzungen/Begrenzungswerten in jeder zMatrix verursacht. Man kann aber nichts falsch einstellen, sollten für X und Y vom Benutzer unzulässige Werte eingestellt werden, ändert die Firmware diese auf den nächstliegenden gültigen Wert.  
**[S]** legt die Lernrate fest  
**S** = {100}: Die zMatrix wird beim Beginn des Scans komplett aus dem EEPROM in den RAM des Druckers geladen. Bisherige nicht gespeicherte Änderungen werden verworfen.  
**S** = {0..99}: Die zMatrix im RAM wird von 0 - 99% Gewichtung/Penetranz des gemessenen Offsets angepasst. Dies ist eine tolle Möglichkeit mit mehreren inkrementellen Korrekturen eine insgesamt gute Korrekturquote zu erreichen. Das hilft z.B. wenn man den ersten Messwerten/Messpositionen nicht zu 100% trauen kann (Stichwort Popel). Man will letztlich die Korrekturen von Verschiedenen Positionen gemeinsam nutzen.  
**[P]** Entfernungsgewichtung  
**P** = {0..100}  
Die Entfernungsgewichtung stellt ein, dass ein, an einer definierten Position gemessener Offset, in seiner nächsten Nähe einen hohen Einfluss auf die zMatrixkorrektur hat, diese aber mit der Entfernung (Radius) abnimmt.
Man kann z.B. nach dem Aus- und Einbau des Heizbettes in der Bettmitte und anschließend an den vier Ecken des Heizbettes mit einer jeweils hohen Entfernungsgewichtung messen. Damit wirkt sich dieses Feature wie ein "Auto-Bed-Levelling" auf die zMatrix aus. Damit wird das Heizbett "geradegerichtet" ohne einen neuen HBS-Scan durchführen zu müssen.  
_Idee: Die Welligkeit des Heizbettes bleibt gleich, nur der Einbauwinkel könnte sich minimal geändert haben._  
Die zMatrix wird durch den Einsatz des Parameters [P] quasi "verbogen" und nicht mit einem konstanten Wert angehoben oder abgesenkt. (Dies Option könnte für Heizbetten bei denen sich die Ecken bei unterschiedlichen Temperaturen anheben/senken gut funktionieren um etwaige Eselsohren herauszukorrigieren)  

Der mhier Z-Offset-Scan kann im Druckermenü unter ```-> Konfiguration ->  Z-Konfiguration -> Z-Offset Scan```  
gestartet werden. In diesem Fall werden immer die vorkonfigurierten oder bereits geänderten (siehe unter M3901/letzte verwendete Einstellungen) Einstellungen aus dem RAM des Druckers verwendet. Diese M3900/M3901 Einstellungen werden nicht im EEPROM des Druckers gespeichert, sind also nach einem Neustart des Druckers wieder auf Standard eingestellt.  

* M3901 **Xn Ym Sy Px**  
Konfiguriert den mhier Z-Offset-Scan, ohne diesen auszuführen.

## SenseOffset (Kompensation der thermischen Nachdehnung)  

* M3909 Pn Sm  
**P** = maximale Digitgrenze = {1....14999} [digits]  
**S** = maximaler Mess/Korrektur-Abstand = {1...300} Standardeinstellung wenn nicht explizit definiert.: 180 [um]  

Der Drucker wird, wenn das Feature aktiviert ist, automatisch den gemessenen Filament-Einpress-Druck (positive Digits) unter der vorgegebenen Grenze **[P]** halten.  
Das gelingt, indem der Abstand zwischen Düse und Bett größerregelt/vergrößert wird, wodurch sich der entstehende Innendruck im Hotend verringert. Das Heizbett wird ausschließlich von der Düse weggeregelt, das bedeutet, dass das Bett durch dieses Feature niemals näher an die Düse heranfahren wird. 

Der Regelung wurde vorgegeben, dass der Wert **[S]** der maximale Korrektur-Abstand in [um]=Mikrometer darstellt, um den das Heizbett nach unten justiert werden darf. Erfahrungsgemäß wird die tatsächliche Nachdehnung weniger als 0.15mm / 150um betragen. Reicht das nicht, war das zOffset nicht exakt justiert. Wird ein hoher Korrekturwert (SenseOffset / sO im Druckermenü 2) ständig erreicht, sollte man die Düse auf schleichende Verstopfung prüfen, die obere Digitgrenze dem Material anpassen/erhöhen oder das Anfangs-Z-Offset genauer einstellen.

Bei der Wahl des Wertes für **[P]** sollte man Erfahrungswerte heranziehen.
Man weiß anhand vorangegangener Drucke welchen Kraftwert "digits" der Drucker (Basierend auf gleichem Material, gleichen Drucktemperaturen, gleiche Druckgeschwindigkeit, gleiche Düse) produziert.
Auf diesen Wert sollte man, je nach gewünschtem Anpressdruck der ersten Lage, bis zu ca. 20% oder 1000 Digits an Krafttoleranz dazu addieren. 
_Beispiel:_  
Der Drucker druckt ein Teil mit ABS. Der Digit-Kraft-Wert schwankt tendentiell zwischen 2300 und 2700. Ein guter Wert für [P] wäre bei diesem Slicer-Profil vermutlich ~ 3000 bis 3500.  
_Beispiel 2:_  
Der Drucker druckt ein Teil aus TPE. Vermutlich ist der ideale Wert für P kleiner als 1000, da das Material sehr flüssig sein kann.  
_Beispiel 3:_  
Der Drucker druckt PLA sehr heiß und sehr langsam. Die Digits schwanken sehr stark zwischen 1000 und 2500. Vermutlich hilft es, das PLA schneller zu drucken, dann stabilisieren sich die Digits, ein gutes Limit in der ersten Lage könnte z.B. 3000 digits sein. Anschließend kann man sein Gefühl für das Material verfeinern.

Wird aus irgendeinem Grund die Z-Kompensation ausgeschaltet wird SenseOffset ebenfalls deaktiviert.
"M3909 P0" sorgt für die manuelle Abschaltung der Funktion, dies sollte aber normalerweise nicht nötig sein. Ist die Funktion aktiv am Regeln, so ändert sich das "@" im Modmenü zu einem "^" Circumflex, daran erkennt man sehr einfach, ob senseOffset tatsächlich die Extrusionskraft überwacht. Die Funktion ist nur aktiv, wenn sich die Z-Höhe unter ~0.3mm befindet. Dieses Arbeitshöhenlimit, ab dem die Funktion inaktiv wird, entspricht dem unteren Grenzwert der Z-Kompensation. Siehe auch M3007 (http://www.rf1000.de/wiki/index.php/GCodes#M3007_-_Minimale_H.C3.B6he_f.C3.BCr_Z-Kompensation_festlegen)
![ ](https://downfight.de/picproxy.php?url=https://downfight.de/grafiken/dimi/modmenudisplay.png "Angebratenes ABS")  
Abschließend, sobald das Druckobjekt ausgegeben wird, wird M3909 inaktiv und vollständig deaktiviert.
Ohne Z-Kompensation kann SenseOffset nicht aktiviert werden, es wird sofort deaktiviert.

Bei einer manuell programmierten Startmade SenseOffset-AutoStart ausschalten und `M3909 P…` nach der letzten Bewegung der Startmade einfügen. [Reihenfolge im Startcode](docs/first-layer-control/automatic-start-line.md#manual-start-lines)

Während des Drucks kann man während des ersten Layers diese Funktion aktivieren oder das Limit ändern, indem man die "links"- und "rechts"-Navigationstasten an Bedienfeld des Druckers nutzt, solange man sich im "Mod-Menü" des Druckers befindet. (Seite 2 auf dem Display des Druckers. Seite 2 kann über die Pfeil nach unten Taste vom Hauptbildschirm aus aufgerufen werden).  

## Konfigurierbarer Z-Notaus- und Notpause-Digits in Menü und EEPROM  
Menü -> Configuration -> General  

## Konfigurierbarer Motorstrom im Menü und EEPROM  
Man kann alle Stepperströme im Menü am Drucker einstellen. Dieses Features ist ein ersatz für den alten Silent-Mode. Auch hier besteht EEPROM support, die Einstellungen werden beim nächsten Druckerstart automatisch geladen.
Menü -> Configuration -> Stepper  

## Konfigurierbare Double-/Quad-Step Freqenz im Menü und EEPROM  
Damit kann man sehr einfach die Druckergeschwindigkeit im Bereich ~43mm/s bis 80mm/s optimieren bzw. die CPU für Kleinbewegungen entlasten. Wir haben die Umschaltfrequenz von 12000 = 79mm/s auf 6500 = 43mm/s heruntergesetzt, um eine bessere Performance und ein präziseres Timing bei der Ansteuerung zu erhalten - diese Umstellung aber auch für jeden selbst konfigurierbar gemacht.
Double-/Quad-Stepping wirkt wie eine Gangschaltung. Die Pulse zum Motordriver werden gepackt in 2er oder 4er Pulspäckchen ausgegeben um Rechenleistung einzusparen. Die einstellbare Frequenz legt fest, wann auf Dual-Steps hochgeschaltet wird. Bei der doppelten Frequenz oder höher wird auf Quad-Steps umgeschaltet.
Siehe http://www.rf1000.de/viewtopic.php?f=70&t=1986 für mehr Details.  
Menü -> Configuration -> Stepper :: DblFq = 6500 oder {5000...12000}

## Konfigurierbare Temperatureinstellungen im Menü und EEPROM  
Der PID-Autotune, die weiteren Regelparameter und der Sensortyp sind im Menü einstellbar.  
Unter **Configuration → Temperatures** bietet das Menü die Sensortypen `1`, `3`, `8` und `13` für die Extruder sowie `3` und `4` für das Heizbett. Die vollständige Tabelle enthält auch die Typen für die Build-Konfiguration und das EEPROM. [Temperatursensoren und IDs](docs/board-connectors.md#temperature-sensor-ids)

## Feature Digit-Z-Kompensation (beta)
Wenn auf die DMS gedrückt wird, wird das Bett in seiner Höhe korrigiert, wenn die Z-Kompensation arbeitet. Damit wird die Durchbiegung der DMS-Sensoren unter Last korrigiert. Die Änderungen bewegen sich in etwa im Bereich von +0,01mm wenn +1000 Digits Kraft auf die Sensoren aufgebracht werden.

## RF2000: Zusätzlicher Temperatursensor für den Heizraum  
Diese Option aktiviert einen weiteren Temperatursensor C, dessen Messwerte automatisch mit der Statusabfrage ausgegeben werden.
Beispiel: ``` 22:09:24.271: T:21.7 /0 @:0 B:20.6 /0 B@:0 T0:21.7 /0 @0:0 T1:23.2 /0 @1:0 C:21.5 /0 C@:0 F:-173 /0 @:0 ```  
`RESERVE_ANALOG_SENSOR_TYPE` in `RF2000.h` wählt die Umrechnung für den Eingang `X35_ADC_CHANNEL`. [Temperatursensoren und IDs](docs/board-connectors.md#temperature-sensor-ids)

Man muss, um den Temperatursensor zu nutzen, ein zusätzliches Kabel mit einem Temperatursensor am Port X35 an der Druckerplatine des RF2000-Druckers anschliessen. Anschließend kann der optionale Sensor an einem beliebigen Punkt im Drucker positioniert werden.
Es könnte z.B. die Temperatur in der Nähe der Hauptplatine, die Hitzeentwicklung an einem Steppermotor oder die Temperatur im Druckraum gemessen werden.

## Erweiterte Profile für PID Autotune

Erweiterter GCode M303:
[J0] Classic PID  
[J1] Pessen Integral Rule (empfohlen für Hotend)  
[J2] Some Overshoot  
[J3] No Overshoot  
[J4] Tyreus-Luyben (empfohlen für Heizbett)  
[Rn] Configurable Autotune cycles  
Autotune support über das Druckermenü am Display.  
Siehe auch http://www.rf1000.de/viewtopic.php?f=7&t=1963  

## Pause Funktion 
Mit Temperaturabsenkung für das aktive Hotend während der Pause.  
Mit Motorstromabsenkung für beide Extruder-Motoren während der Pause, um eine unnötige Erwärmung des Filamentes zu verhindern.  
Manuelles extrudieren während der Pause deaktiviert den automatischen Retract des Extruders beim Fortsetzen.

## Wessix's Hilfe-Video:
[![ScreenShot](https://downfight.de/picproxy.php?url=http://image.prntscr.com/image/d7b7fade0c7343eeb67b680339478894.png)](http://youtu.be/iu9Nft7SXD8)  