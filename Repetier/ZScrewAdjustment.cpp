/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Repetier.h"
#include "ZScrewAdjustment.h"

namespace ZScrewAdjustment {

static Advice advice{};
const Advice& current() { return advice; }

void calculate() {
    Com::printFLN(PSTR(" "));
    Com::printFLN(PSTR("Z-Schrauben-Helper: "));

    /*IDEAL ist es, wenn mand den Drucker "kalt" und frisch aufgeheizt einstellt.*/

    // Gerade eben muss ein Z-Scan die Matrix korrigiert haben.
    // Der Extruder darf sich seither nicht verändert (T0 -> T1) und nicht abgekühlt haben
    // Dann wird pro °C des aktiven Extruders ~0.001mm Längung angenommen.
    // Dann wird pro °C des Heizbettes ~0.0015mm Längung angenommen.

    // Wegen der unbekannten Nachlängung werden 0.15mm Puffer auf Z=0 angestrebt.
    // Einfach so: wird ein Puffer von 0.05 angestrebt. Die ideale Matrix-Verschiebung wird auf 260°C Hotend / 120°C Bett, was ein Extremwert darstellen soll auf -0.05mm angepeilt.

    //Config
    const float maxExtruderTemperature = Extruder::current->tempControl.heaterCutoffC;
    float maxBedTemperature = 120.0f; //120°C ist ok... mit 180 zu rechnen wäre übertrieben.
    float BedThermalExplansionInMikrons = 1.5f;
    float ExtruderThermalExpansionInMikrons = 0.95f;
    float maxNachdehnungInMikrons = 150.0f;
    float RestAbstandInMikrons = 50.0f;

    //Ist-Abstand.
    float MatrixMaximumInMikrons = (float)bedMatrixHighestZSteps() * Printer::mmPerStep(Z_AXIS) * 1000.0f;

    float ExtruderTemperature = Extruder::current->tempControl.currentTemperatureC;
    if (ExtruderTemperature < 20.0f)
        ExtruderTemperature = 20.0f; // Wenn zu kalt oder undefiniert dann Standardbedingungen annehmen.

    float BedTemperature = Extruder::getHeatedBedTemperature();

    // Umrechnung des aktuellen Zustandes auf die heißesten Werte:
    float MinDistanceInMikronsKalt = MatrixMaximumInMikrons
        + (maxExtruderTemperature - ExtruderTemperature) * ExtruderThermalExpansionInMikrons
        + (maxBedTemperature - BedTemperature) * BedThermalExplansionInMikrons;
    float MinDistanceInMikronsWarm = MinDistanceInMikronsKalt + maxNachdehnungInMikrons;

    Com::printFLN(PSTR("- Alle Werte in Mikrometern / Einheit [um] -"));
    Com::printFLN(PSTR("Matrix-Minimum: "), MatrixMaximumInMikrons);
    Com::printFLN(PSTR("Weitere Extruderausdehnung maximal: "), (maxExtruderTemperature - ExtruderTemperature) * ExtruderThermalExpansionInMikrons);
    Com::printFLN(PSTR("Weitere Heizbettausdehnung maximal: "), (maxBedTemperature - BedTemperature) * BedThermalExplansionInMikrons);
    Com::printFLN(PSTR("Maximalwert-zMatrix bei Maximaltemperaturen (kalter Drucker): "), MinDistanceInMikronsKalt);
    Com::printFLN(PSTR("Maximalwert-zMatrix bei Maximaltemperaturen (durchgewaermter Drucker): "), MinDistanceInMikronsWarm);

    //z.B. -200 <-- um diesen Wert dürfte man korrigieren, wenn der Drucker zum Messzeitpunkt voll durchgewärmt wäre. Weiß er aber nicht!
    // Ein vorgewärmter Drucker justiert das Heizbett eher auf -0.2, ein kalter Drucker justiert es eher auf -0.05 bei Spitzentemperaturen. Beides ist ok.
    //float SollkorrekturKalt = (MinDistanceInMikronsKalt + RestAbstandInMikrons);
    //z.B.  -50 <-- diesen Wert darf man in jedem Fall korrigieren.
    float SollkorrekturWarm = (MinDistanceInMikronsWarm + RestAbstandInMikrons);

    /*
    Wenn ich den Test mit einem bereits warmen Drucker mache, plane ich unnötig eine Sicherheit ein, die ich nur einrechne, weil der Drucker aktuell kalt sein könnte.
    Also warmer Drucker: Bett weiter hoch justieren, also Schraube weiter rein, also Drehsinn Minus.
    */
    // |+0-|..Puffer..|.......Nachdehnung........|Kalt-Soll-Einstellung|
    // |+0-|..Puffer..|Warm-Soll-Einstellung|

    /* TIPP: -> Schraube bis maxNachdehnungInMikrons ~ 150um weiter runter(=Bett weiter hoch =Drehsinn Minus) empfehlen, wenn der Drucker aktuell "mehr druchgewärmt" wäre. */

    Com::println();
    Com::printSharpLine();
    Com::printF(PSTR("Sollkorrektur: "), SollkorrekturWarm, 0);
    Com::printF(PSTR(" [um] = "), SollkorrekturWarm * 0.001f, 3);
    Com::printFLN(PSTR(" [mm]"));

    //meldung:
    advice.millimetres = SollkorrekturWarm * 0.001f;

    float ZSchraubenDrehungenWarm = SollkorrekturWarm * 0.002f; //[Sollkorrektur in mm] geteilt durch [Gewinde: 0.5 mm/Umdrehung] -> Sollkorrektur / 1000 / 0.5
    Com::printF(PSTR("Sollumdrehungen: "), ZSchraubenDrehungenWarm, 1);
    Com::printF(PSTR(" [U] = "), ZSchraubenDrehungenWarm * 360, 0);
    Com::printF(PSTR(" [Grad]"));

    //meldung:
    advice.turns = ZSchraubenDrehungenWarm;

    if (ZSchraubenDrehungenWarm > 0)
        Com::printFLN(PSTR(" (+ heisst rausdrehen/linksrum/gegen die Uhr)")); // Bett wird nach unten justiert
    else
        Com::printFLN(PSTR(" (- heisst reindrehen/rechtsrum/im Uhrzeigersinn)")); // Bett wird nach oben justiert

    Com::printFLN(PSTR("Je kaelter der Gesamtdrucker aktuell ist (nach langer Pause frisch angeschaltet), desto besser der Korrekturwert."));
    advice.status = Status::Ok;                                                             // Neg -> Matrix negativ -> ok. ausser, wenn:
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2        // TODO: Prüfen ob das beim RF2000v2 stimmen wird.
    if (-0.5f <= ZSchraubenDrehungenWarm && SollkorrekturWarm < 40.0f /* [um] */) { // < 0.25mm = 0.5Umdrehungen ist mit dem RF2000 nicht machbar.
        Com::printFLN(PSTR(" (Die Z-Schraube ist ok!)"));                           // Das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        advice.status = Status::Ok;                                                                        //neg
    } else if (SollkorrekturWarm >= 40.0f /* [um] */) {                                            // Dann bin ich rechnerisch um Z = 0 (50um mit 10um toleranz, die ich fordere.)
                                                                                                   // Eine korrektur von mehr als +40um heißt, ich bin gerade vermutlich im Z>0
        Com::printFLN(PSTR(" (Die Z-Schraube weiter raus! Das Bett scheint zu hoch zu liegen.)")); // Das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        advice.status = Status::BedTooHigh; //pos
    }
    Com::printFLN(PSTR(" (RF2000: Minimal eine halbe Schraubendrehung (dZ=0.25mm-Schritte) einstellbar.)")); // Das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
#else                                                                                                        //if MOTHERBOARD == DEVICE_TYPE_RF1000
    if (SollkorrekturWarm >= 40.0f /* [um] */) { // Dann bin ich rechnerisch um Z = 0 (50um mit 10um toleranz, die ich fordere.)
        // Eine korrektur von mehr als +40um heißt, ich bin gerade vermutlich im Z>0
        Com::printFLN(PSTR(" (Die Z-Schraube weiter raus! Das Bett scheint zu hoch zu liegen.)")); // Das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        advice.status = Status::BedTooHigh; //pos
    }
#endif
    Com::printSharpLine();
}

} // namespace ZScrewAdjustment
