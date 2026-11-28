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

FSTRINGVALUE(ui_text_min_reached_unhomed, UI_TEXT_MIN_REACHED_UNHOMED)

/* Diese Funktion soll die Step_Size eines Z-Steps einstellbar machen. */
/* Noch ist das statisch über eine Tabelle, aber man könnte das später auch berechnen */

void configureMANUAL_STEPS_Z(int8_t increment) {
    static const unsigned short stepsize_table[NUM_ACCEPTABLE_STEP_SIZE_TABLE] PROGMEM = ACCEPTABLE_STEP_SIZE_TABLE;

    int loop = 0;
    // Suche die aktuelle Einstellungsposition (oder aufgerundet) in der Tabelle:
    for (loop = 0; loop < NUM_ACCEPTABLE_STEP_SIZE_TABLE; loop++)
        if (pgm_read_word(&stepsize_table[loop]) >= Printer::manualMotion.steps(Z_AXIS))
            break;

    // Ändere die Position in die Wunschposition:
    if (increment >= 0)
        loop += 1;
    else
        loop -= 1;

    // Begrenze ringmenge
    if (loop > NUM_ACCEPTABLE_STEP_SIZE_TABLE - 1)
        loop = 0;
    else if (loop < 0)
        loop = NUM_ACCEPTABLE_STEP_SIZE_TABLE - 1;

    // Nutze neuen Wert:
    Printer::manualMotion.setSteps(Z_AXIS, pgm_read_word(&stepsize_table[loop]));

    EEPROM::storeZJogSteps(Printer::manualMotion.steps(Z_AXIS));
} // configureMANUAL_STEPS_Z()

void moveXAction(int8_t direction) {
    bool moveKosys = Printer::manualMotion.coordinateSystem();

    if (Printer::pause.isPaused()) {
        moveKosys = KOSYS_DIRECTOFFSET;
    } else if (PrintLine::hasLines() || isAnyScanRunning()) {
        // this operation is not allowed while a printing or scanning is in progress
        return;
    }

    if (direction < 0 && Printer::isXMinEndstopHit()) {
        // we shall move to the left but the x-min-endstop is hit already, so we do nothing
        showInformation((void*)ui_text_x_axis, (void*)ui_text_min_reached);
        return;
    }

    if (direction > 0 && Printer::isXMaxEndstopHit()) {
        // we shall move to the right but the end of the x-axis has been reached already, so we do nothing
        showInformation((void*)ui_text_x_axis, (void*)ui_text_max_reached);
        return;
    }

    // show that we are active
    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());

    switch (Printer::manualMotion.mode(X_AXIS)) {
    case MOVE_MODE_SINGLE_STEPS: {
        int32_t steps = int32_t(Printer::manualMotion.steps(X_AXIS)) * direction;

        if (direction < 0 && Printer::xyMotion.motorSteps(X_AXIS) + steps < 0 && Printer::isAxisHomed(X_AXIS)) {
            // If we are very close to the homed known Y-Endstop go to 0-Coordinate
            steps = -Printer::xyMotion.motorSteps(X_AXIS);
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::manualMotion.feedrateSource() == FEEDRATE_GCODE) {
                feedrate = Printer::requestedFeedrate();
            }
            Printer::queueRelativeStepsCoordinates(steps, 0, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::manualMotion.coordinateSystem() == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(steps, 0, 0, 0, DirectMoveMode::ManualPosition);
        }

        break;
    }
    case MOVE_MODE_1_MM:
    case MOVE_MODE_10_MM:
    case MOVE_MODE_50_MM: {
        uint8_t distanceMM = 1;
        if (Printer::manualMotion.mode(X_AXIS) == MOVE_MODE_10_MM)
            distanceMM = 10;
        if (Printer::manualMotion.mode(X_AXIS) == MOVE_MODE_50_MM)
            distanceMM = 50;

        int32_t plannedDestination = Printer::getPlannedDirectAxisSteps(X_AXIS) + (int32_t)(direction * distanceMM * Printer::stepsPerMm(X_AXIS));
        if (plannedDestination > Printer::xyMaxSteps(X_AXIS)) {
            showInformation((void*)ui_text_x_axis, (void*)ui_text_max_reached);
            return;
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::manualMotion.feedrateSource() == FEEDRATE_GCODE) {
                feedrate = Printer::requestedFeedrate();
            }
            Printer::queueRelativeMMCoordinates(distanceMM * direction, 0, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::manualMotion.coordinateSystem() == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(distanceMM * Printer::stepsPerMm(Y_AXIS) * direction, 0, 0, 0, DirectMoveMode::ManualPosition);
        }

        break;
    }
    case MOVE_MODE_SINGLE_MOVE: {
        if (PrintLine::directMoveActive()) {
            // we are moving already, there is nothing more to do
            return;
        }

        int32_t steps;
        // Errate homed möglichst den erzwungenen stop-punkt wegen dem bonus der decelleration
        if (direction < 0)
            steps = (Printer::isAxisHomed(X_AXIS) ? -Printer::xyMotion.motorSteps(X_AXIS) : -Printer::xyMaxSteps(X_AXIS));
        else
            steps = (Printer::isAxisHomed(X_AXIS) ? Printer::xyMaxSteps(X_AXIS) - Printer::xyMotion.motorSteps(X_AXIS) : Printer::xyMaxSteps(X_AXIS));

        Printer::offsetRelativeStepsCoordinates(steps, 0, 0, 0, DirectMoveMode::Button);
        break;
    }
    }
} // moveXAction

void moveYAction(int8_t direction) {
    bool moveKosys = Printer::manualMotion.coordinateSystem();

    if (Printer::pause.isPaused()) {
        moveKosys = KOSYS_DIRECTOFFSET;
    } else if (PrintLine::hasLines() || isAnyScanRunning()) {
        // this operation is not allowed while a printing or scanning is in progress
        return;
    }

    if (direction < 0 && Printer::isYMinEndstopHit()) {
        // we shall move to the back but the y-min-endstop is hit already, so we do nothing
        showInformation((void*)ui_text_y_axis, (void*)ui_text_min_reached);
        return;
    }

    if (direction > 0 && Printer::isYMaxEndstopHit()) {
        // we shall move to the front but the end of the y-axis has been reached already, so we do nothing
        showInformation((void*)ui_text_y_axis, (void*)ui_text_max_reached);
        return;
    }

    // show that we are active
    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());

    switch (Printer::manualMotion.mode(Y_AXIS)) {
    case MOVE_MODE_SINGLE_STEPS: {
        int32_t steps = int32_t(Printer::manualMotion.steps(Y_AXIS)) * direction;

        if (direction < 0 && Printer::xyMotion.motorSteps(Y_AXIS) + steps < 0 && Printer::isAxisHomed(Y_AXIS)) {
            // If we are very close to the homed known Y-Endstop go to 0-Coordinate
            steps = -Printer::xyMotion.motorSteps(Y_AXIS);
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::manualMotion.feedrateSource() == FEEDRATE_GCODE) {
                feedrate = Printer::requestedFeedrate();
            }
            Printer::queueRelativeStepsCoordinates(0, steps, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::manualMotion.coordinateSystem() == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(0, steps, 0, 0, DirectMoveMode::ManualPosition);
        }

        break;
    }
    case MOVE_MODE_1_MM:
    case MOVE_MODE_10_MM:
    case MOVE_MODE_50_MM: {
        uint8_t distanceMM = 1;
        if (Printer::manualMotion.mode(Y_AXIS) == MOVE_MODE_10_MM)
            distanceMM = 10;
        if (Printer::manualMotion.mode(Y_AXIS) == MOVE_MODE_50_MM)
            distanceMM = 50;

        int32_t plannedDestination = Printer::getPlannedDirectAxisSteps(Y_AXIS) + (int32_t)(direction * distanceMM * Printer::stepsPerMm(Y_AXIS));
        if (plannedDestination > Printer::xyMaxSteps(Y_AXIS)) {
            showInformation((void*)ui_text_y_axis, (void*)ui_text_max_reached);
            return;
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::manualMotion.feedrateSource() == FEEDRATE_GCODE) {
                feedrate = Printer::requestedFeedrate();
            }
            Printer::queueRelativeMMCoordinates(0, distanceMM * direction, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::manualMotion.coordinateSystem() == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(0, distanceMM * Printer::stepsPerMm(Y_AXIS) * direction, 0, 0, DirectMoveMode::ManualPosition);
        }

        break;
    }
    case MOVE_MODE_SINGLE_MOVE: {
        if (PrintLine::directMoveActive()) {
            // we are moving already, there is nothing more to do
            return;
        }

        int32_t steps;
        // Errate homed möglichst den erzwungenen stop-punkt wegen dem bonus der decelleration
        if (direction < 0)
            steps = (Printer::isAxisHomed(Y_AXIS) ? -Printer::xyMotion.motorSteps(Y_AXIS) : -Printer::xyMaxSteps(Y_AXIS));
        else
            steps = (Printer::isAxisHomed(Y_AXIS) ? Printer::xyMaxSteps(Y_AXIS) - Printer::xyMotion.motorSteps(Y_AXIS) : Printer::xyMaxSteps(Y_AXIS));

        Printer::offsetRelativeStepsCoordinates(0, steps, 0, 0, DirectMoveMode::Button);
        break;
    }
    }
} // moveYAction

void moveZAction(int8_t direction) {
    if (isAnyScanRunning() || !Printer::pause.allowsManualMove()) {
        // this operation is not allowed while some scan is in progress
        return;
    }

    // show that we are active
    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());

    char moveMode = Printer::manualMotion.mode(Z_AXIS);

    const int button = uid.activeButton();
    if (button == UI_ACTION_RF_HEAT_BED_DOWN || button == UI_ACTION_RF_HEAT_BED_UP) {
        // Es wäre evtl. cooler die variable durchzuschleifen, aber so gehts auch!
        // VORSICHT: ES könnte evtl. sein dass sich lastbuttonaction während der ausführung ändert. (??)

        // Sollte diese Aktion durch die Knöpfe ausgeführt worden sein, dann in der Sprungweite limitieren:
        if (PrintLine::hasLines()) {
            // there is some printing in progress at the moment - do not allow single move in this case
            moveMode = MOVE_MODE_SINGLE_STEPS;
        } else {
            moveMode = MOVE_MODE_SINGLE_MOVE;
        }
    }

    if (Printer::pause.isPaused()) {
        moveMode = MOVE_MODE_SINGLE_MOVE;
    } else if (PrintLine::hasLines()) {
        // in case we are printing at the moment, only the single step movements are allowed
        moveMode = MOVE_MODE_SINGLE_STEPS;
    }

    // Limits für die Bewegung in Z
    if (direction > 0 && Printer::isZMaxEndstopHit()) {
        // we shall move to the front but the end of the y-axis has been reached already, so we do nothing
        showInformation((void*)ui_text_z_axis, (void*)ui_text_max_reached);
        return;
    }

    if (direction < 0 && Printer::isZMinEndstopHit()) {
        if (!Printer::isAxisHomed(Z_AXIS)) {
            return;
        } else {
            // Fall down to Single Steps @Endstop
            moveMode = MOVE_MODE_SINGLE_STEPS;
        }
    }

    switch (moveMode) {
    case MOVE_MODE_SINGLE_STEPS: {
        int32_t steps = int32_t(Printer::manualMotion.steps(Z_AXIS)) * direction;

        if (Printer::zMotion.manualStepExceedsMinimum(steps)) {
            showInformation((void*)ui_text_z_axis, (void*)ui_text_min_reached);
            return;
        }

        steps = Printer::pause.limitZJog(steps);

        Printer::offsetRelativeStepsCoordinates(0, 0, steps, 0, DirectMoveMode::ManualPosition);
        break;
    }
    case MOVE_MODE_1_MM:
    case MOVE_MODE_10_MM:
    case MOVE_MODE_50_MM: {
        uint8_t distanceMM = 1;
        if (moveMode == MOVE_MODE_10_MM)
            distanceMM = 10;
        if (moveMode == MOVE_MODE_50_MM)
            distanceMM = 50;

        const long motorZ = Printer::zMotion.motorSteps();
        if (!Printer::isAxisHomed(Z_AXIS)
            || direction > 0
            || motorZ * Printer::mmPerStep(Z_AXIS) >= distanceMM) {
            Printer::offsetRelativeStepsCoordinates(0, 0, distanceMM * Printer::stepsPerMm(Z_AXIS) * direction, 0, DirectMoveMode::ManualPosition);
            return;
        }

        // Approach the switch coordinate first, then allow small single moves.
        const long steps = motorZ > 0 ? -motorZ : -int32_t(Printer::manualMotion.steps(Z_AXIS));
        Printer::offsetRelativeStepsCoordinates(0, 0, steps, 0, DirectMoveMode::ManualPosition);
        break;
    }
    case MOVE_MODE_SINGLE_MOVE: {
        if (PrintLine::directMoveActive()) {
            // we are moving already, there is nothing more to do
            return;
        }

        // Plan to the estimated stop so a homed jog decelerates near it.
        long steps = direction < 0 ? -Printer::zMotion.maxSteps() : Printer::zMotion.maxSteps();
        if (Printer::isAxisHomed(Z_AXIS))
            steps = (direction < 0 ? 0L : Printer::zMotion.maxSteps()) - Printer::zMotion.motorSteps();

        steps = Printer::pause.limitZJog(steps);

        Printer::offsetRelativeStepsCoordinates(0, 0, steps, 0, DirectMoveMode::Button);
        break;
    }
    }
} // moveZAction
