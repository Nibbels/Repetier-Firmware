# Mod Menu: transferring learned SenseOffset into persistent Z-offset

> **This is intentional behavior, not a random Z-offset bug.** Confirmed from the panel actions, `Configuration.h` and the original **2017-01-25 RF.01.37.mod changelog**, which explicitly states: “Within the Mod Menu you can alter your Z-Offset and easily push SenseOffset to your Z-Offset with the Z-Up and Z-Down Button.”

## Why this exists

During the first layer, SenseOffset gradually raises nozzle clearance when DMS force suggests the gap is too small. That **dynamic correction is RAM-only**. Sometimes the measured correction reveals a *persistent* setup discrepancy that should survive a restart: a thicker/thinner magnetic print sheet, bed or nozzle positioning after manually pulling a printed object from a bed, or an imperfect initial Z-offset calibration.

The operator can deliberately **transfer (capitalize) part of the dynamic correction into the saved manual Z offset** without changing net clearance. This helps improve the baseline for subsequent prints. It does not automatically decide which DMS readings reflect permanent versus thermal or material-dependent errors: the operator chooses to make that correction permanent.

## The buttons and their direction matter

The behavior applies only on **Mod Menu / page 2** (`uid.menuLevel == 0 && uid.menuPos[0] == 1`). In other contexts, the same buttons command *direct Z motion*, not a Z-offset adjustment.

In the Mod Menu:
- Physical **Z / heat-bed DOWN** action `UI_ACTION_RF_HEAT_BED_DOWN` calls `zOffsetPlusAction()` and **increases the saved manual offset by 5 µm**; dynamic SenseOffset is unchanged, total clearance increases 5 µm.
- Physical **Z / heat-bed UP** action `UI_ACTION_RF_HEAT_BED_UP` calls `zOffsetMinusAction()`. When dynamic SenseOffset is positive, it **consumes up to 5 µm of dynamic correction first**, leaving manual Z-offset unchanged as long as sufficient dynamic correction remains. Net clearance decreases by up to 5 µm.
- Each actual change in the manual offset is written to the **MCU's internal EEPROM**, with checksum update if the value changed. Dynamic SenseOffset remains RAM-only.

The step size is `Z_OFFSET_BUTTON_STEPS=5` µm (not the regular menu's `Z_OFFSET_MENU_STEPS=25` µm).

## Example: 200 µm already learned, manual offset initially zero

| Mod-menu action | Saved manual offset | Live SenseOffset | Combined static correction |
| --- | ---: | ---: | ---: |
| Before | 0 µm | +200 µm | +200 µm |
| Bed DOWN (plus) | +5 µm | +200 µm | +205 µm |
| Bed UP (minus) | +5 µm | +195 µm | +200 µm |
| Bed DOWN (plus) | +10 µm | +195 µm | +205 µm |
| Bed UP (minus) | +10 µm | +190 µm | +200 µm |

Each **completed DOWN→UP pair** transfers 5 µm from dynamic to stored manual offset while restoring the initial total correction. The reverse UP→DOWN pair has the same net effect when the controller is not modifying the dynamic offset between presses and neither action hits a limit.

After 40 completed pairs, approximately +200 µm has moved from dynamic to manual; the total remains approximately +200 µm. The operator can also use an **extra DOWN press** to add clearance if even the combined correction is insufficient. The pair itself does **not** add more space.

## Important constraints / edge cases

1. **Active pressure regulation is asynchronous.** SenseOffset may increase again between button presses if force stays above threshold. The simple invariant applies only when no new dynamic correction is added during the pair.
2. **Clamps and zero snapping:** buttons clamp the menu offset to the configured range and snap near zero to zero; the arithmetic is not exact at all arbitrary offset values (especially values entered through G-code that are not multiples of 5 µm). Do not assert perfectly constant combined correction in all edge cases.
3. **Dynamic near zero:** once the remaining dynamic correction is below 5 µm, the last UP press consumes it and can also change the saved manual offset. A fully paired sequence must be reviewed at this boundary.
4. **EEPROM writes:** the accumulating manual correction is committed at each actual manual-offset change. This is by design, but many repeated cycles generate repeated EEPROM writes; avoid treating this as a high-frequency automatic control loop.
5. **Do not reset dynamic offset when enabling or editing the manual offset** without honoring this transfer operation. A general “always zero dynamic on adjustment” fix would destroy the feature.
6. **Transfer is a decision, not proof of correct calibration.** Thermal drift, ooze/clogging, filament pressure and sensor bias can produce temporary SenseOffset increases. Taking them into permanent EEPROM could make future *cold* or *warm* prints worse. The operator may want a calibration scan or visual first-layer check before retaining a large correction.

## Relevance to the Z lifecycle bugs

- **Intentional:** `zOffsetMinusAction()` preferentially decreases dynamic SenseOffset instead of the saved manual offset; alternating UP/DOWN transfers it to EEPROM. The earlier risk register called this “intentional but surprising”; this document specifies the engineering purpose.
- **Fixed on `ai-firmware-review`:** before [commit b4ae1dc](https://github.com/Nibbels/Repetier-Firmware/commit/b4ae1dc53a184c7a866db987f4419413912a58d5), `M3006` omitted the current dynamic offset when recomputing `g_staticZSteps`. The patched `M3006` preserves the sum. It still *sets the manual offset absolutely* rather than performing the operator's button-driven transfer, as intended.
- **Partially fixed:** Z homing now removes only the dynamic remainder **after disabling CMP**, preserving the EEPROM-backed portion deliberately transferred by the operator ([ba93708](https://github.com/Nibbels/Repetier-Firmware/commit/ba93708d41c84fcce16872720b6b954da70ea157)). CMP-off also clears dynamic offset even after `M3909 P0` ([733833a](https://github.com/Nibbels/Repetier-Firmware/commit/733833a7c062b461c9fbb9de8a72ea202a65c812)). **Consecutive prints without Z homing remain open.**

## Source and history

- [Mod-menu buttons mapped to plus/minus](../../Repetier/UiActions.cpp)
- [Offset plus/minus logic and immediate EEPROM persistence](../../Repetier/UiActions.cpp)
- [5 µm per Mod-menu button](../../Repetier/Configuration.h#L901-L908)
- [Historical first-hand design intent, January 25, 2017](../../changelog.txt#L1760-L1768)
- [All offset terms and storage](offset-ledger.md) · [Lifecycle risks](lifecycle-and-findings.md)
