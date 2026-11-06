# Proto3 to TBST transfer review

## Result

`Proto3_to_TBST_transfer.params` is the broad transfer file. It contains 109
parameters: 105 Proto3 configuration differences plus four explicit values that
route the only GPS to the physical GPS2/SERIAL4 connector.

It includes the battery capacity and QLand failsafe actions, full actuator map,
servo endpoints/reversals/trims, `Q_TILT_ENABLE`, vectored-yaw settings,
`Q_OPTIONS`, QRTL/landing behavior, QuickTune configuration, flight modes,
telemetry stream rates, and the remaining Proto3 flight configuration.

## Excluded sensor-specific values

- `AHRS_ORIENTATION`, `AHRS_TRIM_X`, and `AHRS_TRIM_Y`.
- Airspeed device ID and offset.
- Barometer device ID and ground-pressure measurement.
- Compass device IDs, priorities, external/internal selection, and associated
  hardware-specific configuration.
- All changed `INS_*` accelerometer/gyro IDs, offsets, scales, calibration
  temperatures, and sensor sampling configuration.
- Battery power-module monitor type, voltage/current pins, and voltage/current
  calibration multipliers. The pack capacity and failsafe policy do transfer.
- Proto3 GPS-port layout. Instead the result explicitly uses one logical GPS1 on
  the physical GPS2 connector: `SERIAL3_PROTOCOL=-1`, `SERIAL4_PROTOCOL=5`,
  `GPS1_TYPE=1`, and `GPS2_TYPE=0`.

## Excluded PID gains

The following nine changed, vehicle-tuned gains remain at TBST's values:

- `Q_A_RAT_PIT_P`, `Q_A_RAT_PIT_I`, `Q_A_RAT_PIT_D`
- `Q_A_RAT_RLL_P`, `Q_A_RAT_RLL_I`, `Q_A_RAT_RLL_D`
- `Q_A_RAT_YAW_P`, `Q_A_RAT_YAW_I`, `Q_A_RAT_YAW_D`

The associated filter settings (`FLTT`, `FLTD`, and `FLTE`) transfer because they
are not P/I/D gains. QuickTune is also transferred, but its saved Proto3 gains are
not; `QWIK_AUTO_SAVE=0` remains as in Proto3.

## Preserved TBST functionality and non-configuration state

- Precision landing remains enabled and all `PLND_*` parameters stay untouched.
- The TBST rangefinder configuration remains untouched.
- `MIS_TOTAL` and `STAT_*` are excluded because they are mission/runtime state,
  not transferable configuration.
- Old Proto3 names `Q_P_JERK_D` and `Q_P_JERK_NE` are excluded because ArduPlane
  4.7.1 already exposes their renamed equivalents with the same values.

## Battery note

Proto3 and TBST both have `BATT_LOW_VOLT=0`, `BATT_CRT_VOLT=0`,
`BATT_LOW_MAH=0`, and `BATT_CRT_MAH=0`; those zero values disable the triggers.
The transfer copies Proto3's `BATT_FS_LOW_ACT=4`, `BATT_FS_CRT_ACT=4` (QLand),
and `BATT_LOW_TIMER=8`, but there will be no battery-triggered QLand until a
voltage or remaining-capacity threshold is configured.

## Loading and verification

1. Save a fresh full TBST parameter backup and remove all propellers.
2. Load `Proto3_to_TBST_transfer.params`, reboot, and refresh parameters.
3. Load the same file again so the newly enabled `Q_TILT_*` and `QWIK_*` subgroups
   are accepted, then reboot and refresh once more.
4. Verify GPS detection on the physical GPS2 connector, all servo/motor functions,
   directions and travel, tilt operation and vectored yaw, flight-mode positions,
   battery monitor readings, rangefinder, and precision-landing messages.
5. Review `ARMING_SKIPCHK=4684`; it is copied from Proto3, but skipped pre-arm
   checks should be reduced after the hardware is ready.
