# Parameters excluded from Proto3 → TBST transfer

This is the exact exclusion set: **63 parameters** from the 143 changed common
parameters and 25 Proto3-only parameters. Values are shown as **TBST → Proto3**.

## Sensor identity, calibration, mounting, or hardware configuration (44)

| Parameter | TBST | Proto3 | Reason |
|---|---:|---:|---|
| `AHRS_ORIENTATION` | 0 | 12 | FC/sensor mounting orientation |
| `AHRS_TRIM_X` | 0 | -0.024539083242416382 | Vehicle level trim |
| `AHRS_TRIM_Y` | 0 | 0.056982666254043579 | Vehicle level trim |
| `ARSPD_DEVID` | 141329 | 0 | Airspeed sensor identity |
| `ARSPD_OFFSET` | 30.652420043945312500 | 0 | Airspeed zero calibration |
| `BARO1_DEVID` | 751361 | 721186 | Barometer identity |
| `BARO1_GND_PRESS` | 97240.921875 | 98322.359375 | Local/boot pressure measurement |
| `BATT_AMP_PERVLT` | 36.360000610351562500 | 36.363998413085937500 | Current-sensor calibration |
| `BATT_CURR_PIN` | 4 | 1 | Power-module wiring |
| `BATT_MONITOR` | 4 | 3 | Power-module/monitor type |
| `BATT_VOLT_MULT` | 18.180000305175781250 | 18.181999206542968750 | Voltage-sensor calibration |
| `BATT_VOLT_PIN` | 8 | 0 | Power-module wiring |
| `COMPASS_DEV_ID` | 658433 | 1122313 | Compass identity |
| `COMPASS_DEV_ID2` | 0 | 658945 | Compass identity |
| `COMPASS_EXTERNAL` | 0 | 1 | Compass installation/selection |
| `COMPASS_PRIO1_ID` | 658433 | 1122313 | Compass identity/priority |
| `COMPASS_PRIO2_ID` | 0 | 658945 | Compass identity/priority |
| `INS_ACC1_CALTEMP` | 44.323673248291015625 | 44.907493591308593750 | Accelerometer calibration temperature |
| `INS_ACC2OFFS_X` | 0.098997846245765686 | 0.451402336359024048 | Accelerometer offset |
| `INS_ACC2OFFS_Y` | -0.173634901642799377 | 0.042171914130449295 | Accelerometer offset |
| `INS_ACC2OFFS_Z` | 0.023992538452148438 | 0.340983927249908447 | Accelerometer offset |
| `INS_ACC2SCAL_X` | 1 | 0.983528017997741699 | Accelerometer scale calibration |
| `INS_ACC2SCAL_Y` | 1 | 0.984356880187988281 | Accelerometer scale calibration |
| `INS_ACC2SCAL_Z` | 1 | 0.983131170272827148 | Accelerometer scale calibration |
| `INS_ACC2_CALTEMP` | 39.5 | 35 | Accelerometer calibration temperature |
| `INS_ACC2_ID` | 2818570 | 2688010 | Accelerometer identity |
| `INS_ACCOFFS_X` | 0.121198192238807678 | 0.000971345580182970 | Accelerometer offset |
| `INS_ACCOFFS_Y` | -0.131653413176536560 | 0.083431825041770935 | Accelerometer offset |
| `INS_ACCOFFS_Z` | -0.035792350769042969 | -0.014146574772894382 | Accelerometer offset |
| `INS_ACCSCAL_X` | 1 | 0.998977184295654297 | Accelerometer scale calibration |
| `INS_ACCSCAL_Y` | 1 | 0.998420417308807373 | Accelerometer scale calibration |
| `INS_ACCSCAL_Z` | 1 | 0.985351264476776123 | Accelerometer scale calibration |
| `INS_ACC_ID` | 3408650 | 2621706 | Accelerometer identity |
| `INS_FAST_SAMPLE` | 3 | 1 | IMU hardware/sampling configuration |
| `INS_GYR1_CALTEMP` | 24.033817291259765625 | 29.464717864990234375 | Gyro calibration temperature |
| `INS_GYR2OFFS_X` | -0.003087744116783142 | -0.003554038703441620 | Gyro offset |
| `INS_GYR2OFFS_Y` | 0.003648944431915879 | 0.002258557360619307 | Gyro offset |
| `INS_GYR2OFFS_Z` | -0.002093879040330648 | -0.000190007005585358 | Gyro offset |
| `INS_GYR2_CALTEMP` | 9.25 | 10 | Gyro calibration temperature |
| `INS_GYR2_ID` | 2818314 | 2687754 | Gyro identity |
| `INS_GYROFFS_X` | 0.000643055303953588 | 0.045350577682256699 | Gyro offset |
| `INS_GYROFFS_Y` | -0.015959171578288078 | 0.008335118182003498 | Gyro offset |
| `INS_GYROFFS_Z` | -0.000017780414054869 | -0.004995238967239857 | Gyro offset |
| `INS_GYR_ID` | 3408650 | 2621706 | Gyro identity |

## Vehicle-tuned PID gains (9)

| Parameter | TBST | Proto3 |
|---|---:|---:|
| `Q_A_RAT_PIT_D` | 0.003599999938160181 | 0.007923411205410957 |
| `Q_A_RAT_PIT_I` | 0.25 | 0.227144867181777954 |
| `Q_A_RAT_PIT_P` | 0.25 | 0.227144867181777954 |
| `Q_A_RAT_RLL_D` | 0.003599999938160181 | 0.013164898380637169 |
| `Q_A_RAT_RLL_I` | 0.25 | 0.346682250499725342 |
| `Q_A_RAT_RLL_P` | 0.25 | 0.346682250499725342 |
| `Q_A_RAT_YAW_D` | 0 | 0.009999998845160007 |
| `Q_A_RAT_YAW_I` | 0.017999999225139618 | 0.050000030547380447 |
| `Q_A_RAT_YAW_P` | 0.180000007152557373 | 0.500000298023223877 |

## TBST precision-landing/rangefinder protection (2)

| Parameter | TBST | Proto3 | Reason |
|---|---:|---:|---|
| `PLND_ENABLED` | 1 | 0 | Keep TBST precision landing enabled |
| `RNGFND1_TYPE` | 16 | 0 | Keep TBST landing rangefinder enabled |

All other `PLND_*` and `RNGFND1_*` settings occur only in TBST and therefore are
not present in the Proto3 transfer source at all; they remain untouched.

## Mission/runtime state, not configuration (6)

| Parameter | TBST | Proto3 |
|---|---:|---:|
| `MIS_TOTAL` | 0 | 7 |
| `STAT_BOOTCNT` | 36 | 114 |
| `STAT_DISTFLWN` | 0 | 5114.02490234375 |
| `STAT_FLTCNT` | 0 | 196 |
| `STAT_FLTTIME` | 0 | 4565 |
| `STAT_RUNTIME` | 27548 | 64840 |

## Obsolete/renamed Proto3 parameters (2)

| Parameter | TBST | Proto3 | TBST 4.7.1 equivalent |
|---|---:|---:|---|
| `Q_P_JERK_D` | absent | 5 | `Q_P_D_JERK=5` |
| `Q_P_JERK_NE` | absent | 2 | `Q_P_NE_JERK=2` |

These were not transferred because the target already has the renamed parameters
with identical values.
