# EPICS Asyn Client for iseg Hardware Abstraction Layer
Asyn-based device support for Iseg HV PS via iseghal service library

## Introduction
The isegHAL library offers a string based application interface - API. The data
collection from the iseg high voltage modules is done in the background. All
communication handshake is handled by the isegHAL.

drvAsynIseghalService offers EPICS device support routines to use the isegHAL library
within your EPICS applications.

## Build
Set paths to `EPICS_BASE` and `ISEGHAL` in `configure/RELEASE.local`.
The header files from isegHAL are searched in `$ISEGHAL` and `$ISEGHAL/include`,
the shared object files in `$ISEGHAL` and `$ISEGHAL/lib`.

If the variable `CHECK_TIMESTAMPS` is defined in `configure/RELEASE.local`
the device support will check if the isegHAL has updated the corresponding value
within the last 30 seconds. If not the record is set to a TIMEOUT_ALARM.

### Cross compiling
If you want to cross-compile the devIsegHal module, the `ISEGHAL` variable should
not be defined in `configure/RELEASE.local`. Instead, only define `EPICS_BASE` in
this file. `ISEGHAL` should be defined in `configure/RELEASE.Common.{T_A}` where
`{T_A}` stands for the target architecture (e.g. linux-arm).

## Supported Record Types

| Record type                | isegDataType |
| -------------------------- |:------------:|
| ai/ao records              | R4           |
| bi/bo records              | BOOL         |
| mbbiDirect records         | UI1 & UI4    |
| longin/longout records     | UI1 & UI4    |
| stringin/stringout records | STR          |

*Note: the maximum string length for stringin/out records is limited to 40 characters while the maximal length for the value of an IsegItemValue is 200.
Thus only the first 39 characters of the IsegItemValue are copied to record's VAL field (plus Null-Character for string termination).*
