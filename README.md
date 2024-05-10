# EPICS Asyn PortDriver Client for ISEG Hardware Abstraction Layer
Asyn-based device support for Iseg iCS-based HV/LV control systems using the isegHAL service library.

## Introduction
EPICS Asyn Driver for ISEG HV/LV PS systems based on the unified ISEG control platform 'iCS' capable systems i.e iCSmini 2, CC24, and SHR(not tested yet) controllers
offering control and monitoring of ISEG high voltage through different communication interfaces. The driver uses the isegHAL services library providing a string-based TCP/IP application interface - API  based on a Qt SSL Socket using OpenSSL. Data collection from the ISEG HV/LV modules is done on request or from a background task piping access request to the port driver. All communication handshake is handled by the isegHAL service library patched to provide some level of connexion handshaking management.

More info : [isegHAL](https://iseg-hv.com/download/SOFTWARE/iCS/doc/isegHAL)

## Supported iCS Systems
-  ICS  MINI 2 Ethernet/Wifi Box for CAN/Serial Connected HV devices
-  CC24 controller module for use with MMS/MPOD compatible ECH crate series and modules
-  SHR  switchable high-end high precision ac/dc desktop hv source-measure-unit

More info: [ICS](https://iseg-hv.com/ics/)
## Dependencies

-  [EPICS base 7.0.7 or later](http://www.aps.anl.gov/epics/)
-  [AsynDriver 4-42 or later](http://www.aps.anl.gov/epics/modules/soft/asyn/)
-  [isegHAL  version 1.9.0 or later](https://iseg-hv.com/download/?dir=SOFTWARE/isegHAL)

## Build & Install
 1.  Edit 'configure/RELEASE.local' and change the paths to ASYN and EPICS_BASE
 2.  Edit 'configure/RELEASE.local' and set the TOP path for ISEGHAL header and library.
 3.  Type `make install` to compile the package.

Note:
-  isegHAL Service header files are searched in `$ISEGHAL` and `$ISEGHAL/include`, and the shared object files in `$ISEGHAL` and `$ISEGHAL/lib`.

-  If the variable `CHECK_TIMESTAMPS` is defined in `configure/RELEASE.local` the driver will check if the isegHAL has updated the corresponding value within the last 30 seconds. If not the record is set to a TIMEOUT_ALARM (this feature is not yet implemented)

## Usage
To load the driver use the command inside the IOC
```
drvAsynIsegHalServiceConfig( "ISEGHALSESSION", "ISEGHALIFACE", "iCSMODEL", AUTOCNNECT=1 )
```
where `ISEGHALSESSION` is the name used by Asyn to identify the driver and the isegHAL connection,
`ISEGHALIFACE` INTERFACE is the actual name of the hardware interface used by the iCS system (e.g. "can0" for a CAN interface)
`iCSMODEL` refers to the ICS control system devices the driver connects to, i.e ICSMINI2, CC24 or SHR
and `autoConnect` is a flag that whether the driver should automatically connect to the device must be set to 1.

## Records
EPICS records using `drvAsynIsegHalService`, set its `DTYP` field to one of the supported Asyn interfaces registered by the driver.
The driver provides an asynUINT32Digital interface for read/writing Status related isegHal items. asynInt32, aysnFloat64, and asynOctet
interfaces are provided for reading other isegHAL items data types. Nonetheless, status-related items can be accessed using the asynInt32 interface too.

This driver provides generic access to the supported iCS control systems devices. That is, device address access for the related isegHAL item is done via predefined port  driver user parameters.
Instead, This is done at the record level and inside the driver using the `asynDrvUser` interface. The Database records INP and OUT links follow this format: `@asyn($(PORT))TYPE_ValidItem($(ADDR=0.1000)). `

Here `TYPE` is `INT, DBL, DIG or STR`. `ValidItem` is a valid isegHAL items. And `ADDR` is the item's address within the connected CAN system.
The `TYPE_ValidItem($(ADDR=0.1000)) ` drvInfo string is parsed to build the fully qualified name, FQN for the isegHAL item, i.e `0.1000.ValidItem` associated with the correct port driver registered interface for its `TYPE`.

isegHAL provides its timestamp for the last time a value was changed (timeStampLastChanged). To use it as the record's timestamp, the TSE field has to be set to -2.

```
record( ai, "$(P)$(R):CRATE$(ID=1000)-FanSpeed") {
    field(DTYP, "asynFloat64")
    field(INP,  "@asyn($(PORT))DBL_FanSpeed($(LINEADDR=0.1000))")
    field(TSE,  "-2")
}
```
*Note: the maximum string length for stringin/out records is limited to 40 characters while the maximum length for the value of an IsegItemValue is 200.
Thus only the first 39 characters of the IsegItemValue are copied to the record's VAL field (plus Null-Character for string termination).*

## Asynchronous Handling
ICS-based systems provide a web interface interface where control and configuration parameters can be updated. Moreover, some items are updated by the underlying iCS systems under certain configured conditions.
i.e., if a trip occurs, the corresponding `setON` bit in the channel control register will be reset. Such changes are monitored by drvAsynIsegHalService through a polling thread.

The driver uses `ASYN_CANBLOCK` flag to define an asynchronous communication with the isegHAL device. Each output record (except for stringout and stringin records) with  `info(asyn:READBACK, "1")`
is automatically registered to this thread and their values are checked for updates from isegHAL. a record VAL field and timestamp are set to the new values only when a value has changed.
Setting the SCAN field of input records to I/O Intr will also register these records for the thread monitoring the values in isegHAL.

The polling thread goes through the list of registered records, queues a request to drvAsynIsegHalService, and waits for 2 seconds. device access will happen in the port thread via the registered callback.
the callback method will check for an update on the value read and callback for record further processing.
Tw0 tunning timing parameter can be set using IOC shell commands to adjust the polling time and the queueRequest frequency to avoid `queueRequest is already queued` warning from aynManager.

*Note: The interrupt asynUsers in the poller thread queuRequest list is a duplicate of all asynUsers that have registered for interrupts. it's created once when the poller thread object is instantiated.
Thus the list is static and cannot be updated at run time.*

## IOC Shell Commands

```
drvIsegHalPollerThreadSetOpt( "key", "value" )

drvIsegHalPollerThreadSetOpt("Interval", "0.05")
drvIsegHalPollerThreadSetOpt("RequestInterval", "0.005")

```

| Key       | Meaning                                    | Value                                                          |
| --------- | ------------------------------------------ |:--------------------------------------------------------------:|
| Interval  | Change the interval of the polling thread  | A value of 0 means no pause between two iterations of the list |
| RequestInterval  | Poller Thread interrupt queueRequest delay          | adjust this accordingly                        |
