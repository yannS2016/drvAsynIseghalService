#!../../bin/linux-x86_64/drvAsynIsegHalServiceTest

< envPaths

# Module based systems
epicsEnvSet(MICC, "")
#- CC24/CC23/CC44/CC43 only
#-epicsEnvSet(CCXX, "")

#- Device connection parameters
epicsEnvSet(HAL_PORT, "1454")
epicsEnvSet(USER, "user")
epicsEnvSet(PASSWORD, "pass")
epicsEnvSet(IFACE, "can0")
epicsEnvSet(HAL_SERVER, "hal://X.X.X.X")

epicsEnvSet(ISEGSESSION, "ETHHAL")
epicsEnvSet(ISEGIFACE, "$(HAL_SERVER):$(HAL_PORT)/$(IFACE),$(USER),$(PASSWORD)")

#- Register all support components
dbLoadDatabase "$(TOP)/dbd/drvAsynIsegHalServiceTest.dbd"
drvAsynIsegHalServiceTest_registerRecordDeviceDriver pdbbase

epicsEnvSet(iCSMODEL, "icsmini")
epicsEnvSet(AUTOCNNECT, "1")
#- Load ISEGHAL service driver
drvAsynIsegHalServiceConfig( "$(ISEGSESSION)", "$(ISEGIFACE)", "$(iCSMODEL)", $(AUTOCNNECT) )

epicsThreadSleep(1)

#asynSetTraceMask("$(ISEGSESSION)", 0, ERROR|FLOW|DRIVER|WARNING|DEVICE)

dbLoadRecords("../../db/system-items.template","P=DRVASYNISEGHAL,R=,PORT=$(ISEGSESSION)")
dbLoadRecords("../../db/can-items.template","P=DRVASYNISEGHAL,R=,PORT=$(ISEGSESSION)")
$(MICC=#-)dbLoadRecords("../../db/module-items.template","P=DRVASYNISEGHAL,R=,PORT=$(ISEGSESSION)")
$(CCXX=#-)dbLoadRecords("../../db/crate-items.template","P=DRVASYNISEGHAL,R=,PORT=$(ISEGSESSION)")
dbLoadRecords("../../db/channel-items.template","P=DRVASYNISEGHAL,R=,PORT=$(ISEGSESSION)")

## Load record instances

cd ${TOP}/iocBoot/${IOC}
iocInit
