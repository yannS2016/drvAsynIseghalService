#!../../bin/linux-x86_64/drvAsynIseghalServiceTest

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
epicsEnvSet(HAL_SERVER, "hal://ts2-cm-antenna-10kv-hvps.cslab.esss.lu.se")

epicsEnvSet(ISEGSESSION, "ETHHAL")
epicsEnvSet(ISEGIFACE, "$(HAL_SERVER):$(HAL_PORT)/$(IFACE),$(USER),$(PASSWORD)")

#- Register all support components
dbLoadDatabase "$(TOP)/dbd/drvAsynIseghalServiceTest.dbd"
drvAsynIseghalServiceTest_registerRecordDeviceDriver pdbbase

epicsEnvSet(iCSMODEL, "icsmini")
epicsEnvSet(AUTOCNNECT, "1")
#- Load ISEGHAL service driver
drvAsynIseghalServiceConfig( "$(ISEGSESSION)", "$(ISEGIFACE)", "$(iCSMODEL)", $(AUTOCNNECT) )

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
