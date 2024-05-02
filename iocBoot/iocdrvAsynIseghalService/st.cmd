#!../../bin/linux-x86_64/drvAsynIseghalServiceTest

< envPaths


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


#- Load ISEGHAL service driver
drvAsynIseghalServiceConfig( "$(ISEGSESSION)", "$(ISEGIFACE)", "icsmini", 1 )

epicsThreadSleep(1)

#asynSetTraceMask("$(ISEGSESSION)", 0, ERROR|FLOW|DRIVER|WARNING|DEVICE)


dbLoadRecords("../../db/drvAsynIseghalService.db","P=iseghal,R=service,PORT=$(ISEGSESSION), ADDR=0.0.0")

## Load record instances

cd ${TOP}/iocBoot/${IOC}
iocInit
