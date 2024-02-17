#!../../bin/linux-x86_64/drvAsynIseghalServiceTest

< envPaths


#device connection parameters
epicsEnvSet(HAL_PORT, "1454")
epicsEnvSet(USER, "user")
epicsEnvSet(PASSWORD, "pass")
epicsEnvSet(IFACE, "can0")
epicsEnvSet(HAL_SERVER, "hal://ts2-cm-antenna-10kv-hvps.cslab.esss.lu.se")

epicsEnvSet(ISEGSESSION, "ETHHAL")
epicsEnvSet(ISEGIFACE, "$(HAL_SERVER):$(HAL_PORT)/$(IFACE),$(USER),$(PASSWORD)")


## Register all support components
dbLoadDatabase "$(TOP)/dbd/drvAsynIseghalService.dbd"
drvAsynIseghalService_registerRecordDeviceDriver pdbbase


## Load ISEGHAL service driver
drvAsynIseghalServiceConfig( "$(ISEGSESSION)", "$(ISEGIFACE)", "icsmini", 1 )

## Load record instances

cd ${TOP}/iocBoot/${IOC}
iocInit
