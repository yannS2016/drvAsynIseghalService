#!../../bin/linux-x86_64/drvAsynIsegHalServiceTest

< envPaths
# Module based systems
#epicsEnvSet(ICSMINI, "")
#- CC24/CC23/CC44/CC43 only
epicsEnvSet(CC24, "")
epicsEnvSet(P, "LAB-Crm:")
epicsEnvSet(R, "SC-IOC-014:")
epicsEnvSet(EVR, "OCTOPUS-010:RFS-EVR-101:")

#- Device connection parameters
epicsEnvSet(HAL_PORT, "1454")
epicsEnvSet(USER, "user")
epicsEnvSet(PASSWORD, "pass")
epicsEnvSet(IFACE, "can0")
epicsEnvSet(HAL_SERVER, "hal://172.30.244.97")
epicsEnvSet(ISEGSESSION, "ETHHAL")
epicsEnvSet(ISEGIFACE, "$(HAL_SERVER):$(HAL_PORT)/$(IFACE),$(USER),$(PASSWORD)")

#- Optional
callbackSetQueueSize(10000)

#- Register all support components
dbLoadDatabase "$(TOP)/dbd/drvAsynIsegHalServiceTest.dbd"
drvAsynIsegHalServiceTest_registerRecordDeviceDriver pdbbase

#-epicsEnvSet(ICSMODEL, "iCSmini2")
epicsEnvSet(ICSMODEL, "CC24")
epicsEnvSet(RECONATTEMPT, "2")

#- Load ISEGHAL service driver
drvAsynIsegHalServiceConfig( "$(ISEGSESSION)", "$(ISEGIFACE)", "$(ICSMODEL)", $(RECONATTEMPT) )

epicsThreadSleep(1)

asynSetTraceMask("$(ISEGSESSION)", 0, ERROR|DRIVER)
#$(ICSMINI=#-)dbLoadRecords("../../db/icsmini-micc.db","P=$(P), R=$(R),PORT=$(ISEGSESSION)")
$(CC24=#-)dbLoadRecords("../../db/cc24.db","P=$(P), R=$(R),PORT=$(ISEGSESSION)")
dbLoadRecords("../../db/app.db","P=$(P), R=$(R), MOD="7100092", HVCHAN="0", EVR=$(EVR)")

cd ${TOP}/iocBoot/${IOC}

iocInit

date

# Setting poller thread frequency to 1hz (stable for srf HV IOC)
epicsEnvSet(POLLER_FREQ, "0")
#- Poller Thread interrupt queueRequest delay: 0.005 ms(stable for srf HV IOC).
epicsEnvSet(REQ_FREQ, "0.003")

drvIsegHalPollerThreadSetOpt("Interval", "$(POLLER_FREQ)")
drvIsegHalPollerThreadSetOpt("RequestInterval", "$(REQ_FREQ)")
