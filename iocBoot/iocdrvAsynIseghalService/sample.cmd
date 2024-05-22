#!../../bin/linux-x86_64/drvAsynIsegHalServiceTest

< envPaths
# Module based systems
epicsEnvSet(ICSMINI, "")
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

#- Optional
callbackSetQueueSize(10000)

#- Register all support components
dbLoadDatabase "$(TOP)/dbd/drvAsynIsegHalServiceTest.dbd"
drvAsynIsegHalServiceTest_registerRecordDeviceDriver pdbbase

epicsEnvSet(iCSMODEL, "iCSmini2")
epicsEnvSet(RECONATTEMPT, "2")

#- Load ISEGHAL service driver
drvAsynIsegHalServiceConfig( "$(ISEGSESSION)", "$(ISEGIFACE)", "$(iCSMODEL)", $(RECONATTEMPT) )

epicsThreadSleep(1)

asynSetTraceMask("$(ISEGSESSION)", 0, ERROR|DRIVER)
$(ICSMINI=#-)dbLoadRecords("../../db/icsmini-micc.db","P=DRVASYNISEGHAL:,R=ESS:,PORT=$(ISEGSESSION)")
$(CCXX=#-)dbLoadRecords("../../db/cc24.db","P=DRVASYNISEGHAL:,R=ESS:,PORT=$(ISEGSESSION)")
#-dbLoadRecords("../../db/test.db","P=DRVASYNISEGHAL:,R=ESS:,PORT=$(ISEGSESSION)")

cd ${TOP}/iocBoot/${IOC}

iocInit
date

epicsThreadSleep(1)
#- set user defined polling thread frequency.
#- for higher freq 'queueRequest' will complain try to adjust 'RequestInterval'
drvIsegHalPollerThreadSetOpt("Interval", "0.008")
#- Poller Thread interrupt queueRequest delay, adjust this accordingly.
drvIsegHalPollerThreadSetOpt("RequestInterval", "0.002")
