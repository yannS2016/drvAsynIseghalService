TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DBD += drvAsynIseghalService.dbd

LIBRARY_IOC += drvAsynIseghalService

drvAsynIseghalService_SRCS += drvAsynIseghalServiceVersion.c
drvAsynIseghalService_SRCS += drvAsynIseghalService.cpp

drvAsynIseghalService_LIBS += $(EPICS_BASE_IOC_LIBS)

drvAsynIseghalService_DBD += base.dbd
drvAsynIseghalService_DBD += asyn.dbd
drvAsynIseghalService_DBD += drvAsynIseghalServiceVersion.dbd
#=============================
# Auto-generate a header file containing a version string.
# Version comes from the VCS if available, else date+time.
GENVERSION = drvAsynIseghalServiceVersion.h
# Macro name
GENVERSIONMACRO = drvAsynIseghalServiceVERSION

PROD_IOC += drvAsynIseghalServiceTest

# drvAsynIseghalService_registerRecordDeviceDriver.cpp will be created from drvAsynIseghalService.dbd
drvAsynIseghalServiceTest_SRCS += drvAsynIseghalService_registerRecordDeviceDriver.cpp
drvAsynIseghalServiceTest_SRCS_DEFAULT += drvAsynIseghalServiceMain.cpp

drvAsynIseghalServiceTest_LIBS += drvAsynIseghalService asyn
drvAsynIseghalServiceTest_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
# Explicit dependency needed for generated header file
drvAsynIseghalServiceVersion$(DEP): $(COMMON_DIR)/$(GENVERSION)
