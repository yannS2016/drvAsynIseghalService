TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
EXCLUDE_ARCHS += linux-ppc64e6500
EXCLUDE_ARCHS += linux-corei7-poky

ifneq ($(ISEGHAL),)
    USR_INCLUDES += -I$(ISEGHAL) -I$(ISEGHAL)/include
    USR_LDFLAGS  += -L$(ISEGHAL) -L$(ISEGHAL)/lib
endif

USR_CXXFLAGS += -Wno-format-truncation -O3 -std=c++11  -g -Wall -mtune=generic -m64 -fPIC

DBD += drvAsynIsegHalService.dbd

LIBRARY_IOC += drvAsynIsegHalService

drvAsynIsegHalService_SRCS += drvAsynIsegHalServiceVersion.c
drvAsynIsegHalService_SRCS += drvAsynIsegHalService.cpp drvIsegHalPollerThread.cpp

drvAsynIsegHalService_SYS_LIBS += isegHAL-service
drvAsynIsegHalService_LIBS += $(EPICS_BASE_IOC_LIBS)

drvAsynIsegHalService_DBD += base.dbd
drvAsynIsegHalService_DBD += asyn.dbd
drvAsynIsegHalService_DBD += drvAsynIsegHalServiceVersion.dbd
drvAsynIsegHalService_DBD += drvAsynIsegHalServiceSupport.dbd
#=============================
# Auto-generate a header file containing a version string.
# Version comes from the VCS if available, else date+time.
GENVERSION = drvAsynIsegHalServiceVersion.h
# Macro name
GENVERSIONMACRO = drvAsynIsegHalServiceVERSION

PROD_IOC += drvAsynIsegHalServiceTest

DBD += drvAsynIsegHalServiceTest.dbd

drvAsynIsegHalServiceTest_DBD += base.dbd
drvAsynIsegHalServiceTest_DBD += drvAsynIsegHalService.dbd

drvAsynIsegHalServiceTest_LIBS += drvAsynIsegHalService asyn

# drvAsynIsegHalService_registerRecordDeviceDriver.cpp will be created from drvAsynIsegHalService.dbd
drvAsynIsegHalServiceTest_SRCS += drvAsynIsegHalServiceTest_registerRecordDeviceDriver.cpp
drvAsynIsegHalServiceTest_SRCS_DEFAULT += drvAsynIsegHalServiceMain.cpp


drvAsynIsegHalServiceTest_SYS_LIBS += isegHAL-service
drvAsynIsegHalServiceTest_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
# Explicit dependency needed for generated header file
drvAsynIsegHalServiceVersion$(DEP): $(COMMON_DIR)/$(GENVERSION)
