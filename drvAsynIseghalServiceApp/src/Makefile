TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Use typed rset structure (see 3.16.1 release notes)
USR_CPPFLAGS += -DUSE_TYPED_RSET

# xxxRecord.h will be created from xxxRecord.dbd
DBDINC += xxxRecord

# Install xxxSupport.dbd into <top>/dbd
DBD += xxxSupport.dbd

# Build an IOC support library
LIBRARY_IOC += drvAsynIseghalServiceSupport

# Compile and add code to the support library
drvAsynIseghalServiceSupport_SRCS += xxxRecord.c
drvAsynIseghalServiceSupport_SRCS += devXxxSoft.c

# Link locally-provided code into the support library,
# rather than directly into the IOC application, that
# causes problems on Windows DLL builds
drvAsynIseghalServiceSupport_SRCS += dbSubExample.c
drvAsynIseghalServiceSupport_SRCS += devdrvAsynIseghalServiceVersion.c
drvAsynIseghalServiceSupport_SRCS += drvAsynIseghalServiceHello.c
drvAsynIseghalServiceSupport_SRCS += initTrace.c

drvAsynIseghalServiceSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

# Auto-generate a header file containing a version string.
# Version comes from the VCS if available, else date+time.
GENVERSION = drvAsynIseghalServiceVersion.h
# Macro name
GENVERSIONMACRO = drvAsynIseghalServiceVERSION

# Build the IOC application
PROD_IOC = drvAsynIseghalService

# drvAsynIseghalService.dbd will be created and installed
DBD += drvAsynIseghalService.dbd

# drvAsynIseghalService.dbd will include these files:
drvAsynIseghalService_DBD += base.dbd
drvAsynIseghalService_DBD += xxxSupport.dbd
drvAsynIseghalService_DBD += dbSubExample.dbd
drvAsynIseghalService_DBD += devdrvAsynIseghalServiceVersion.dbd
drvAsynIseghalService_DBD += drvAsynIseghalServiceHello.dbd
drvAsynIseghalService_DBD += initTrace.dbd

# drvAsynIseghalService_registerRecordDeviceDriver.cpp derives from drvAsynIseghalService.dbd
drvAsynIseghalService_SRCS += drvAsynIseghalService_registerRecordDeviceDriver.cpp

# Build the main IOC entry point where needed
drvAsynIseghalService_SRCS_DEFAULT += drvAsynIseghalServiceMain.cpp
drvAsynIseghalService_SRCS_vxWorks += -nil-

# Link in the code from our support library
drvAsynIseghalService_LIBS += drvAsynIseghalServiceSupport

# To build SNL programs, SNCSEQ must be defined
# in the <top>/configure/RELEASE file
ifneq ($(SNCSEQ),)
    # Build sncExample into drvAsynIseghalServiceSupport
    sncExample_SNCFLAGS += +r
    drvAsynIseghalService_DBD += sncExample.dbd
    # A .stt sequence program is *not* pre-processed:
    drvAsynIseghalServiceSupport_SRCS += sncExample.stt
    drvAsynIseghalServiceSupport_LIBS += seq pv
    drvAsynIseghalService_LIBS += seq pv

    # Build sncProgram as a standalone program
    PROD_HOST += sncProgram
    sncProgram_SNCFLAGS += +m
    # A .st sequence program *is* pre-processed:
    sncProgram_SRCS += sncProgram.st
    sncProgram_LIBS += seq pv
    sncProgram_LIBS += $(EPICS_BASE_HOST_LIBS)
endif

# Link QSRV (pvAccess Server) if available
ifdef EPICS_QSRV_MAJOR_VERSION
    drvAsynIseghalService_LIBS += qsrv
    drvAsynIseghalService_LIBS += $(EPICS_BASE_PVA_CORE_LIBS)
    drvAsynIseghalService_DBD += PVAServerRegister.dbd
    drvAsynIseghalService_DBD += qsrv.dbd
endif

# Finally link IOC to the EPICS Base libraries
drvAsynIseghalService_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE

# Explicit dependency needed for generated header file
devdrvAsynIseghalServiceVersion$(DEP): $(COMMON_DIR)/$(GENVERSION)
