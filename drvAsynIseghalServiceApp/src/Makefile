TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Use typed rset structure (see 3.16.1 release notes)
USR_CPPFLAGS += -DUSE_TYPED_RSET

# Build an IOC support library
LIBRARY_IOC += drvAsynIseghalService

# Link locally-provided code into the support library,
# rather than directly into the IOC application, that
# causes problems on Windows DLL builds

drvAsynIseghalService_SRCS += drvAsynIseghalServiceVersion.c

drvAsynIseghalService_LIBS += $(EPICS_BASE_IOC_LIBS)

# Auto-generate a header file containing a version string.
# Version comes from the VCS if available, else date+time.
GENVERSION = drvAsynIseghalServiceVersion.h
# Macro name
GENVERSIONMACRO = drvAsynIseghalServiceVERSION

# Build the IOC application
PROD_IOC = drvAsynIseghalServiceIOC

# drvAsynIseghalServiceIOC.dbd will be created and installed
DBD += drvAsynIseghalServiceIOC.dbd


# drvAsynIseghalService.dbd will include these files:
drvAsynIseghalServiceIOC_DBD += base.dbd
drvAsynIseghalServiceIOC_DBD += drvAsynIseghalServiceVersion.dbd

# drvAsynIseghalServiceIOC_registerRecordDeviceDriver.cpp derives from drvAsynIseghalServiceIOC.dbd
drvAsynIseghalServiceIOC_SRCS += drvAsynIseghalServiceIOC_registerRecordDeviceDriver.cpp

# Build the main IOC entry point where needed
drvAsynIseghalServiceIOC_SRCS_DEFAULT += drvAsynIseghalServiceMain.cpp
drvAsynIseghalServiceIOC_SRCS_vxWorks += -nil-

# Link in the code from our support library
drvAsynIseghalServiceIOC_LIBS += drvAsynIseghalService

# Link QSRV (pvAccess Server) if available
ifdef EPICS_QSRV_MAJOR_VERSION
    drvAsynIseghalServiceIOC_LIBS += qsrv
    drvAsynIseghalServiceIOC_LIBS += $(EPICS_BASE_PVA_CORE_LIBS)
    drvAsynIseghalServiceIOC_DBD += PVAServerRegister.dbd
    drvAsynIseghalServiceIOC_DBD += qsrv.dbd
endif

# Finally link IOC to the EPICS Base libraries
drvAsynIseghalServiceIOC_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE

# Explicit dependency needed for generated header file
drvAsynIseghalServiceVersion$(DEP): $(COMMON_DIR)/$(GENVERSION)
