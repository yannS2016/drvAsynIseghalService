/* drvAsynIseghalService.cpp
 *
 * Driver for ISEG Spezialelektronik GmbH MMC/MICC/MMS controller (iCSmini 2 & CC24 controller)
 * based HV PS system via  IsegHAL-Service Library using asynPortDriver base class
 *
 * Yann Stephen Mandza
 * February 14, 2024
*/

// ANSI C/C++ includes
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

// EPICS includes
#include <epicsEvent.h>
#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTimer.h>
#include <epicsTypes.h>
#include <iocsh.h>

// ASYN includes
#include <asynPortDriver.h>

// local includes
#include "drvAsynIseghalService.h"


static const char *driverName = "drvAsynIseghalService";

#define DEFAULT_POLL_TIME 0.01

drvAsynIseghalService::drvAsynIseghalService( const char *portName, const char *interface, bool mpod ) 
  : asynPortDriver( portName, 
                    1, // maxAddr
                    NUM_ISEGHAL_SERVICE_PARAMS,
										// Interface mask
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask | asynOctetMask,
                    // Interrupt mask
										asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynOctetMask,
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, // asynFlags.
                    1, // Autoconnect
                    0, // Default priority
                    0 ), // Default stack size
										mpod_(mpod),
										pollTime_(DEFAULT_POLL_TIME)
{
static const char *functionName = "drvAsynIseghalService";


	deviceName_ = epicsStrDup( portName );
	interface_  = epicsStrDup( interface );
}

asynStatus drvAsynIseghalService::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeFloat64( asynUser *pasynUser, epicsFloat64 value ){ return asynSuccess; }
asynStatus drvAsynIseghalService::readFloat64( asynUser *pasynUser, epicsFloat64 *value ){ return asynSuccess; }
asynStatus drvAsynIseghalService::readInt32(asynUser *pasynUser, epicsInt32 *value){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeInt32(asynUser *pasynUser, epicsInt32 value){ return asynSuccess; }
asynStatus drvAsynIseghalService::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual){ return asynSuccess; }

// Configuration routines. Called directly, or from the iocsh function below
extern "C" {
  
  //----------------------------------------------------------------------------
  //! @brief   EPICS iocsh callable function to call constructor
  //!          for the drvAsynIseghalService class.
  //!
  //! @param  [in]  portName The name of the asyn port driver to be created.
  //! @param  [in]  interface Addr for HAL-service running on a CC24 or iCSmini (HAL protocol)
  //! @param  [in]  mpod specify wether or not the system host wiener MPOD slaves.
  //----------------------------------------------------------------------------
  int drvAsynIseghalServiceConfig( const char *portName, const char *interface, bool mpod ) {
    new drvAsynIseghalService( portName, interface, mpod);
    return( asynSuccess );
  }
  static const iocshArg initIseghalServiceArg0 = { "portName", 	iocshArgString };
  static const iocshArg initIseghalServiceArg1 = { "interface",	iocshArgString };
  static const iocshArg initIseghalServiceArg2 = { "mpod",       iocshArgInt };
  static const iocshArg * const initIseghalServiceArgs[] = { &initIseghalServiceArg0, &initIseghalServiceArg1, &initIseghalServiceArg2 };
  static const iocshFuncDef initIseghalServiceFuncDef = { "drvAsynIseghalServiceConfig", 3, initIseghalServiceArgs };
  
	static void initIseghalServiceCallFunc( const iocshArgBuf *args ) {
    drvAsynIseghalServiceConfig( args[0].sval, args[1].sval, args[2].ival );
  }
	
	//----------------------------------------------------------------------------
  //! @brief   Register functions to EPICS
  //----------------------------------------------------------------------------
  void drvAsynIseghalServiceRegister( void ) {
    static int firstTime = 1;
    if ( firstTime ) {
      iocshRegister( &initIseghalServiceFuncDef, initIseghalServiceCallFunc );
      firstTime = 0;
    }
  }
  epicsExportRegistrar( drvAsynIseghalServiceRegister );
}