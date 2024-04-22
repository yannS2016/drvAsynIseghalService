/* drvAsynIseghalService.cpp
 *
 * Driver for ISEG Spezialelektronik GmbH HV PS, iCS based systems (iCSmini 2 & CC24 controllers)
 * using isegHALService via asynPortDriver base class.
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
#include <vector>
#include <unistd.h>
#include <algorithm>

// EPICS includes
#include <epicsEvent.h>
#include <epicsExport.h>
#include <epicsExit.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTimer.h>
#include <epicsTypes.h>
#include <epicsAssert.h>
#include <epicsString.h>
#include <iocsh.h>

// ASYN includes
#include <asynPortDriver.h>

// local includes
#include "drvAsynIseghalService.h"

/* isegHAL includes */
#include <isegclientapi.h>

static const char *driverName = "drvAsynIseghalService";

#define DEFAULT_POLL_TIME 0.01
#define WRITE_LEN 20
#define FQN_LEN 34
//_____ D E F I N I T I O N S __________________________________________________
typedef  std::map<epicsUInt32, std::string>::const_iterator  itemIter;

/** Connect to iseg device
	*
	* This method finds or connect to a device.  It is called from the driver constructor.
	* \param [in]  name        drvAsynIseghalService internal name of the interface handle
	* \param [in]  interface   name of the hardware interface
	* \return      true if interface is already connected or if successfully connected
*/
int drvAsynIseghalService::devConnect( std::string const& name, std::string const& interface ) {

  std::vector< std::string >::iterator it;
  it = std::find( session_.begin(), session_.end(), name );
  if( it != session_.end() ) return true;

  IsegResult status = iseg_connect( name.c_str(), interface.c_str(), NULL );
  if ( ISEG_OK != status ) {
     return false;
  }

	// new iseghal session to iseg device.
	session_.push_back( name );

  /* iseg HAL starts collecting data from hardware after connect.
	* wait 5 secs to let all values 'initialize'
	*/
  epicsThreadSleep( 2 );

  return true;
}

/** Disconnect from iseg device session
	*
	* \param [in]  name   drvAsynIseghalService internal name of the interface handle
*/
int drvAsynIseghalService::devDisconnect( std::string const& name ) {
  std::vector< std::string >::iterator it;
  it = std::find( session_.begin(), session_.end(), name );

  if( it != session_.end() ) {
  	int status = iseg_disconnect( name.c_str() );
    if ( ISEG_OK != status ) {
  		return false;
  	}
    session_.erase( it );
  }
	return true;
}

/** Check if a device is connected
	*
	* \param [in]  name   drvAsynIseghalService internal name of the interface handle
*/
int drvAsynIseghalService::devConnected( std::string const& name ) {

  std::vector< std::string >::iterator it;
  it = std::find( session_.begin(), session_.end(), name );
  if( it != session_.end() ) return true;

  return false;
}

/** Called by epicsAtExit to shutdown iseghal session */
static void iseghalSessionShutdown( void* arg) {
	asynStatus status;
	drvAsynIseghalService *pPvt = (drvAsynIseghalService *) arg;
	status = pasynManager->lockPort(pPvt->self_);
  if(status!=asynSuccess)
        asynPrint(pPvt->self_, ASYN_TRACE_ERROR, "%s: cleanup locking error\n", pPvt->portName);

	asynPrint(pPvt->self_, ASYN_TRACE_WARNING, "drvAsynIseghalService: Closing iseghal %s session...\n", pPvt->portName);

	pPvt->devDisconnect(pPvt->portName);
  if(status==asynSuccess)
    pasynManager->unlockPort(pPvt->self_);

}

drvAsynIseghalService::drvAsynIseghalService( const char *portName, const char *interface, const char *icsCtrtype, epicsInt16 autoConnect )
	: asynPortDriver( portName,
		1, // maxAddr
		NUM_ISEGHAL_SERVICE_PARAMS,
		// Interface mask
		asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask | asynOctetMask,
		asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynOctetMask,
		ASYN_CANBLOCK | ASYN_MULTIDEVICE, // asynFlags.
		autoConnect, // Autoconnect
		0, // Default priority
		0 ), // Default stack size
		pollTime_(DEFAULT_POLL_TIME)
		
{
	static const char *functionName = "drvAsynIseghalService";


	deviceSession_ 	= epicsStrDup( portName );
	interface_  	= epicsStrDup( interface );
	deviceModel_	= epicsStrDup( icsCtrtype );
	self_ = pasynUserSelf;

  createParam( P_ISEGHAL_SERVICE_CHANVSET_STRING,             asynParamFloat64,       &P_ChanVset );
  createParam( P_ISEGHAL_SERVICE_CHANISET_STRING,             asynParamFloat64,       &P_ChanISet );
  createParam( P_ISEGHAL_SERVICE_CHANVMOM_STRING,             asynParamFloat64,       &P_ChanVMom );
  createParam( P_ISEGHAL_SERVICE_CHANIMOM_STRING,             asynParamFloat64,       &P_ChanIMom );
	 setDoubleParam (P_ChanVMom,       1.0);
	
	/* Register the shutdown function for epicsAtExit */
	epicsAtExit(iseghalSessionShutdown, (void*)this);
	itemIndex = 0;
}


asynStatus drvAsynIseghalService::writeFloat64( asynUser *pasynUser, epicsFloat64 value ){ return asynSuccess; }

/*
*  @brief   Called when asyn clients call pasynFloat64->read().
*
*  @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
*  @param   [in]  value      Address of the value to read
*
*  @return  in case of no error occured asynSuccess is returned. Otherwise
*           asynError or asynTimeout is returned. A error message is stored
*           in pasynUser->errorMessage.
*/
asynStatus drvAsynIseghalService::readFloat64( asynUser *pasynUser, epicsFloat64 *value ) {
	static const char *functionName = "readFloat64";
	
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char *propertyName;
	IsegItem item = EmptyIsegItem;
	epicsFloat64 dVal = 0;
	char* tmp;

	// Test if interface is connected to isegHAL server
  if( !devConnected( this->deviceSession_ ) ) {
			// If we have no camera, then just fail 
		return asynError;
  }

	getParamName(function, &propertyName);

	std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: " << "Reason: "<<function << "Property Name: " <<propertyName <<  "\033[0m" << std::endl;

  dVal = 9.5;
	
/* M2: set value, set double param, and then getdouble param
   *value = dVal;
   setDoubleParam (function, dVal );
	 status = (asynStatus) getDoubleParam(function, &dVal); 
*/
	
/*M1: set double parameter and calls base class method
	setDoubleParam (function,       dVal );
  return asynPortDriver::readFloat64(pasynUser, value);
*/
	//return asynSuccess;
}

/**
	* @brief   Called when asyn clients call pasynUInt32Digital->write().
	*
	* @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
	* @param   [in]  value      Value to write
	* @param   [in]  mask       Mask value to use when reading the value.
	*
	* @return  in case of no error occured asynSuccess is returned. Otherwise
	*          asynError or asynTimeout is returned. A error message is stored
	*          in pasynUser->errorMessage.
**/
asynStatus drvAsynIseghalService::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ) {

	
	return asynSuccess;
}

/**
	*  @brief   Called when asyn clients call pasynUInt32Digital->write().
	* 
	*  @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
	*  @param   [in]  value      Value to write
	*  @param   [in]  mask       Mask value to use when reading the value.
	* 
	*  @return  in case of no error occured asynSuccess is returned. Otherwise
	*           asynError or asynTimeout is returned. A error message is stored
	*           in pasynUser->errorMessage.
**/
asynStatus drvAsynIseghalService::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ) { 
 	return asynSuccess;
}

asynStatus drvAsynIseghalService::readInt32(asynUser *pasynUser, epicsInt32 *value) { 
	
	return asynSuccess; 
}
asynStatus drvAsynIseghalService::writeInt32(asynUser *pasynUser, epicsInt32 value) { 

	return asynSuccess; 
}
asynStatus drvAsynIseghalService::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual){ return asynSuccess; }



/**
* @brief       Connect driver to device
* @param [in]  pasynUser  pasynUser structure that encodes the reason and address.
* @return      in case of no error occured asynSuccess is returned. Otherwise
*              asynError or asynTimeout is returned. A error message is stored
*              in pasynUser->errorMessage.
*/
asynStatus drvAsynIseghalService::connect(asynUser *pasynUser) {
 std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: "<<"\033[0m" << std::endl;

	if( devConnected( deviceSession_ ) )	{
		epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "%s: Link to %s already open!", deviceSession_, interface_ );
    return asynError;
	}

  asynPrint( pasynUser, ASYN_TRACEIO_DRIVER,
             "%s: Open connection to %s\n", deviceSession_, interface_ );

	if( !devConnect(deviceSession_, interface_) ) {
    epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "%s: Can't open %s: %s", deviceSession_, interface_, strerror( errno ) );
    return asynError;
	}
	
/* check devices is responsive.
	* modelBA: iseg iCS based HV system controller model: cc24 for crate type
	* icsmini based system will expose the first crate controller module model i.e MICC.
*/

	char modelBA[15];
	// icsmini default
	strcpy(modelBA, "0.0.Article");

	if( strcmp( deviceModel_, "cc24" ) == 0 ) {
		strcpy(modelBA, "0.100.Article");
	}


  IsegItem model = iseg_getItem( deviceSession_, modelBA );
  if( strcmp( model.quality, ISEG_ITEM_QUALITY_OK ) != 0 ) {

    epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "\033[31;1m%s: Error while reading from device (Q: %s): %s\033[0m", deviceSession_, model.quality, strerror( errno ) );
    return asynError;
  }
	else {
				printf("\033[0;32m iCS controller model: %s\n\033[0m", model.value);
	}
	pasynManager->exceptionConnect( pasynUser );

	return asynSuccess;

}


/**
* @brief       Disconnect driver from device
* @param [in]  pasynUser  pasynUser structure that encodes the reason and address.
* @return      in case of no error occured asynSuccess is returned. Otherwise
*              asynError or asynTimeout is returned. A error message is stored
*              in pasynUser->errorMessage.
*/
asynStatus drvAsynIseghalService::disconnect(asynUser *pasynUser) {

	asynPrint( pasynUser, ASYN_TRACEIO_DRIVER,
             "%s: disconnect %s\n", deviceSession_, interface_ );

	if(!devDisconnect( deviceSession_ )) {
		epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "%s: cannot diconnect from %s ", deviceSession_, interface_ );
    return asynError;
	}

	pasynManager->exceptionDisconnect(pasynUser);

	return asynSuccess;

}
// Configuration routines. Called directly, or from the iocsh function below
extern "C" {

  /*
  * @brief   EPICS iocsh callable function to call constructor
  *          for the drvAsynIseghalService class.
  *
  * @param  [in]  portName The name of the asyn port driver to be created.
  * @param  [in]  interface Addr for HAL-service running on a CC24 or iCSmini (HAL protocol)
  * @param  [in]  mpod specify wether or not the system host wiener MPOD slaves.
  */
  int drvAsynIseghalServiceConfig( const char *portName, const char *interface, const char *icsCtrtype, epicsInt16 autoConnect ) {
    new drvAsynIseghalService( portName, interface, icsCtrtype, autoConnect );
    return( asynSuccess );
  }
  static const iocshArg initIseghalServiceArg0 = { "portName",		iocshArgString };
  static const iocshArg initIseghalServiceArg1 = { "interface",		iocshArgString };
  static const iocshArg initIseghalServiceArg2 = { "icsCtrtype",	iocshArgString };
  static const iocshArg initIseghalServiceArg3 = { "autoConnect",	iocshArgInt };
  static const iocshArg * const initIseghalServiceArgs[] = { &initIseghalServiceArg0, &initIseghalServiceArg1, &initIseghalServiceArg2, &initIseghalServiceArg3 };
  static const iocshFuncDef initIseghalServiceFuncDef = { "drvAsynIseghalServiceConfig", 4, initIseghalServiceArgs };

	static void initIseghalServiceCallFunc( const iocshArgBuf *args ) {
    drvAsynIseghalServiceConfig( args[0].sval, args[1].sval, args[2].sval, args[3].ival );
  }

	/*
  * @brief   Register functions to EPICS
  */
  void drvAsynIseghalServiceRegister( void ) {
      iocshRegister( &initIseghalServiceFuncDef, initIseghalServiceCallFunc );
  }
  epicsExportRegistrar( drvAsynIseghalServiceRegister );
}