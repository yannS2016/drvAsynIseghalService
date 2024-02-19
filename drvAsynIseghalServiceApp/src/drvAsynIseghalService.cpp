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
	//printf("drvAsynIseghalService: Stopping %s... ", pPvt->portName);
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
                    // Interrupt mask
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
	self_ = pasynManager->createAsynUser(0,0);
	if( autoConnect ) {
		
		pasynManager->exceptionDisconnect( pasynUserSelf );
		
    asynStatus status = this->connect( pasynUserSelf );
		
    if( status ) {
      asynPrint( pasynUserSelf, ASYN_TRACE_ERROR,
                 "drvAsynIseghalService, error calling connect - %s\n",
                 pasynUserSelf->errorMessage );
    }
		/* Register the shutdown function for epicsAtExit */
    epicsAtExit(iseghalSessionShutdown, (void*)this);
  }

	itemIndex = 0;
}

asynStatus drvAsynIseghalService::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeFloat64( asynUser *pasynUser, epicsFloat64 value ){ return asynSuccess; }
asynStatus drvAsynIseghalService::readFloat64( asynUser *pasynUser, epicsFloat64 *value ){ return asynSuccess; }
asynStatus drvAsynIseghalService::readInt32(asynUser *pasynUser, epicsInt32 *value){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeInt32(asynUser *pasynUser, epicsInt32 value){ return asynSuccess; }
asynStatus drvAsynIseghalService::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual){ return asynSuccess; }

/*Drv info utility to skip whitespace from userparms 
* reusing the definitions from asynEpicsUtils*/
static const char *skipWhite(const char *pstart, int underscoreOk){
    const char *p = pstart;
    while(*p && (isspace((int)*p) || (underscoreOk && (*p=='_')))) p++; 
    return p;
}

int drvAsynIseghalService::hasIsegHalItem (const char *item) {
	std::vector< std::string >::iterator it;
	it = std::find( iseghalItemLookup.begin(), iseghalItemLookup.end(), std::string(item) );
  if( it != iseghalItemLookup.end() ) return true;
	return false;
}

asynStatus drvAsynIseghalService::drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize ) { 
  
	static const char *functionName = "drvUserCreate";
	
  /* the parameter is of format TYPE_item($(ADDR)) where Type is INT for int, DBL for double, or STR for string and DIG for UINT32DIGITAL
  * The fully qualified name for setting a new item value is item.ADDR, this is used to create the corresponding isegitem parameter.
	*/
  if (strlen(drvInfo) > 4 ) {

    size_t len;
    const char *p;
    const char *pnext;
		
    pnext = skipWhite(drvInfo,0);
		p = pnext;
		
		for(len=0; *p && (*p!='_') && (*p!=' '); len++, p++){}
		if(*p==0) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
            "invalid USERPARAM Must be TYPE _/ item(ADDR)");
        return asynError;
		}
		char paramtype[len+1];
		paramtype[len] = 0;
		strncpy(paramtype, pnext, len);

		//next is item
		pnext = skipWhite(p,1);
		p = pnext;
		if(*pnext==0) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
            "invalid USERPARAM Must be TYPE _/ item(ADDR)");
        return asynError;
		}

		for(len=0; *pnext && (*pnext!='('); len++, pnext++){}

		if(*pnext==0) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
            "invalid USERPARAM Must be TYPE _/ item(ADDR)");
        return asynError;
		}
		char iseghalItem[len+1];
		iseghalItem[len] = 0;
		strncpy(iseghalItem, p, len);
	
		//next is addr
		pnext = strstr(p,"(");
		if(!pnext) {
			  epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
            "invalid USERPARAM Must be TYPE _/ item(ADDR)");
        return asynError;
		}
		pnext++;
		pnext = skipWhite(pnext,0);
		p = pnext;
		for(len=0; *p && (*p!=' ') && (*p!=')'); len++, p++){}
		if(*p==0) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
            "invalid USERPARAM Must be TYPE _/ item(ADDR)");
        return asynError;
		}
		
		char iseghalItemAddr[len+1];
		iseghalItemAddr[len] = 0;
		strncpy(iseghalItemAddr, pnext, len);
		
			asynPrint( pasynUser, ASYN_TRACE_ERROR,
             "Fully Qualifid iseg Object: %s :%s :%s\n", paramtype, iseghalItem, iseghalItemAddr );

    // Check we have this iseghal item 
		if (!this->hasIsegHalItem(iseghalItem)) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                        "%s:%s: Parameter '%s' doesn't exist on iseghal item list\n",
                        driverName, functionName, iseghalItem);
      return asynError;
    }
		//dont forget the dot between property and address
		len = strlen(iseghalItem) + strlen(iseghalItemAddr) + 2;
		char fullyQualifyName[len];
		
		epicsSnprintf(fullyQualifyName, len+1, "%s.%s", iseghalItemAddr, iseghalItem);
		//0.0.1.VoltageSet

		int index;
		if (findParam(fullyQualifyName, &index) ) {
      // Check we have allocated enough space 
      if (itemIndex > NITEMS) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                        "%s:%s: Not enough space allocated to store all iseghal items, increase NITEMS\n",
                        deviceSession_, functionName);
            return asynError;
      }

			/* Make parameter of the correct type*/
			
			if( strcmp(paramtype, "INT") == 0) {
				createParam(fullyQualifyName, asynParamInt32, &(this->iseghalItems[itemIndex]));
			} else if (strcmp(paramtype, "DBL") == 0) {
				createParam(fullyQualifyName, asynParamFloat64, &(this->iseghalItems[itemIndex]));
			} else if (strcmp(paramtype, "STR") == 0) {
				createParam(fullyQualifyName, asynParamOctet, &(this->iseghalItems[itemIndex]));
			} else if (strcmp(paramtype, "DIG") == 0) {
				createParam(fullyQualifyName, asynParamUInt32Digital, &(this->iseghalItems[itemIndex]));
			} else {
				asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Expected Type is INT|DBL|STR|DIG. Got '%s'\n", driverName, functionName, paramtype);
        return asynError;
			}
			
			itemIndex++;
		}

	
	}

	return asynSuccess; 
}

/**
* @brief       Connect driver to device
* @param [in]  pasynUser  pasynUser structure that encodes the reason and address.
* @return      in case of no error occured asynSuccess is returned. Otherwise
*              asynError or asynTimeout is returned. A error message is stored
*              in pasynUser->errorMessage.
*/
asynStatus drvAsynIseghalService::connect(asynUser *pasynUser) {
	
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