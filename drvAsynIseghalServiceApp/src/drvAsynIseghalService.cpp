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

	/* Register the shutdown function for epicsAtExit */
	epicsAtExit(iseghalSessionShutdown, (void*)this);
	itemIndex = 0;
}


asynStatus drvAsynIseghalService::writeFloat64( asynUser *pasynUser, epicsFloat64 value ){ 
		std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: " << "\033[0m" << std::endl;
  
	return asynSuccess; 
}

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
	std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: " << "Reason: "<<function << "\033[0m" << std::endl;
 
	// Test if interface is connected to isegHAL server
  if( !devConnected( this->deviceSession_ ) ) {
			// If we have no camera, then just fail 
		return asynError;
  }

	getParamName(function, &propertyName);
	if (propertyName == NULL) {
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      deviceSession_, functionName, propertyName);
    return asynError;
  }

  IsegItemProperty itemProperty = iseg_getItemProperty( deviceSession_, propertyName );

  if(strcmp( itemProperty.access, "R" ) != 0) {

		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      deviceSession_, functionName, propertyName, itemProperty.access );
		// Update alarms status.
		pasynUser->alarmStatus = 1; 		// READ
		pasynUser->alarmSeverity = 3; 	// INVALID
		
    return asynError;
  }

	item = iseg_getItem(deviceSession_, propertyName);

  if( strcmp( item.quality, ISEG_ITEM_QUALITY_OK ) != 0 ) {
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "%s:%s: Error while reading from %s : %s",
      deviceSession_, functionName, propertyName, strerror( errno ) );
			
		// Update alarms status.
		pasynUser->alarmStatus = 1; 		// READ
		pasynUser->alarmSeverity = 3; 	// INVALID
		
    return asynError;
	}

	epicsUInt32 seconds = 0;
  epicsUInt32 microsecs = 0;
	
  if( sscanf( item.timeStampLastChanged, "%u.%u", &seconds, &microsecs ) != 2 ) {
		return asynError;
	}

  epicsTimeStamp time;
  time.secPastEpoch = seconds - POSIX_TIME_AT_EPICS_EPOCH;
  time.nsec = microsecs * 100000;

  // set new paramter value and update record val field
  dVal = (epicsFloat64)strtod (item.value, NULL);
  *value = dVal;
  setDoubleParam (function, dVal );
  pasynUser->timestamp = time;
  status = (asynStatus) getDoubleParam(function, &dVal); 

/*	setDoubleParam (function,       dVal );
  return asynPortDriver::readFloat64(pasynUser, value);*/
  if( status )
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s: status=%d, function=%d, value=%f",
                   deviceSession_, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s: function=%d, value=%f\n", 
              deviceSession_, functionName, function, *value );
							
	// Update alarms status.
	pasynUser->alarmStatus = 0;
	pasynUser->alarmSeverity = 0;
  
	return asynSuccess;
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


	static const char *functionName = "readUInt32Digital";
	
  int function = pasynUser->reason;
	
	std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: " << "Reason: "<<function << "\033[0m" << std::endl;
	
	
  const char *propertyName;
  asynStatus status = asynSuccess;
	
	IsegItem item = EmptyIsegItem;
	
	epicsUInt32 iVal = 0;
	
 // Test if interface is connected to isegHAL server
  if( !devConnected( this->deviceSession_ ) ) {
			// If we have no camera, then just fail 
			return asynError;
  }

	getParamName(function, &propertyName);

	if (propertyName == NULL) {
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      deviceSession_, functionName, propertyName);
      return asynError;
  }

  IsegItemProperty itemProperty = iseg_getItemProperty( deviceSession_, propertyName );

  if(strcmp( itemProperty.access, "R" ) != 0) {

		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      deviceSession_, functionName, propertyName, itemProperty.access );
		// Update alarms status.
		pasynUser->alarmStatus = 1; 		// READ
		pasynUser->alarmSeverity = 3; 	//INVALID
		
    return asynError;
  }


	item = iseg_getItem(deviceSession_, propertyName);

  if( strcmp( item.quality, ISEG_ITEM_QUALITY_OK ) != 0 ) {
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "%s:%s: Error while reading from %s : %s",
      deviceSession_, functionName, propertyName, strerror( errno ) );
			
		// Update alarms status.
		pasynUser->alarmStatus = 1; 		// READ
		pasynUser->alarmSeverity = 3; 	// INVALID
		
    return asynError;
	}

  
	// Parameter object contains the fully qualified name to read out an iseg item from data cache
	epicsUInt32 seconds = 0;
  epicsUInt32 microsecs = 0;
	
  if( sscanf( item.timeStampLastChanged, "%u.%u", &seconds, &microsecs ) != 2 ) {
		return asynError;
	}

  epicsTimeStamp time;
  time.secPastEpoch = seconds - POSIX_TIME_AT_EPICS_EPOCH;
  time.nsec = microsecs * 100000;


  iVal = (epicsUInt32)atoi(item.value);
  *value = iVal;
		status = (asynStatus) setUIntDigitalParam(function, iVal, mask );
		pasynUser->timestamp = time;
	// update record val field
	status = (asynStatus) getUIntDigitalParam(function, &iVal, mask );

/*	setDoubleParam (function,       dVal );
  return asynPortDriver::readFloat64(pasynUser, value);*/
	
  if( status ) 
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s: status=%d, function=%d, value=%d",
                   deviceSession_, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s: function=%d, value=%d\n", 
              deviceSession_, functionName, function, *value );
							
	// Update alarms status.
	pasynUser->alarmStatus = 0; 		// READ
	pasynUser->alarmSeverity = 0; 	//INVALID		
	
	return status;
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

	static const char *functionName = "writeUInt32Digital";
	
  int function = pasynUser->reason;
	
  const char *propertyName;
	
	epicsTimeStamp timeStamp; 
	getTimeStamp(&timeStamp);
	
  asynStatus status = asynSuccess;
	
	IsegItem item = EmptyIsegItem;
	std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: " << "Reason: "<<function << "\033[0m" << std::endl;
  
	
	char sVal[20];

	itemIter it = isegHalItemsLookup.find( function );
	
	if (it != isegHalItemsLookup.end() ) {
		
		if(it->second == "Status") {
			std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() 
								<< " thread: " << "Write Item: " << it->second <<"\033[0m" << std::endl;
			return asynSuccess;
		}
	} else {
		
		return asynError;
	}
	
	// Test if interface is connected to isegHAL server
  if( !devConnected( this->deviceSession_ ) ) {
			// If we have no camera, then just fail 
			return asynError;
  }

	getParamName(function, &propertyName);

	if (propertyName == NULL) {
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      deviceSession_, functionName, propertyName);
      return asynError;
  }

  IsegItemProperty itemProperty = iseg_getItemProperty( deviceSession_, propertyName );

  if(strcmp( itemProperty.access, "RW" ) != 0) {

		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      deviceSession_, functionName, propertyName, itemProperty.access );
		// Update alarms status.
		pasynUser->alarmStatus = 2; 		// READ
		pasynUser->alarmSeverity = 3; 	//INVALID
		
    return asynError;
  }

	epicsSnprintf(  sVal, WRITE_LEN, "%d", (value & mask) );

  if(iseg_setItem(deviceSession_, propertyName,  sVal) != ISEG_OK ) {
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "%s:%s: Error while writing value '%d' for %s : %s",
      deviceSession_, functionName, value, propertyName, strerror( errno ) );
			
		// Update alarms status.
		pasynUser->alarmStatus = 2; 		// READ
		pasynUser->alarmSeverity = 3; 	// INVALID
		
    return asynError;
	}
	
	// update value of parameter
	status = (asynStatus) setUIntDigitalParam(function, value, mask );
	/* Do callbacks so higher layers see any changes */
	status = (asynStatus) callParamCallbacks();
	
  if( status ) {
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s: status=%d, function=%d, value=%d",
                   deviceSession_, functionName, status, function, value );
		// Update alarms status.
		pasynUser->alarmStatus = 2; 		// Write
		pasynUser->alarmSeverity = 3; 	// INVALID

	}

  else {
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s: function=%d, value=%d\n", 
              deviceSession_, functionName, function, value );
	}       

  /* Get the current timestamp */
  getTimeStamp(&timeStamp);
  pasynUser->timestamp = timeStamp;

	return status; 
}


asynStatus drvAsynIseghalService::readInt32(asynUser *pasynUser, epicsInt32 *value) { 
  
	static const char *functionName = "readInt32";
	
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char *propertyName;
	IsegItem item = EmptyIsegItem;
	epicsInt32 iVal = 0;
	char* tmp;

	std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: " << "Reason: "<<function << "\033[0m" << std::endl;
 
	// Test if interface is connected to isegHAL server
  if( !devConnected( this->deviceSession_ ) ) {
			// If we have no camera, then just fail 
		return asynError;
  }

	getParamName(function, &propertyName);

	if (propertyName == NULL) {
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      deviceSession_, functionName, propertyName);
    return asynError;
  }

  IsegItemProperty itemProperty = iseg_getItemProperty( deviceSession_, propertyName );

  if(strcmp( itemProperty.access, "RW" ) != 0) {

		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      deviceSession_, functionName, propertyName, itemProperty.access );
		// Update alarms status.
		pasynUser->alarmStatus = 1; 		// READ
		pasynUser->alarmSeverity = 3; 	// INVALID
		
    return asynError;
  }

	item = iseg_getItem(deviceSession_, propertyName);

  if( strcmp( item.quality, ISEG_ITEM_QUALITY_OK ) != 0 ) {
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
      "%s:%s: Error while reading from %s : %s",
      deviceSession_, functionName, propertyName, strerror( errno ) );
			
		// Update alarms status.
		pasynUser->alarmStatus = 1; 		// READ
		pasynUser->alarmSeverity = 3; 	// INVALID
		
    return asynError;
	}

	epicsUInt32 seconds = 0;
  epicsUInt32 microsecs = 0;
	
  if( sscanf( item.timeStampLastChanged, "%u.%u", &seconds, &microsecs ) != 2 ) {
		return asynError;
	}

  epicsTimeStamp time;
  time.secPastEpoch = seconds - POSIX_TIME_AT_EPICS_EPOCH;
  time.nsec = microsecs * 100000;

  // set new paramter value and update record val field
		iVal = (epicsInt32)atoi(item.value);
  *value = iVal;
		setIntegerParam(function, iVal);
  pasynUser->timestamp = time;
  status = (asynStatus) getIntegerParam(function, &iVal); 


  if( status )
		epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "%s:%s: status=%d, function=%d, value=%d",
                   deviceSession_, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE, 
               "%s:%s: function=%d, value=%d\n", 
              deviceSession_, functionName, function, *value );
							
	// Update alarms status.
	pasynUser->alarmStatus = 0;
	pasynUser->alarmSeverity = 0; 
	
	return asynSuccess;
}
asynStatus drvAsynIseghalService::writeInt32(asynUser *pasynUser, epicsInt32 value) { 

	static const char *functionName = "writeInt32";
	
  int function = pasynUser->reason;
	
  const char *propertyName;
	
	epicsTimeStamp timeStamp; 
	getTimeStamp(&timeStamp);
	
  asynStatus status = asynSuccess;
	
	IsegItem item = EmptyIsegItem;
	std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: " << "Reason: "<<function << "\033[0m" << std::endl;
  

	char sVal[20];
	getParamName(function, &propertyName);


	return asynSuccess; 
}
asynStatus drvAsynIseghalService::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason) { 
	
	std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: " << "\033[0m" << std::endl;
  
	return asynSuccess; 
}
asynStatus drvAsynIseghalService::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual) { 
			std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: " << "\033[0m" << std::endl;
  
	return asynSuccess; 
}

/*Drv info utility to skip whitespace from userparms
* reusing the definitions from asynEpicsUtils*/
static const char *skipWhite(const char *pstart, int underscoreOk){
    const char *p = pstart;
    while(*p && (isspace((int)*p) || (underscoreOk && (*p=='_')))) p++;
    return p;
}

int drvAsynIseghalService::hasIsegHalItem (const char *item) {
	std::vector< std::string >::iterator it;
	it = std::find( validIsegHalItems.begin(), validIsegHalItems.end(), std::string(item) );
  if( it != validIsegHalItems.end() ) return true;
	return false;
}

asynStatus drvAsynIseghalService::drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize ) {

	static const char *functionName = "drvUserCreate";

  /* the parameter is of format TYPE_item($(ADDR)) where Type is INT for int, DBL for double, or STR for string and DIG for UINT32DIGITAL
  * The fully qualified name (FQN) for gettting/setting an item value is ADDR.item, this is used to create the corresponding iseghalitem parameter.
	*/
	size_t len;
  const char *p;
  const char *pnext;
	bool sysItem =  0;
  if (strlen(drvInfo) > 4 ) {
		
    pnext = skipWhite(drvInfo,0);
		p = pnext;
		
		for(len=0; *pnext && isalpha(*pnext); len++, pnext++){}
		
		if(*pnext==0) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
            "invalid USERPARAM Must be TYPE _/ 'I'tem(ADDR)");
        return asynError;
		}
		
		char halItemType[len+1];
		halItemType[len] = 0;
		strncpy(halItemType, p, len);

		//next is item
		p = skipWhite(pnext,1);
		pnext = p;

		if(*p==0 || !isupper(*p)) {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
            "invalid USERPARAM Must be TYPE _/ 'I'tem(ADDR)");
        return asynError;
		}
		
		
		for(len=0; *p && isalpha(*p); len++, p++){}
		// 3: we expect 2  additional character for control param address
		int len_ = len+4;
		char halItem[len];
		char tmp[len_];

		strncpy(halItem, pnext, len);
		*(halItem+len)=0x0; // terminate string
						/* std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() 
									<< " thread: " << "Item:" << halItem << "\033[0m" << std::endl; */
    // Check we have this iseghal item
		if (!this->hasIsegHalItem(halItem)) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "\033[0;33m %s:%s: Parameter '%s' doesn't exist on iseghal item list\n\033[0m",
                driverName, functionName, halItem);
					// Update alarms status.
			pasynUser->alarmStatus = 17; 		// UDF
			pasynUser->alarmSeverity = 3; 	// INVALID
      return asynError;
    }
		

		if(strcmp(halItem, "Control") == 0) {

			p++; // skip this ':'
			pnext = p;
			
			for(len=0; *p && isdigit(*p); len++, p++){}
			char ctrlAddr[len+1];
			strncpy(ctrlAddr, pnext, len);
			epicsSnprintf(tmp, len_, "%s:%s", halItem, ctrlAddr);
			*(tmp+len_) = 0x0;

		
		} else {
			strcpy(tmp, halItem);
			*(tmp+len_) = 0x0;
		}
		
		char halItemFQN[FQN_LEN];
		//next is addr, no addr for system items
		pnext = p;
		pnext = strstr(p,"(");
		if(!pnext) {
			//not a system item?
			if(strcmp(tmp, "Status") == 0 || strcmp(tmp, "CrateNumber") == 0
				|| strcmp(tmp, "ModuleNumber") == 0 || strcmp(tmp, "CycleCounter") == 0 || strcmp(tmp, "Configuration") == 0
				|| strcmp(tmp, "Read") == 0 || strcmp(tmp, "Write ") == 0 || strcmp(tmp, "LogLevel") == 0 
				|| strcmp(tmp, "LogPath") == 0 || strcmp(tmp, "LiveInsertionMode") == 0
				|| strcmp(tmp, "SaveConfiguration") == 0 || strcmp(tmp, "LiveInsertionMode") == 0
				|| strcmp(tmp, "ServerVersion") == 0 || strcmp(tmp, "NetworkTimeout") == 0 || strcmp(tmp, "SessionName") == 0) {
				// Create fully qualified object for system items
				strcpy(halItemFQN, tmp);
	
			} else {
				epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
            "invalid USERPARAM Must be TYPE _/ 'I'tem(ADDR)");
				pasynUser->alarmStatus = 17; 		// UDF
				pasynUser->alarmSeverity = 3; 	// INVALID
        return asynError;
			}

		} else {
			pnext++;
			p = skipWhite(pnext,0);
		
			for(len=0; *p && (*p!=' ') && (*p!=')'); len++, p++){}
			
			if(*p==0) {
				epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
					"invalid USERPARAM Must be TYPE _/ item(ADDR)");
							return asynError;
			}
			char itemAddr[len+1];
			strncpy(itemAddr, pnext, len);
			*(itemAddr+len)=0;
	
			// Create fully qualified object for system items
			epicsSnprintf(halItemFQN, FQN_LEN, "%s.%s", itemAddr, tmp);
		}

/* 			std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() 
								<< " thread: " <<"halItemFQN: " << halItemFQN << "\033[0m" << std::endl;
		 */




		
		int index;
		if (findParam(halItemFQN, &index) ) {
			//Make parameter of the correct type
			if( strcmp(halItemType, "INT") == 0) {
					createParam(halItemFQN, asynParamInt32, &itemIndex);
			} else if (strcmp(halItemType, "DBL") == 0) {
					createParam(halItemFQN, asynParamFloat64, &itemIndex);
// needed for  M1 using set and get method to update rec val field
         // setDoubleParam (itemIndex,       0.0);
			} else if (strcmp(halItemType, "STR") == 0) {
					createParam(halItemFQN, asynParamOctet, &itemIndex);
			} else if (strcmp(halItemType, "DIG") == 0) {
					createParam(halItemFQN, asynParamUInt32Digital, &itemIndex);
			} else {
					asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
											"%s:%s: Expected Type is INT|DBL|STR|DIG. Got '%s'\n", driverName, functionName, halItemType);
					return asynError;
			}
			
			pasynUser->reason = itemIndex;
			isegHalItemsLookup.insert( std::make_pair( itemIndex, std::string(tmp) ) );
			
			std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: " << "FQN: " << halItemFQN << " index: " << itemIndex <<"\033[0m" << std::endl;
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