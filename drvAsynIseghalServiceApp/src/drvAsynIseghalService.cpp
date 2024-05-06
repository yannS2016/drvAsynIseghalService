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
#include <iocsh.h>
#include <initHooks.h>
#include <epicsStdio.h>
#include <cantProceed.h>

// ASYN includes
#include <asynPortDriver.h>
#include <asynCommonSyncIO.h>

// local includes
#include "drvAsynIseghalService.h"

static const char *driverName = "drvAsynIseghalService";

#define WRITE_LEN 20
#define FQN_LEN 34

typedef  std::map<epicsUInt32, std::string>::const_iterator  itemIter;
epicsMutexId      hookMutexId;
static drvIsegHalPollerThread *drvIsegHalPollerThread_ = NULL;

// stop access of iseghal instance outside of portthread
bool initStatus = 0;

std::vector<std::string> validIsegHalItems
{
   "Status",
   "EventStatus",
   "EventMask",
   "Control",
   "CrateNumber",
   "CrateList",
   "ModuleNumber",
   "CycleCounter",
   "LogLevel",
   "LogPath",
   "LiveInsertionMode",
   "SaveConfiguration",
   "EthName",
   "EthAddress",
   "ServerVersion",
   "NetworkTimeout",
   "SessionName",
   // Can Line items
   "BitRate",
   // Crate items
   "Connected",
   "Alive",
   "PowerOn",
   "FanSpeed",
   "SerialNumber",
   "DeviceClass",
   "FirmwareRelease",
   "FirmwareName",
   "Temperature" ,
   "Supply",
   "Article",

   // device/module items
   "SerialNumber",
   "SampleRate",
   "DigitalFilter",
   "VoltageRampSpeed",
   "CurrentRampSpeed",
   "VoltageLimit"  ,
   "CurrentLimit"  ,
   "DoClear",
   "FineAdjustment",
   "KillEnable" ,

  // MICC option
   "HighVoltageOk",

   // Channel items
   "VoltageSet",
   "CurrentSet",
   "VoltageMeasure"  ,
   "CurrentMeasure"  ,
   "VoltageBounds",
   "CurrentBounds",
   "VoltageNominal",
   "CurrentNominal",
   "DelayedTripAction",
   "DelayedTripTime",
   "ExternalInhibitAction",
   "TemperatureTrip",

   // Option Voltage controlled by temperature (VCT)
   "TemperatureExternal",
   "VctCoefficient",

   // Option STACK
   "Resistance",
   "VoltageRampPriority",
   "VoltageBottom",

   // Option Reversible
   "OutputMode",
   "OutputModeList",
   "OutputPolarity",
   "OutputPolarityList",
   "VoltageMode",
   "VoltageModeList",
   "CurrentMode",
   "CurrentModeList",
   "VoltageRampSpeedUp",
   "VoltageRampSpeedDown",
   "VoltageRampSpeedMin",
   "VoltageRampSpeedMax",
   "CurrentRampSpeedUp",
   "CurrentRampSpeedDown",
   "CurrentRampSpeedMin",
   "CurrentRampSpeedMax",
   "VoltageTerminalMeasure",
   "VoltageRiseRate",
   "VoltageFallRate",
   "VoltageLowTrip",
   "VoltageTrip",
   "VoltageTerminalTrip",
   "CurrentTrip",
   "PowerTrip",
   "VoltageTripMax",
   "VoltageTerminalTripMax",
   "CurrentTripMax",
   "TemperatureTripMax",
   "PowerTripMax",
   "VoltageLowTripTime",
   "VoltageTripTime",
   "VoltageTerminalTripTime",
   "CurrentTripTime",
   "TemperatureTripTime",
   "PowerTripTime",
   "TimeoutTripTime",
   "Group",
   "Name",
   "TripAction",
   "VoltageLowTripAction",
   "VoltageTripAction",
   "VoltageTerminalTripAction",
   "CurrentTripAction",
   "TemperatureTripAction",
   "PowerTripAction",
   "TimeoutTripAction",
   "UserConfigFlags",
};

/** Connect to iseg device
  *
  * This method finds or connect to a device.  It is called from the driver constructor.
  * \param [in]  name        drvAsynIseghalService internal name of the interface handle
  * \param [in]  interface   name of the hardware interface
  * \return      true if interface is already connected or if successfully connected
*/
int drvAsynIseghalService::devConnect( std::string const& name, std::string const& interface ) {

  if (devInitOk_) {
    if( iseg_isConnError(name.c_str()) ) {
    // We session was disconnected! attempt reconexion n times
    // otherwise close the connexion and open a new one. we use autoconnect freq here.
        asynPrint( pasynUserSelf, ASYN_TRACEIO_DRIVER,"%s: Attempting reconnexion... to %s\n", name.c_str(), interface.c_str() );

        IsegResult status = iseg_reconnect(name.c_str(), interface.c_str() );
        if ( ISEG_OK != status ) return false;
        conMan_ = 0;
        drvIsegHalPollerThread_->changeIntervall(2.0);
    }
  }
  else {
    asynPrint( pasynUserSelf, ASYN_TRACEIO_DRIVER,"%s: Open connection to %s\n", name.c_str(), interface.c_str() );
    IsegResult status = iseg_connect( name.c_str(), interface.c_str(), NULL );
    if ( ISEG_OK != status ) return false;
    // new iseghal session to iseg device.
    openedSessions.push_back( name );
    devInitOk_ = true;
  }

  /* iseg HAL starts collecting data from hardware after connect.
    * wait 5 secs to let all values 'initialize'
  */
  epicsThreadSleep( 5 );
  return true;
}


/** Disconnect from iseg device session
  * \param [in]  name   drvAsynIseghalService internal name of the interface handle
*/
int drvAsynIseghalService::devDisconnect( std::string const& name ) {

    int status = iseg_disconnect( name.c_str() );
    if ( status != ISEG_OK  ) return false;
    openedSessions.clear();
    return true;
}


/** Check if a device is connected
  * name:   drvAsynIseghalService internal name of the interface handle
*/
int drvAsynIseghalService::devConnected( std::string const& name ) {
  std::vector< std::string >::iterator it;
  it = std::find( openedSessions.begin(), openedSessions.end(), name );
  // cant access hal object unless connect was successfully called
  if(!devInitOk_) return false;
  return ( it != openedSessions.end() && !iseg_isConnError(name.c_str()));
}

/** Called by epicsAtExit to shutdown iseghal session */
static void iseghalSessionShutdown( void* pDrv) {
  asynStatus status;
  drvAsynIseghalService *pPvt = (drvAsynIseghalService *) pDrv;
  pPvt->iseghalExiting_ = true;
  status =  pasynCommonSyncIO->disconnectDevice(pPvt->exitUser_);
  if(status!=asynSuccess)
        asynPrint(pPvt->exitUser_, ASYN_TRACE_ERROR, "%s: drvAsynIseghalService cleanup  error %s\n", pPvt->portName, pPvt->exitUser_->errorMessage);
}

// all record have been initialized
static void startPolling(initHookState state) {
    if (state == initHookAfterIocRunning) {
      initStatus = 1;
      // all record init done: interrupt list filled
      if(drvIsegHalPollerThread_ != NULL) {
        drvIsegHalPollerThread_->thread.start();
      }
    }
}

drvAsynIseghalService::drvAsynIseghalService( const char *portName, const char *interface, const char *icsCtrtype, epicsInt16 autoConnect )
  : asynPortDriver( portName,
    1, // maxAddr
    NITEMS,
    // Interface mask
    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynOctetMask | asynDrvUserMask,
    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynOctetMask,
    ASYN_CANBLOCK |ASYN_MULTIDEVICE, // asynFlags.
    1, // Autoconnect
    0, // Default priority
    0 ), // Default stack size
    iseghalExiting_(false),
    conMan_(false),
    iseghalReconAttempt_(10)

{
    static const char *functionName = "drvAsynIseghalService";

    session_  		= epicsStrDup( portName );
    interface_    = epicsStrDup( interface );
    deviceModel_	= epicsStrDup( icsCtrtype );
		itemReason_ 	= 0;
    devInitOk_ 		= 0;
    exitUser_ 		= pasynManager->createAsynUser(0, 0);

    asynStatus status;

    /* Connect this user to the port so we can use it to cleanup connexion at exit*/
    status = pasynCommonSyncIO->connect(session_, 0, &exitUser_, NULL);
    if (status) {
      printf("%s:%s: pasynCommonSyncIO->connect failure, status=%d\n", driverName, functionName, status);
      return;
    }

		/* iseg HAL starts collecting data from hardware after connect.
			* allow 5 secs timeout to let all values 'initialize'
		*/
    pasynManager->setAutoConnectTimeout(5.0);

		hookMutexId = epicsMutexCreate();
    /* Register the shutdown function for epicsAtExit */
    epicsAtExit(iseghalSessionShutdown, (void*)this);

		/* Register the start polling fucntion: we wait till ioc is in running state before starting */
    initHookRegister(startPolling);

		// instantiate the polling thread, but dont start yet.
    drvIsegHalPollerThread_ = new drvIsegHalPollerThread(this);
    if(!drvIsegHalPollerThread_) return;

}

char *drvAsynIseghalService::getSessionName (){
  return session_;
}

asynStandardInterfaces drvAsynIseghalService::getAsynStdIface()
{
  return asynStdInterfaces;
}

asynStatus drvAsynIseghalService::getIsegHalItem (asynUser *isegHalUser, IsegItem *item)
{
  static const char *functionName = "getIsegHalItem";
  const char *propertyName;

  asynStatus status = asynSuccess;

  epicsInt16 function = isegHalUser->reason;

  printf("\033[0;33m%s : (%s)\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__);

  if(!initStatus) return asynError;

  if (iseghalExiting_) return asynSuccess;

  getParamName(function, &propertyName);

  if (propertyName == NULL) {
    epicsSnprintf( isegHalUser->errorMessage, isegHalUser->errorMessageSize,
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      session_, functionName, propertyName);
    return asynError;
  }

  // Test if interface is connected to isegHAL server
  if( !devConnected( session_ ) ) {
    if(devInitOk_ && !conMan_) {
      this->disconnect(isegHalUser);
      drvIsegHalPollerThread_->changeIntervall(10.0);
    }
    return asynError;
  }

  IsegItemProperty itemProperty = iseg_getItemProperty( session_, propertyName );
	//printf("\033[0;33m%s : %s : propertyName %s: Permission %s\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, propertyName, itemProperty.access);
  if(!(strcmp( itemProperty.access, "R" ) == 0 || strcmp( itemProperty.access, "RW" ) == 0) ) {

    epicsSnprintf( isegHalUser->errorMessage, isegHalUser->errorMessageSize,
      "\033[31;1m%s:%s Wrong item '%s' permission: Access right: '%s'\033[0m\n",
      session_, functionName, propertyName, itemProperty.access );

    isegHalUser->alarmStatus = 1;
    isegHalUser->alarmSeverity = 3;
    return asynError;
  }

  *item = iseg_getItem(session_, propertyName);

  if( strcmp( item->quality, ISEG_ITEM_QUALITY_OK ) != 0 ) {
    epicsSnprintf( isegHalUser->errorMessage,isegHalUser->errorMessageSize,"\033[31;1m%s:%s Error reading from device (Q: %s): %s\033[0m",
									session_, functionName, item->quality, strerror( errno ) );

    isegHalUser->alarmStatus = 1;
    isegHalUser->alarmSeverity = 3;

    return asynError;
  }

  epicsUInt32 seconds = 0;
  epicsUInt32 microsecs = 0;
  epicsTimeStamp time;

  if( sscanf( item->timeStampLastChanged, "%u.%u", &seconds, &microsecs ) != 2 ) {
    epicsSnprintf( isegHalUser->errorMessage, isegHalUser->errorMessageSize,
      "%s:%s: Error while reading from %s : %s",
      session_, functionName, propertyName, strerror( errno ) );

    isegHalUser->alarmStatus = 1;
    isegHalUser->alarmSeverity = 3;
    return asynError;
  }

  time.secPastEpoch = seconds - POSIX_TIME_AT_EPICS_EPOCH;
  time.nsec = microsecs * 100000;
  isegHalUser->timestamp = time;

  return status;
}



asynStatus drvAsynIseghalService::writeFloat64( asynUser *pasynUser, epicsFloat64 value )
{

  static const char *functionName = "writeFloat64";

  int function = pasynUser->reason;
  const char *propertyName;
  epicsTimeStamp timeStamp;
  asynStatus status = asynSuccess;
  char sVal[20];

  printf("\033[0;33m%s : (%s) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, function);
  if (iseghalExiting_) return asynSuccess;
  // Test if interface is connected to isegHAL server
  if( !devConnected( session_ ) ) {
    if(devInitOk_ && !conMan_) {
      this->disconnect(pasynUser);
    }
    return asynError;
  }

  getParamName(function, &propertyName);

  if (propertyName == NULL) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      session_, functionName, propertyName);
      return asynError;
  }

  IsegItemProperty itemProperty = iseg_getItemProperty( session_, propertyName );

  if(strcmp( itemProperty.access, "RW" ) != 0) {

    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      session_, functionName, propertyName, itemProperty.access );

    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;

    return asynError;
  }

  epicsSnprintf( sVal, WRITE_LEN, "%f", value);

  if(iseg_setItem(session_, propertyName,  sVal) != ISEG_OK ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "%s:%s: Error writing value '%f' for %s : %s",
      session_, functionName, value, propertyName, strerror( errno ) );

    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;

    return asynError;
  }

  pasynUser->alarmStatus = 0;
  pasynUser->alarmSeverity = 0;

  getTimeStamp(&timeStamp);
  pasynUser->timestamp = timeStamp;

  // update value of parameter
  status = (asynStatus) setDoubleParam(function, value);
	status = (asynStatus) callParamCallbacks();
  if( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                   "%s:%s: status=%d, function=%d, value=%f",
                   session_, functionName, status, function, value );
  else
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE,
               "%s:%s: function=%d, value=%f\n",
              session_, functionName, function, value );


  return status;
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
asynStatus drvAsynIseghalService::readFloat64( asynUser *pasynUser, epicsFloat64 *value )
{
  static const char *functionName = "readFloat64";

  epicsInt16 function = pasynUser->reason;
  epicsFloat64 dVal = 0;

  IsegItem item = EmptyIsegItem;
  asynStatus status = asynSuccess;

  printf("\033[0;33m%s : (%s) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, function);
	status = getIsegHalItem (pasynUser, &item);
	if(status != asynSuccess) return asynError;

  // set new paramter value and update record val field
  dVal = (epicsFloat64)strtod (item.value, NULL);
  *value = dVal;
  setDoubleParam (function, dVal );
  status = (asynStatus) getDoubleParam(function, &dVal);

  if( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                   "%s:%s: status=%d, function=%d, value=%f",
                   session_, functionName, status, function, *value );
  else
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE,
               "%s:%s: function=%d, value=%f\n",
              session_, functionName, function, *value );

  pasynUser->alarmStatus = 0;
  pasynUser->alarmSeverity = 0;

  return status;
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
asynStatus drvAsynIseghalService::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask )
{

  static const char *functionName = "readUInt32Digital";

  epicsInt16 function = pasynUser->reason;
  printf("\033[0;33m%s : (%s) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, function);

  asynStatus status = asynSuccess;
  IsegItem item = EmptyIsegItem;
  epicsUInt32 iVal = 0;

	status = getIsegHalItem (pasynUser, &item);
	if(status != asynSuccess) return asynError;

  iVal = (epicsUInt32)atoi(item.value);
  *value = iVal;
  status = (asynStatus) setUIntDigitalParam(function, iVal, mask );
  pasynUser->alarmStatus = 0;
  pasynUser->alarmSeverity = 0;
  status = (asynStatus) getUIntDigitalParam(function, &iVal, mask );

  if( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                   "%s:%s: status=%d, function=%d, value=%d",
                   session_, functionName, status, function, *value );
  else
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE,
               "%s:%s: function=%d, value=%d\n",
              session_, functionName, function, *value );

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
asynStatus drvAsynIseghalService::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask )
{
  static const char *functionName = "writeUInt32Digital";

  epicsInt16 function = pasynUser->reason;
  const char *propertyName;
  epicsTimeStamp timeStamp;
  asynStatus status = asynSuccess;

  printf("\033[0;33m%s : (%s) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, function);

  if (iseghalExiting_) return asynSuccess;

  // Test if interface is connected to isegHAL server
  if( !devConnected( session_ ) ) {
    if(devInitOk_ && !conMan_) {
      this->disconnect(pasynUser);
    }
    return asynError;
  }

  getTimeStamp(&timeStamp);

  char sVal[20];

  itemIter it = isegHalItemsLookup.find( function );

  if (it != isegHalItemsLookup.end() ) {

    if(it->second == "Status") {
      return asynSuccess;
    }
  }


  getParamName(function, &propertyName);

  if (propertyName == NULL) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      session_, functionName, propertyName);
      return asynError;
  }

  IsegItemProperty itemProperty = iseg_getItemProperty( session_, propertyName );

  if(strcmp( itemProperty.access, "RW" ) != 0) {

    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      session_, functionName, propertyName, itemProperty.access );

    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;

    return asynError;
  }

  epicsSnprintf( sVal, WRITE_LEN, "%d", (value & mask) );

  if(iseg_setItem(session_, propertyName,  sVal) != ISEG_OK ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "%s:%s: Error while writing value '%d' for %s : %s",
      session_, functionName, value, propertyName, strerror( errno ) );

    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;

    return asynError;
  }

  pasynUser->alarmStatus = 0;
  pasynUser->alarmSeverity = 0;

  getTimeStamp(&timeStamp);
  pasynUser->timestamp = timeStamp;
  // update value of parameter
  status = (asynStatus) setUIntDigitalParam(function, value, mask );
  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks();

  if( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                   "%s:%s: status=%d, function=%d, value=%d",
                   session_, functionName, status, function, value );
  else
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE,
               "%s:%s: function=%d, value=%d\n",
              session_, functionName, function, value );
  return status;
}


asynStatus drvAsynIseghalService::readInt32(asynUser *pasynUser, epicsInt32 *value) {

  static const char *functionName = "readInt32";

  epicsInt16 function = pasynUser->reason;
  printf("\033[0;33m%s : (%s) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, function);

  asynStatus status = asynSuccess;
  IsegItem item = EmptyIsegItem;
  epicsInt32 iVal = 0;

	status = getIsegHalItem (pasynUser, &item);
	if(status != asynSuccess) return asynError;

  // set new paramter value and update record val field
  iVal = (epicsInt32)atoi(item.value);
  *value = iVal;
  setIntegerParam(function, iVal);
  status = (asynStatus) getIntegerParam(function, &iVal);

  if( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                   "%s:%s: status=%d, function=%d, value=%d",
                   session_, functionName, status, function, *value );
  else
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE,
               "%s:%s: function=%d, value=%d\n",
              session_, functionName, function, *value );

  // Update alarms status.
  pasynUser->alarmStatus = 0;
  pasynUser->alarmSeverity = 0;

  return status;
}
asynStatus drvAsynIseghalService::writeInt32(asynUser *pasynUser, epicsInt32 value) {

  static const char *functionName = "writeInt32";

  int function = pasynUser->reason;
  const char *propertyName;
  epicsTimeStamp timeStamp; getTimeStamp(&timeStamp);
  asynStatus status = asynSuccess;
  char sVal[20];
  printf("\033[0;33m%s : (%s) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, function);

  if (iseghalExiting_) return asynSuccess;

  // Test if interface is connected to isegHAL server
  if( !devConnected( session_ ) ) {
    if(devInitOk_ && !conMan_) {
      this->disconnect(pasynUser);
    }
    return asynError;
  }

  getParamName(function, &propertyName);

  if (propertyName == NULL) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      session_, functionName, propertyName);
      return asynError;
  }

  IsegItemProperty itemProperty = iseg_getItemProperty( session_, propertyName );

  if(strcmp( itemProperty.access, "RW" ) != 0) {

    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      session_, functionName, propertyName, itemProperty.access );
    // Update alarms status.
    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;

    return asynError;
  }

  epicsSnprintf( sVal, WRITE_LEN, "%d", value );

  if(iseg_setItem(session_, propertyName,  sVal) != ISEG_OK ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "%s:%s: Error while writing value '%d' for %s : %s",
      session_, functionName, value, propertyName, strerror( errno ) );

    // Update alarms status.
    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;

    return asynError;
  }

  pasynUser->alarmStatus = 0;
  pasynUser->alarmSeverity = 0;

  getTimeStamp(&timeStamp);
  pasynUser->timestamp = timeStamp;
  // update value of parameter
  status = (asynStatus) setIntegerParam(function, value);

  status = (asynStatus) callParamCallbacks();

  if( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                   "%s:%s: status=%d, function=%d, value=%d",
                   session_, functionName, status, function, value );
  else
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE,
               "%s:%s: function=%d, value=%d\n",
              session_, functionName, function, value );
  return status;
}
asynStatus drvAsynIseghalService::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason)
{

  static const char *functionName = "readOctet";

  epicsInt16 function = pasynUser->reason;
  printf("\033[0;33m%s : (%s) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, function);

  char octetValue[maxChars];
  IsegItem item = EmptyIsegItem;
  asynStatus status = asynSuccess;

	status = getIsegHalItem (pasynUser, &item);
	if(status != asynSuccess) return asynError;

  strncpy(value, item.value, maxChars);
  *nActual = strlen(item.value);
  setStringParam( function, item.value);
  status = (asynStatus) getStringParam(function, maxChars, octetValue);

  if( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                   "%s:%s: status=%d, function=%d, value=%s",
                   session_, functionName, status, function, value );
  else
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE,
               "%s:%s: function=%d, value=%s\n",
              session_, functionName, function, value );

  pasynUser->alarmStatus = 0;
  pasynUser->alarmSeverity = 0;

  return status;


}
asynStatus drvAsynIseghalService::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual)
{
  static const char *functionName = "writeOctet";

  int function = pasynUser->reason;
  const char *propertyName;
  asynStatus status = asynSuccess;
  epicsTimeStamp timeStamp;

  printf("\033[0;33m%s : (%s) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, function);
  // Test if interface is connected to isegHAL server
  if( !devConnected( session_ ) ) {
    if(devInitOk_ && !conMan_) {
      this->disconnect(pasynUser);
    }
    return asynError;
  }

  getParamName(function, &propertyName);

  if (propertyName == NULL) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      session_, functionName, propertyName);
      return asynError;
  }

  IsegItemProperty itemProperty = iseg_getItemProperty( session_, propertyName );

  if(strcmp( itemProperty.access, "RW" ) != 0) {

    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      session_, functionName, propertyName, itemProperty.access );

    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;

    return asynError;
  }


  if(iseg_setItem(session_, propertyName,  value) != ISEG_OK ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "%s:%s: Error while writing value '%s' for %s : %s",
      session_, functionName, value, propertyName, strerror( errno ) );

    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;

    return asynError;
  }
  getTimeStamp(&timeStamp);
  pasynUser->timestamp = timeStamp;

  pasynUser->alarmStatus = 0;
  pasynUser->alarmSeverity = 0;
  // update value of parameter
  status = (asynStatus) setStringParam(function, value);

  status = (asynStatus) callParamCallbacks();

  if( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
                   "%s:%s: status=%d, function=%d, value=%s",
                   session_, functionName, status, function, value );
  else
    asynPrint( pasynUser, ASYN_TRACEIO_DEVICE,
               "%s:%s: function=%d, value=%s\n",
              session_, functionName, function, value );

  return status;
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

    // Check we have this iseghal item
    if (!this->hasIsegHalItem(halItem)) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "\033[0;33m%s:%s: Parameter '%s' doesn't exist on iseghal item list\n\033[0m",
                driverName, functionName, halItem);
          // Update alarms status.
      pasynUser->alarmStatus = 17;    // UDF
      pasynUser->alarmSeverity = 3;   // INVALID
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
        pasynUser->alarmStatus = 17;    // UDF
        pasynUser->alarmSeverity = 3;   // INVALID
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

    int index;
    if (findParam(halItemFQN, &index) == asynParamNotFound) {
      //Create parameter of the correct type

      if( strcmp(halItemType, "INT") == 0) {
          createParam(halItemFQN, asynParamInt32, &itemReason_);
          setIntegerParam (itemReason_,       0);

      } else if (strcmp(halItemType, "DBL") == 0) {
          createParam(halItemFQN, asynParamFloat64, &itemReason_);
          setDoubleParam (itemReason_,       0.0);

      } else if (strcmp(halItemType, "STR") == 0) {
          createParam(halItemFQN, asynParamOctet, &itemReason_);
          setStringParam(itemReason_,         "0.0");

      } else if (strcmp(halItemType, "DIG") == 0) {
          createParam(halItemFQN, asynParamUInt32Digital, &itemReason_);
					setUIntDigitalParam(itemReason_, 0, 0xf );
      } else {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: Expected Type is INT|DBL|STR|DIG. Got '%s'\n", driverName, functionName, halItemType);
          return asynError;
      }

      pasynUser->reason = itemReason_;
      isegHalItemsLookup.insert( std::make_pair( itemReason_, std::string(tmp) ) );

      printf("\033[0;33m%s : (%s) : FQN : '%s' : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, halItemFQN, itemReason_);
      itemReason_++;
    } else {
      pasynUser->reason = index;
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

  printf("\033[0;33m%s : (%s)\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__);
  if( !devConnect(session_, interface_) ) {
    epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "%s: Can't open %s: %s", session_, interface_, strerror( errno ) );
    return asynError;
  }

/* check if devices is responsive.
  * iCSModel: iseg iCS based HV system controller model
  * cc24: for crate systems
	* icsmini: system exposes the first crate controller module model i.e MICC.
*/

  char iCSModel[FQN_LEN];
  // icsmini default
  strcpy(iCSModel, "0.0.Article");

  if( strcmp( deviceModel_, "cc24" ) == 0 ) {
    strcpy(iCSModel, "0.100.Article");
  }


  IsegItem model = iseg_getItem( session_, iCSModel );
  if( strcmp( model.quality, ISEG_ITEM_QUALITY_OK ) != 0 ) {

    epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "\033[31;1m%s: Error while reading from device (Q: %s): %s\033[0m", session_, model.quality, strerror( errno ) );
    return asynError;
  }
  else {
        printf("\033[0;33miCS CONTROLLER MODEL '%s'\n\033[0m", model.value);
  }
  devInitOk_ = 1;
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
 std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread:  "<< iseghalExiting_<<"\033[0m" << std::endl;

  asynPrint( pasynUser, ASYN_TRACEIO_DRIVER,
             "%s: disconnect %s\n", session_, interface_ );
  // we only disconnect from the device if exiting or if multiple reconnexion
  // attempts to the current device session failed.
  if(iseghalExiting_) {
    if( !devDisconnect( session_ )) {
      epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                     "%s: cannot diconnect from %s ", session_, interface_ );
      return asynError;
    }
  }
  // if this call is not a shutdown exit, thus we lost connection to device
  // iseghal will internally issues a reconnexion to the server. we need to listen to that
  // inside our connect method called by autoconnect. for that we make the call below:

  pasynManager->exceptionDisconnect(pasynUser);
  conMan_ = true;
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
  static const iocshArg initIseghalServiceArg0 = { "portName",    iocshArgString };
  static const iocshArg initIseghalServiceArg1 = { "interface",   iocshArgString };
  static const iocshArg initIseghalServiceArg2 = { "icsCtrtype",  iocshArgString };
  static const iocshArg initIseghalServiceArg3 = { "autoConnect", iocshArgInt };
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
