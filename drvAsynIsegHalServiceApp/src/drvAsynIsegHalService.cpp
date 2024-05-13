/* drvAsynIsegHalService.cpp
 *
 * EPICS Asyn Driver for ISEG HV/LV PS systems based the unified iseg control platform 'iCS'
 * capable systems i.e iCSmini 2, CC24 and SHR(not tested yet) controllers
 * The drivers uses iseg Hardware Abstraction Layer (isegHAL) Service based on a Qt based SSL Socket driver
 * to control and monitor iseg high voltage through different communication interfaces.
 *
 * Author: Yann Stephen Mandza
 * Date: February 14, 2024
*/

// ANSI C/C++ includes
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
#include "drvAsynIsegHalService.h"

static const char *driverName = "drvAsynIsegHalService";

#define WRITE_BUF_LEN   8
#define ITEM_TYPE_LEN   4
#define ITEM_FQN_LEN    34
#define ITEM_ADDR_LEN   6

#define DEFAULT_PORT_RECONNECT 15 // 5 min.
typedef  std::map<epicsUInt32, std::string>::const_iterator  itemIter;

// Handle for Poller Thread
static drvIsegHalPollerThread *drvIsegHalPollerThread_ = NULL;

// No access of iseghal instance outside of port thread at init
bool initStatus = 0;

// Valid isegHal items lookup table.
std::vector<std::string> isValidIsegHalItems
{
   "Status",
   "EventStatus",
   "EventMask",
   "Control",
   "CrateNumber",
   "CrateList",
   "ModuleNumber",
   "ModuleList",
   "CycleCounter",
   "Read",
   "LogLevel",
   "LogPath",
   "LiveInsertion",
   "SaveConfiguration",
   "EthName",
   "EthAddress",
   "ServerVersion",
   "NetworkTimeout",
   "SessionName",
   // Can Line items
   "BitRate",
   // Crate/module items
   "Connected",
   "Alive",
   "PowerOn",
   "FanSpeed",
   "SerialNumber",
   "DeviceClass",
   "FirmwareRelease",
   "FirmwareName",
   "Temperatures" ,
   "Supplies",
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
   "ChannelNumber",
   "Temperature" ,
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

   // Option Voltage controlled by temperature ( VCT )
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


/** Called by epicsAtExit to shutdown iseghal session
  * This ensure that disconnection cleanly happens inside the port thread
*/
static void iseghalSessionShutdown( void* pDrv ) {
  asynStatus status;
  drvAsynIsegHalService *pPvt = ( drvAsynIsegHalService * ) pDrv;
  pPvt->iseghalExiting_ = true;
  status =  pasynCommonSyncIO->disconnectDevice( pPvt->exitUser_ );
  if( status!=asynSuccess )
        asynPrint( pPvt->exitUser_, ASYN_TRACE_ERROR, "%s: drvAsynIsegHalService cleanup  error %s\n", pPvt->portName, pPvt->exitUser_->errorMessage );
}

/*
 * Start the polling thread when IOC is in running state
 * we want all interrupt lists filled and ready before the polling thread kicks in.
*/
static void startPolling( initHookState state ) {
  if ( state == initHookAfterIocRunning ) {
    initStatus = 1;
    // all record init done: interrupt list filled
    if( drvIsegHalPollerThread_ != NULL ) {
      drvIsegHalPollerThread_->thread.start( );
    }
  }
}

drvAsynIsegHalService::drvAsynIsegHalService( const char *portName, const char *interface, const char *icsCtrtype, epicsInt16 reconAttempt )
  : asynPortDriver( portName,
    1,                                // maxAddr
    NITEMS,
                                      // Interface mask
    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynOctetMask | asynDrvUserMask,
    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynOctetMask,
    ASYN_CANBLOCK , // asynFlags.
    1,                      // Autoconnect
    0,                                // Default priority
    0 ),                              // Default stack size
    iseghalExiting_( false ),
    conMan_( false ),
    reconAttempt_( reconAttempt > 0 ? reconAttempt : DEFAULT_PORT_RECONNECT )

{
    static const char *functionName = "drvAsynIsegHalService";

    session_      = epicsStrDup( portName );
    interface_    = epicsStrDup( interface );
    deviceModel_  = epicsStrDup( icsCtrtype );
    itemReason_   = 0;
    devInitOk_    = 0;
    exitUser_     = pasynManager->createAsynUser( 0, 0 );

    asynStatus status;
printf( "\033[0;33m%s : ( %s ):'%d'\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__, reconAttempt_ );
    /* Connect this user to the port so we can use it to to connect to the device and cleanup connexion at exit*/
    status = pasynCommonSyncIO->connect( session_, 0, &exitUser_, NULL );
    if ( status ) {
      printf( "%s:%s: pasynCommonSyncIO->connect failure, status=%d\n", driverName, functionName, status );
      return;
    }
		status =  pasynCommonSyncIO->connectDevice( exitUser_ );
		if ( status ) {
      printf( "%s:%s: pasynCommonSyncIO->connectDevice failure, status=%d\n", driverName, functionName, status );
      return;
    }
    /* iseg HAL starts collecting data from hardware after connect.
      * allow 5 secs timeout to let all values 'initialize'
    */
    pasynManager->setAutoConnectTimeout( 5.0 );

    /* Register the shutdown function for epicsAtExit */
    epicsAtExit( iseghalSessionShutdown, ( void* )this );

    /* Register the start polling fucntion: we wait till ioc is in running state before starting */
    initHookRegister( startPolling );

    // instantiate the polling thread, but dont start yet.
    drvIsegHalPollerThread_ = new drvIsegHalPollerThread( this );
    if( !drvIsegHalPollerThread_ ) return;
}

/*
*  @brief   Provide to all interfaces Read methods a uniform read access device data.
*
*  @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
*  @param   [in]  item      Address of the isegHal item to be read
*
*  @return  in case of no error occured asynSuccess is returned. Otherwise
*           asynError or asynTimeout is returned. A error message is stored
*           in pasynUser->errorMessage.
*/

asynStatus drvAsynIsegHalService::getIsegHalItem ( asynUser *isegHalUser, IsegItem *item )
{
  static const char *functionName = "getIsegHalItem";
  const char *propertyName;

  asynStatus status = asynSuccess;
  epicsInt16 function = isegHalUser->reason;

  //printf( "\033[0;33m%s : ( %s )\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__ );

  if( !initStatus ) return asynError;

  if ( iseghalExiting_ ) return asynSuccess;

  getParamName( function, &propertyName );

  if ( propertyName == NULL ) {
    epicsSnprintf( isegHalUser->errorMessage, isegHalUser->errorMessageSize,
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      session_, functionName, propertyName );
    return asynError;
  }

  // Test if interface is connected to isegHAL server
  if( !devConnected( session_ ) ) {
    this->disconnect( isegHalUser );
		// slow the polling thread, sync with autoconnect.
    drvIsegHalPollerThread_->disable();
    isegHalUser->alarmStatus    = 1;
    isegHalUser->alarmSeverity  = 3;
    return asynError;
  }

  IsegItemProperty iHalItem = iseg_getItemProperty( session_, propertyName );

  if( !( epicsStrCaseCmp( iHalItem.access, "R" ) == 0 || epicsStrCaseCmp( iHalItem.access, "RW" ) == 0 ) ) {

    epicsSnprintf( isegHalUser->errorMessage, isegHalUser->errorMessageSize,
      "\033[31;1m%s:%s Wrong item '%s' permission: Access right: '%s'\033[0m\n",
      session_, functionName, propertyName, iHalItem.access );

    isegHalUser->alarmStatus = 1;
    isegHalUser->alarmSeverity = 3;
    return asynError;
  }

  *item = iseg_getItem( session_, propertyName );

  if( strcmp( item->quality, ISEG_ITEM_QUALITY_OK ) != 0 ) {
    epicsSnprintf( isegHalUser->errorMessage,isegHalUser->errorMessageSize,
    "\033[31;1m%s:%s Read Error ( Q: %s ): %s\033[0m", session_, functionName, item->quality, strerror( errno ) );

    isegHalUser->alarmStatus = 1;
    isegHalUser->alarmSeverity = 3;
    return asynError;
  }

  epicsUInt32 seconds = 0;
  epicsUInt32 microsecs = 0;
  epicsTimeStamp time;

  if( sscanf( item->timeStampLastChanged, "%u.%u", &seconds, &microsecs ) != 2 ) {
    epicsSnprintf( isegHalUser->errorMessage, isegHalUser->errorMessageSize,
      "%s:%s: Read Error from %s : %s",
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

/*
  * @brief   Called when asyn clients call pasynFloat64->write( ).
  *
  * @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
  * @param   [in]  value      Value to write
  *
  *  @return  in case of no error occured asynSuccess is returned. Otherwise
  *           asynError or asynTimeout is returned. A error message is stored
  *           in pasynUser->errorMessage.
*/
asynStatus drvAsynIsegHalService::writeFloat64( asynUser *pasynUser, epicsFloat64 value )
{

  static const char *functionName = "writeFloat64";

  const char *propertyName;
  epicsTimeStamp timeStamp;
  char sVal[WRITE_BUF_LEN];
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;

  printf( "\033[0;33m%s : ( %s ) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__, function );
  if ( iseghalExiting_ ) return asynSuccess;
  // Test if interface is connected to isegHAL server
  if( !devConnected( session_ ) ) {
    disconnect( pasynUser );
    drvIsegHalPollerThread_->disable();
    pasynUser->alarmStatus    = 1;
    pasynUser->alarmSeverity  = 3;
    return asynError;
  }

  getParamName( function, &propertyName );

  if ( propertyName == NULL ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      session_, functionName, propertyName );
      return asynError;
  }

  IsegItemProperty iHalItem = iseg_getItemProperty( session_, propertyName );

  if( epicsStrCaseCmp( iHalItem.access, "RW" ) != 0 ) {

    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      session_, functionName, propertyName, iHalItem.access );

    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;
    return asynError;
  }

  epicsSnprintf( sVal, WRITE_BUF_LEN, "%f", value );

  if( iseg_setItem( session_, propertyName,  sVal ) != ISEG_OK ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "%s:%s: Error writing value '%f' for %s : %s",
      session_, functionName, value, propertyName, strerror( errno ) );

    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;
    return asynError;
  }

  pasynUser->alarmStatus = 0;
  pasynUser->alarmSeverity = 0;

  getTimeStamp( &timeStamp );
  pasynUser->timestamp = timeStamp;

  // update value of parameter
  status = ( asynStatus ) setDoubleParam( function, value );
  status = ( asynStatus ) callParamCallbacks( );
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
*  @brief   Called when asyn clients call pasynFloat64->read( ).
*
*  @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
*  @param   [in]  value      Address of the value to read
*
*  @return  in case of no error occured asynSuccess is returned. Otherwise
*           asynError or asynTimeout is returned. A error message is stored
*           in pasynUser->errorMessage.
*/
asynStatus drvAsynIsegHalService::readFloat64( asynUser *pasynUser, epicsFloat64 *value )
{
  static const char *functionName = "readFloat64";

  epicsInt16 function = pasynUser->reason;
  IsegItem item = EmptyIsegItem;
  asynStatus status = asynSuccess;
  epicsFloat64 dVal = 0;

  printf( "\033[0;33m%s : ( %s ) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__, function );
  status = getIsegHalItem ( pasynUser, &item );
  if( status != asynSuccess ) return asynError;

  // set new paramter value and update record val field

  dVal = ( epicsFloat64 )strtod ( item.value, NULL );
  *value = dVal;
  setDoubleParam ( function, dVal );
  status = ( asynStatus ) getDoubleParam( function, &dVal );

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

/*
  * @brief   Called when asyn clients call pasynUInt32Digital->read( ).
  *
  * @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
  * @param   [in]  value      Address of the value to read
  * @param   [in]  mask       Mask value to use when reading the value.
  *
  *  @return  in case of no error occured asynSuccess is returned. Otherwise
  *           asynError or asynTimeout is returned. A error message is stored
  *           in pasynUser->errorMessage.
*/
asynStatus drvAsynIsegHalService::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask )
{

  static const char *functionName = "readUInt32Digital";

  asynStatus status = asynSuccess;
  IsegItem item = EmptyIsegItem;
  epicsUInt32 iVal = 0;
  epicsInt16 function = pasynUser->reason;

  printf( "\033[0;33m%s : ( %s ) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__, function );

  status = getIsegHalItem ( pasynUser, &item );
  if( status != asynSuccess ) return asynError;

  iVal = ( epicsUInt32 )atoi( item.value );
  *value = iVal;
  status = ( asynStatus ) setUIntDigitalParam( function, iVal, mask );
  pasynUser->alarmStatus = 0;
  pasynUser->alarmSeverity = 0;
  status = ( asynStatus ) getUIntDigitalParam( function, &iVal, mask );

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


/*
  * @brief   Called when asyn clients call pasynUInt32Digital->write( ).
  *
  * @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
  * @param   [in]  value      Value to write
  * @param   [in]  mask       Mask value to use when reading the value.
  *
  *  @return  in case of no error occured asynSuccess is returned. Otherwise
  *           asynError or asynTimeout is returned. A error message is stored
  *           in pasynUser->errorMessage.
*/
asynStatus drvAsynIsegHalService::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask )
{
  static const char *functionName = "writeUInt32Digital";

  epicsInt16 function = pasynUser->reason;
  const char *propertyName;
  epicsTimeStamp timeStamp;
  asynStatus status = asynSuccess;

  printf( "\033[0;33m%s : ( %s ) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__, function );

  if ( iseghalExiting_ ) return asynSuccess;

  // Test if interface is connected to isegHAL server
  if( !devConnected( session_ ) ) {
    disconnect( pasynUser );
    drvIsegHalPollerThread_->disable();
    pasynUser->alarmStatus    = 1;
    pasynUser->alarmSeverity  = 3;
    return asynError;
  }

  itemIter it = isegHalItemsLookup.find( function );

  if ( it != isegHalItemsLookup.end( ) ) {
    // Skip writing to Status
    if( it->second == "Status" ) {
      return asynSuccess;
    }
  }

  getParamName( function, &propertyName );

  if ( propertyName == NULL ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      session_, functionName, propertyName );
      return asynError;
  }

  IsegItemProperty iHalItem = iseg_getItemProperty( session_, propertyName );

  if( epicsStrCaseCmp( iHalItem.access, "RW" ) != 0 ) {

    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      session_, functionName, propertyName, iHalItem.access );

    pasynUser->alarmStatus    = 2;
    pasynUser->alarmSeverity  = 3;

    return asynError;
  }

  char sVal[WRITE_BUF_LEN];
  epicsSnprintf( sVal, WRITE_BUF_LEN, "%d", ( value & mask ) );

  if( iseg_setItem( session_, propertyName,  sVal ) != ISEG_OK ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "%s:%s: Write Error - value '%d' for %s : %s",
      session_, functionName, value, propertyName, strerror( errno ) );

    pasynUser->alarmStatus    = 2;
    pasynUser->alarmSeverity  = 3;
    return asynError;
  }

  pasynUser->alarmStatus    = 0;
  pasynUser->alarmSeverity  = 0;

  getTimeStamp( &timeStamp );
  pasynUser->timestamp = timeStamp;
  // update value of parameter
  status = ( asynStatus ) setUIntDigitalParam( function, value, mask );
  /* Do callbacks so higher layers see any changes */
  status = ( asynStatus ) callParamCallbacks( );

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

/*
  * @brief   Called when asyn clients call pasynInt32->read( ).
  *
  * @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
  * @param   [in]  value      Address of the value to read
  *
  *  @return  in case of no error occured asynSuccess is returned. Otherwise
  *           asynError or asynTimeout is returned. A error message is stored
  *           in pasynUser->errorMessage.
*/
asynStatus drvAsynIsegHalService::readInt32( asynUser *pasynUser, epicsInt32 *value ) {

  static const char *functionName = "readInt32";

  asynStatus status = asynSuccess;
  IsegItem item = EmptyIsegItem;
  epicsInt32 iVal = 0;
  epicsInt16 function = pasynUser->reason;
  printf( "\033[0;33m%s : ( %s ) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__, function );

  status = getIsegHalItem ( pasynUser, &item );
  if( status != asynSuccess ) return asynError;

  // set new paramter value and update record val field
  iVal = ( epicsInt32 )atoi( item.value );
  *value = iVal;
  setIntegerParam( function, iVal );
  status = ( asynStatus ) getIntegerParam( function, &iVal );

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
/*
  * @brief   Called when asyn clients call pasynInt32->write( ).
  *
  * @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
  * @param   [in]  value      Value to write
  *
  *  @return  in case of no error occured asynSuccess is returned. Otherwise
  *           asynError or asynTimeout is returned. A error message is stored
  *           in pasynUser->errorMessage.
*/
asynStatus drvAsynIsegHalService::writeInt32( asynUser *pasynUser, epicsInt32 value ) {

  static const char *functionName = "writeInt32";

  int function = pasynUser->reason;
  const char *propertyName;
  epicsTimeStamp timeStamp; getTimeStamp( &timeStamp );
  asynStatus status = asynSuccess;
  char sVal[20];
  printf( "\033[0;33m%s : ( %s ) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__, function );

  if ( iseghalExiting_ ) return asynSuccess;

  // Test if interface is connected to isegHAL server
  if( !devConnected( session_ ) ) {
    disconnect( pasynUser );
    drvIsegHalPollerThread_->disable();
    pasynUser->alarmStatus    = 1;
    pasynUser->alarmSeverity  = 3;
    return asynError;
  }

  getParamName( function, &propertyName );

  if ( propertyName == NULL ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      session_, functionName, propertyName );
      return asynError;
  }

  IsegItemProperty iHalItem = iseg_getItemProperty( session_, propertyName );

  if( epicsStrCaseCmp( iHalItem.access, "RW" ) != 0 ) {

    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      session_, functionName, propertyName, iHalItem.access );
    // Update alarms status.
    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;

    return asynError;
  }

  epicsSnprintf( sVal, WRITE_BUF_LEN, "%d", value );

  if( iseg_setItem( session_, propertyName,  sVal ) != ISEG_OK ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "%s:%s: Write Error - value '%d' for %s : %s",
      session_, functionName, value, propertyName, strerror( errno ) );

    // Update alarms status.
    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;

    return asynError;
  }

  pasynUser->alarmStatus = 0;
  pasynUser->alarmSeverity = 0;

  getTimeStamp( &timeStamp );
  pasynUser->timestamp = timeStamp;
  // update value of parameter
  status = ( asynStatus ) setIntegerParam( function, value );

  status = ( asynStatus ) callParamCallbacks( );

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
/*
  * @brief   Called when asyn clients call pasynOctect->read( ).
  *
  * @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
  * @param   [in]  value      addres of Value to read
  * @param   [in]  maxChars   max number of char to read
  * @param   [in]  actual     addres for actual number of char read
  * @param   [in]  eomReason  address for eom
  *
  *  @return  in case of no error occured asynSuccess is returned. Otherwise
  *           asynError or asynTimeout is returned. A error message is stored
  *           in pasynUser->errorMessage.
*/
asynStatus drvAsynIsegHalService::readOctet( asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason )
{

  static const char *functionName = "readOctet";

  char octetValue[maxChars];
  IsegItem item = EmptyIsegItem;
  asynStatus status = asynSuccess;
  epicsInt16 function = pasynUser->reason;
  printf( "\033[0;33m%s : ( %s ) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__, function );

  status = getIsegHalItem ( pasynUser, &item );
  if( status != asynSuccess ) return asynError;

  strncpy( value, item.value, maxChars );
  *nActual = strlen( item.value );
  setStringParam( function, item.value );
  status = ( asynStatus ) getStringParam( function, maxChars, octetValue );

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

/*
  * @brief   Called when asyn clients call pasynOctect->write( ).
  *
  * @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
  * @param   [in]  value      addres of value to write
  * @param   [in]  maxChars   max number of char to write
  * @param   [in]  actual     addres for actual number of char written
  *
  *  @return  in case of no error occured asynSuccess is returned. Otherwise
  *           asynError or asynTimeout is returned. A error message is stored
  *           in pasynUser->errorMessage.
*/
asynStatus drvAsynIsegHalService::writeOctet( asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual )
{
  static const char *functionName = "writeOctet";

  int function = pasynUser->reason;
  const char *propertyName;
  asynStatus status = asynSuccess;
  epicsTimeStamp timeStamp;

  printf( "\033[0;33m%s : ( %s ) : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__, function );
  // Test if interface is connected to isegHAL server
  if( !devConnected( session_ ) ) {
    disconnect( pasynUser );
    drvIsegHalPollerThread_->disable();
    pasynUser->alarmStatus    = 1;
    pasynUser->alarmSeverity  = 3;
    return asynError;
  }

  getParamName( function, &propertyName );

  if ( propertyName == NULL ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s:Invalid iseghal parameter '%s'\033[0m\n",
      session_, functionName, propertyName );
      return asynError;
  }

  IsegItemProperty iHalItem = iseg_getItemProperty( session_, propertyName );

  if( epicsStrCaseCmp( iHalItem.access, "RW" ) != 0 ) {

    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "\033[31;1m%s:%s Wrong permission on iseghal item %s access right: '%s'\033[0m\n",
      session_, functionName, propertyName, iHalItem.access );

    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;
    return asynError;
  }

  if( iseg_setItem( session_, propertyName,  value ) != ISEG_OK ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize,
      "%s:%s: Write Error - value '%s' for %s : %s",
      session_, functionName, value, propertyName, strerror( errno ) );

    pasynUser->alarmStatus = 2;
    pasynUser->alarmSeverity = 3;
    return asynError;
  }
  getTimeStamp( &timeStamp );
  pasynUser->timestamp = timeStamp;

  pasynUser->alarmStatus    = 0;
  pasynUser->alarmSeverity  = 0;
  // update value of parameter
  status = ( asynStatus ) setStringParam( function, value );
  status = ( asynStatus ) callParamCallbacks( );

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

/*
  * Userparam parsing method: skip separator
*/
static const char *skipSeparator( const char *pstart, int underscoreOk ){
    const char *p = pstart;
    while( *p && ( isspace( ( int )*p ) || ( underscoreOk && ( *p=='_' ) ) ) ) p++;
    return p;
}

/*
  * Userparam parsing method: Validate 'item' part of userparm string
*/
int drvAsynIsegHalService::isValidIsegHalItem( const char *item ) {
  std::vector< std::string >::iterator it;
  it = std::find( isValidIsegHalItems.begin( ), isValidIsegHalItems.end( ), std::string( item ) );
  if( it != isValidIsegHalItems.end( ) ) return true;
  return false;
}
/*
  * @Brief  User param follows this format: 'DTYPE'_'item'($( ADDR ))
  *         where Type is INT, DBL, STR and DIG for UINT32DIGITAL
  *         the fully qualified name ( FQN ) for gettting/setting an isegHal item value
  *         folow this format: ADDR.item, this is used to create port driver parameters.
  *
  * @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
  * @param   [in]  drvInfo    User param string
  *
  *  @return  in case of no error occured asynSuccess is returned. Otherwise
  *           asynError or asynTimeout is returned. A error message is stored
  *           in pasynUser->errorMessage.
*/
asynStatus drvAsynIsegHalService::drvUserCreate( asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize ) {

  static const char *functionName = "drvUserCreate";

  unsigned int len = 0;
  unsigned int prevLen = 0;

  const char *p;
  const char *pnext;

  char uParamType[ITEM_TYPE_LEN];
  char itemSubAddr[ITEM_TYPE_LEN];
  char iHalItem[ITEM_FQN_LEN];
  char iHalItemFQN[ITEM_FQN_LEN];
  char iHalItemAddr[ITEM_ADDR_LEN];

  static bool firstPass = true;

  if ( strlen( drvInfo ) > 4 /* && firstPass */ ) {

    pnext = skipSeparator( drvInfo,0 );
    p = pnext;
    for( len=0; *pnext && isalpha( *pnext ); len++, pnext++ ){}

    if( *pnext==0 ) {
        epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
            "invalid userparam Must be TYPE'_'/' ''I'tem(ADDR)" );
        return asynError;
    }
    strncpy( uParamType, p, len );
    uParamType[len] = 0;

     //next is item
    p = skipSeparator( pnext,1 );
    pnext = p;

   if( *p==0 || !isupper( *p ) ) {
        epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
            "invalid userparam Must be TYPE _/ 'I'tem(ADDR)" );
        return asynError;
    }

    for( len=0; *p && isalpha( *p ); len++, p++ ){}

    strncpy( iHalItem, pnext, len );
    prevLen = len;
    *( iHalItem+len ) = 0;

    if ( !this->isValidIsegHalItem( iHalItem ) ) {
      asynPrint( pasynUser, ASYN_TRACE_ERROR,
                "\033[0;33m%s:%s: Parameter '%s' doesn't exist on iseghal item list\n\033[0m",
                driverName, functionName, iHalItem );
      // Update alarms status.
      pasynUser->alarmStatus = 17;    // UDF
      pasynUser->alarmSeverity = 3;   // INVALID
      return asynError;
    }

    if( *p==':' ) {

      if( ( strcmp( iHalItem, "Control" ) == 0 ) ) {
        pnext = p;
        p++; // skip this ':'
        if( !isdigit( *p ) ){
            epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                "invalid userparam: Sub address must be - %s:digit's'", iHalItem );
            return asynError;
        }

        for( len=0; *p && isdigit( *p ); len++, p++ ){}
        strncpy( itemSubAddr, pnext, len+1 );
        *( itemSubAddr+len+1 ) = 0;
        strcat ( iHalItem, itemSubAddr );
        prevLen+=len+1;
      }
      else {
        epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                  "invalid userparam property: Must be 'Control:ADDR'" );
        pasynUser->alarmStatus = 17;
        pasynUser->alarmSeverity = 3;
        return asynError;
      }
    }

    if ( *p=='[' ) {
      pnext = p;
      p++; // skip this '['
      if( !isdigit( *p ) ){
          epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
              "invalid userparam: Sub address must be - %s[digit's']", iHalItem );
          return asynError;
      }

      for( len=0; *p && isdigit( *p ) && ( *p!=']' ); len++, p++ ){}

      strncpy( itemSubAddr, pnext, len+2 );
      *( itemSubAddr+len+2 ) = 0;
      prevLen+=len+2;
      strcat ( iHalItem, itemSubAddr );
      p++;
    }

    if ( *p =='(' ) {
      p++;
      pnext = skipSeparator( p,0 );
      // Empty bracket or non decimal first char not valid
      if( !isdigit( *pnext ) ){
          epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
              "invalid userparam: item address format is: - %s( digit's' )", iHalItem );
          return asynError;
      }

      /* To do: add aadress format validator
       *        correct formats: digit / digit.digit / digit.digit.digit
      */
      for( len=0; *p && ( *p!=' ' ) && ( *p!=')' ); len++, p++ ){}

      if( len > ITEM_ADDR_LEN ) {
          epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
              "invalid userparam: Wrong format for item '%s' address", iHalItem );
          return asynError;
      }

      prevLen+=len+1;
      strncpy( iHalItemAddr, pnext, len );
      *( iHalItemAddr+len ) = 0;
      epicsSnprintf( iHalItemFQN, prevLen+len, "%s.%s", iHalItemAddr, iHalItem );
    } else {
      //not a system item?
      if ( ( *p == 0 ) && ( strcmp( iHalItem, "Status" ) == 0
          || strcmp( iHalItem, "CrateNumber" ) == 0
          || strcmp( iHalItem, "BitRate" ) == 0
          || strcmp( iHalItem, "ModuleNumber" ) == 0
          || strcmp( iHalItem, "CycleCounter" ) == 0
          || strcmp( iHalItem, "Configuration" ) == 0
          || strcmp( iHalItem, "Read" ) == 0
          || strcmp( iHalItem, "Write" ) == 0
          || strcmp( iHalItem, "LogLevel" ) == 0
          || strcmp( iHalItem, "LogPath" ) == 0
          || strcmp( iHalItem, "LiveInsertion" ) == 0
          || strcmp( iHalItem, "SaveConfiguration" ) == 0
          || strcmp( iHalItem, "ServerVersion" ) == 0
          || strcmp( iHalItem, "NetworkTimeout" ) == 0
          || strcmp( iHalItem, "SessionName" ) == 0
          || strcmp( iHalItem, "ModuleList" ) == 0 ) )
      {
        // Create fully qualified object for system items
        strcpy( iHalItemFQN, iHalItem );
      } else {
        epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
            "invalid userparam Must be TYPE _/ 'I'tem(ADDR)" );
        pasynUser->alarmStatus    = 17;   // UDF
        pasynUser->alarmSeverity  = 3;    // INVALID
        return asynError;
      }
    }

    int index;

    if ( findParam( iHalItemFQN, &index ) == asynParamNotFound ) {
      //Create parameter of the correct type
      if( epicsStrCaseCmp( uParamType, "INT" ) == 0 ) {
          createParam( iHalItemFQN, asynParamInt32, &itemReason_ );
          setIntegerParam ( itemReason_,       0 );

      } else if ( epicsStrCaseCmp( uParamType, "DBL" ) == 0 ) {
          createParam( iHalItemFQN, asynParamFloat64, &itemReason_ );
          setDoubleParam ( itemReason_,       0.0 );

      } else if ( epicsStrCaseCmp( uParamType, "STR" ) == 0 ) {
          createParam( iHalItemFQN, asynParamOctet, &itemReason_ );
          setStringParam( itemReason_,         "0" );

      } else if ( epicsStrCaseCmp( uParamType, "DIG" ) == 0 ) {
          createParam( iHalItemFQN, asynParamUInt32Digital, &itemReason_ );
          setUIntDigitalParam( itemReason_, 0, 0 );

      } else {
          asynPrint( this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Expected: INT|DBL|STR|DIG. Got '%s'\n",
                    driverName, functionName, uParamType );
          return asynError;
      }

      pasynUser->reason = itemReason_;
      isegHalItemsLookup.insert( std::make_pair( itemReason_, std::string( iHalItem ) ) );

      printf( "\033[0;33m%s : ( %s ) : FQN : '%s' : Reason: '%d'\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__, iHalItemFQN, itemReason_ );
      itemReason_++;
    } else {
      // we want the same reason to index the exisiting property name.
      pasynUser->reason = index;
    }
  }

  return asynSuccess;
}


/*
  * @brief       Connect driver to device
  * @param [in]  pasynUser  pasynUser structure that encodes the reason and address.
  * @return      in case of no error occured asynSuccess is returned. Otherwise
  *              asynError or asynTimeout is returned. A error message is stored
  *              in pasynUser->errorMessage.
*/
asynStatus drvAsynIsegHalService::connect( asynUser *pasynUser ) {

  printf( "\033[0;33m%s : ( %s )\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__ );
  if( !devConnect( session_, interface_ ) ) {
    epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "%s: Can't open %s: %s", session_, interface_, strerror( errno ) );
    return asynError;
  }

  /* Test that device is  responsive.
    * iCSModel: iseg iCS based HV system controller model
    * cc24: for crate systems
    * icsmini2: system exposes the controller module model i.e MICC.
  */
  char iCSModel[ITEM_FQN_LEN];

  if( strcmp( deviceModel_, "cc24" ) == 0 ) {
    strcpy( iCSModel, "0.1000.Article" );
  } else if ( strcmp( deviceModel_, "icsmini" ) == 0) {
    strcpy( iCSModel, "0.0.Article" );
  } else if ( strcmp( deviceModel_, "shr" ) == 0){
    // Not comfirmed yet
    strcpy( iCSModel, "0.0.Article" );
  } else {
    return asynError;
  }

  IsegItem model = iseg_getItem( session_, iCSModel );
  if( strcmp( model.quality, ISEG_ITEM_QUALITY_OK ) != 0 ) {

    epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "\033[31;1m%s: Error while reading from device ( Q: %s ): %s\033[0m", session_, model.quality, strerror( errno ) );
    return asynError;
  }
  else {
        printf( "\033[0;33miCS CONTROLLER MODEL '%s'\n\033[0m", model.value );
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
asynStatus drvAsynIsegHalService::disconnect( asynUser *pasynUser ) {
printf( "\033[0;33m%s : ( %s )\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__ );
  asynPrint( pasynUser, ASYN_TRACEIO_DRIVER,
             "%s: disconnect %s\n", session_, interface_ );

  // we only disconnect from the device if exiting
  if( iseghalExiting_ ) {
    if( !devDisconnect( session_ ) ) {
      epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                     "%s: cannot diconnect from %s ", session_, interface_ );
      return asynError;
    }
		return asynSuccess;
  }
  /*
    * if not a shutdown exit, we lost connection to device.
    * we use autoConnect to issue reconnexion attempts to the server.
  */
  pasynManager->exceptionDisconnect( pasynUser );
  return asynSuccess;
}

/*
  * @brief       Connect to isegHal device
  * @param [in]  name  drvAsynIsegHalService internal name of the interface handle.
  * @param [in]  interface   name of the hardware interface.
  * @return      true if interface is already connected or if successfully connected.
*/
int drvAsynIsegHalService::devConnect( std::string const& name, std::string const& interface ) {

  if ( devInitOk_ ) {
		// still in error state?
    if( iseg_isConnError( name.c_str( ) ) ) {
			/* the session was disconnected! attempt reconexion
			* otherwise close the connexion and open a new one. we use autoConnect freq here.
			*/
      asynPrint( pasynUserSelf, ASYN_TRACEIO_DRIVER,"%s: Attempting reconnexion to %s:%d\n", name.c_str( ), interface.c_str( ), reconAttempt_ );
			// if no reconnexion timeout
			if( reconAttempt_ > 0 ) {

				//if no reconnect return false
				IsegResult status = iseg_reconnect( name.c_str( ), interface.c_str( ) );
				if ( ISEG_OK != status ) {
					reconAttempt_--;
					return false;
				}
				// if reconnect enable poller.
				drvIsegHalPollerThread_->enable();

			} else {

				// if time out, no more reconnection, disconnect to cleanup old session instance, set devInitOk_ to 0, return false
				devDisconnect( name ); // need to verify that old hal instance from driver is indeed cleanup?
				devInitOk_ = 0;
				reconAttempt_ = DEFAULT_PORT_RECONNECT;
				return false;
			}

    }
  } else {

    asynPrint( pasynUserSelf, ASYN_TRACEIO_DRIVER,"%s: Opening new session to %s\n", name.c_str( ), interface.c_str( ) );

    IsegResult status = iseg_connect( name.c_str( ), interface.c_str( ), NULL );
    if ( ISEG_OK != status ) return false;

    // new iseghal session to iseg device.
    openedSessions.push_back( name );
		// enable poller.
		if( drvIsegHalPollerThread_ != NULL) drvIsegHalPollerThread_->enable();

    devInitOk_ = true;
  }

  /* iseg HAL starts collecting data from hardware after connect.
    * wait 5 secs to let all values 'initialize'
  */
  epicsThreadSleep( 5 );
  return true;
}

/*
  * @brief       Disconnect from isegHal device
  * @param [in]  name  drvAsynIsegHalService internal name of the interface handle.
  * @return      true if interface is already connected or if successfully connected.
*/
int drvAsynIsegHalService::devDisconnect( std::string const& name ) {

    int status = iseg_disconnect( name.c_str( ) );
    if ( status != ISEG_OK  ) return false;
    openedSessions.clear( );
    return true;
}

/*
  * @brief       Check if a device is connected
  * @param [in]  name  drvAsynIsegHalService internal name of the interface handle.
  * @return      true device is connected.
*/
int drvAsynIsegHalService::devConnected( std::string const& name ) {
  std::vector< std::string >::iterator it;
  it = std::find( openedSessions.begin( ), openedSessions.end( ), name );

  // cant access hal object unless connect was successfully called
  if( !devInitOk_ ) return false;
  return ( it != openedSessions.end( ) && !iseg_isConnError( name.c_str( ) ) );
}

asynStandardInterfaces drvAsynIsegHalService::getAsynStdIface( )
{
  return asynStdInterfaces;
}

char *drvAsynIsegHalService::getSessionName ( ){
  return session_;
}

// Configuration routines. Called directly, or from the iocsh function below
extern "C" {

  /*
  * @brief   EPICS iocsh callable function to call constructor
  *          for the drvAsynIsegHalService class.
  *
  * @param  [in]  portName The name of the asyn port driver to be created.
  * @param  [in]  interface Addr for HAL-service running on a CC24 or iCSmini ( HAL protocol )
  * @param  [in]  mpod specify wether or not the system host wiener MPOD slaves.
  */
  int drvAsynIsegHalServiceConfig( const char *portName, const char *interface, const char *icsCtrtype, epicsInt16 autoConnect ) {
    new drvAsynIsegHalService( portName, interface, icsCtrtype, autoConnect );
    return( asynSuccess );
  }

  static const iocshArg initIseghalServiceArg0 = { "portName",    iocshArgString };
  static const iocshArg initIseghalServiceArg1 = { "interface",   iocshArgString };
  static const iocshArg initIseghalServiceArg2 = { "icsCtrtype",  iocshArgString };
  static const iocshArg initIseghalServiceArg3 = { "autoConnect", iocshArgInt };
  static const iocshArg * const initIseghalServiceArgs[] = { &initIseghalServiceArg0, &initIseghalServiceArg1, &initIseghalServiceArg2, &initIseghalServiceArg3 };
  static const iocshFuncDef initIseghalServiceFuncDef = { "drvAsynIsegHalServiceConfig", 4, initIseghalServiceArgs };

  static void initIseghalServiceCallFunc( const iocshArgBuf *args ) {
    drvAsynIsegHalServiceConfig( args[0].sval, args[1].sval, args[2].sval, args[3].ival );
  }

  // iocsh function to set options for drv IsegHal Poller Thread
  static const iocshArg setOptArg0 = { "key", iocshArgString };
  static const iocshArg setOptArg1 = { "value",  iocshArgString };
  static const iocshArg * const setOptArgs[] = { &setOptArg0, &setOptArg1};
  static const iocshFuncDef setOptFuncDef = { "drvIsegHalPollerThreadSetOpt", 2, setOptArgs };

  /*
  * @brief   iocsh callable function to set options for the polling thread
  *          This function can be called from the iocsh via "drvIsegHalPollerThreadSetOpt( KEY, VALUE )".
  *
  * @param  [in]  KEY is the name of the option and VALUE the new value for the option.
  * KEYs are:
  *           Interval - set the wait time after going through the list of records inside polling thread
  *           debug     - Enable debug output of polling thread
  */
  static void setOptCallFunc( const iocshArgBuf *args )
  {
    // Set new intervall for polling thread
    if( strcmp( args[0].sval, "Interval" ) == 0 ) {
      double pollingFreq = 0.;
      int n = sscanf( args[1].sval, "%lf", &pollingFreq );
      if( 1 != n ) {
          fprintf( stderr, "\033[31;1mInvalid value for key '%s': %s\033[0m\n", args[0].sval, args[1].sval );
          return;
      }
			if(drvIsegHalPollerThread_) drvIsegHalPollerThread_->changeInterval( pollingFreq );

    }

    if( strcmp( args[0].sval, "RequestInterval" ) == 0 ) {
      double qRequestInterval = 0.;
      int n = sscanf( args[1].sval, "%lf", &qRequestInterval );
      if( 1 != n ) {
          fprintf( stderr, "\033[31;1mInvalid value for key '%s': %s\033[0m\n", args[0].sval, args[1].sval );
          return;
      }
        	if(drvIsegHalPollerThread_)  drvIsegHalPollerThread_->changeqRequestInterval( qRequestInterval );
    }

    // Set new debug level
    if( strcmp( args[0].sval, "Debug" ) == 0 ) {
      unsigned dbgLevel = 0;
      int n = sscanf( args[1].sval, "%u", &dbgLevel );
      if( 1 != n ) {
        fprintf( stderr, "\033[31;1mInvalid value for key '%s': %s\033[0m\n", args[0].sval, args[1].sval );
        return;
      }
      drvIsegHalPollerThread_->setDbgLvl( dbgLevel );
    }
  }

  /*
  * @brief   Register functions to EPICS
  */
  void drvAsynIsegHalServiceRegister( void ) {
    static bool firstTime = true;
    if ( firstTime ) {
      iocshRegister( &initIseghalServiceFuncDef, initIseghalServiceCallFunc );
      iocshRegister( &setOptFuncDef, setOptCallFunc );
      firstTime = false;
    }

  }
  epicsExportRegistrar( drvAsynIsegHalServiceRegister );
}
