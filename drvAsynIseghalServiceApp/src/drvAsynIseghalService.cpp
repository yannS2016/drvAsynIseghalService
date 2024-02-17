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
  epicsThreadSleep( 5 );

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
  if( name.compare( "AUTO" ) == 0 ) return true;

  std::vector< std::string >::iterator it;
  it = std::find( session_.begin(), session_.end(), name );
  if( it != session_.end() ) return true;
  return false;
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


	deviceName_ 	= epicsStrDup( portName );
	interface_  	= epicsStrDup( interface );
	deviceModel_	= epicsStrDup( icsCtrtype );
	
	if( autoConnect ) {
		
		pasynManager->exceptionDisconnect( pasynUserSelf );
		
    asynStatus status = this->connect( pasynUserSelf );
		
    if( status ) {
      asynPrint( pasynUserSelf, ASYN_TRACE_ERROR,
                 "drvAsynIseghalService, error calling connect - %s\n",
                 pasynUserSelf->errorMessage );
    }
  }

	createParam( P_ISEGHAL_SERVICE_SYSSTATUS_STRING,					asynParamUInt32Digital,		&P_SysStatus);
	createParam( P_ISEGHAL_SERVICE_LNSTATUS_STRING,						asynParamUInt32Digital,		&P_LineStatus);
	createParam( P_ISEGHAL_SERVICE_CRTSTATUS_STRING,					asynParamUInt32Digital,		&P_CrtStatus);
	createParam( P_ISEGHAL_SERVICE_CRTEVTSTATUS_STRING,				asynParamUInt32Digital,		&P_CrtEvtStatus);
	createParam( P_ISEGHAL_SERVICE_CRTEVTMASK_STRING,					asynParamUInt32Digital,		&P_CrtEvtMask);
	createParam( P_ISEGHAL_SERVICE_CRTCTRL_STRING,						asynParamUInt32Digital,		&P_CrtCtrl);
	createParam( P_ISEGHAL_SERVICE_MODSTATUS_STRING,					asynParamUInt32Digital,		&P_ModStatus);
	createParam( P_ISEGHAL_SERVICE_MODEVTSTATUS_STRING,				asynParamUInt32Digital,		&P_ModevtStatus);
	createParam( P_ISEGHAL_SERVICE_MODEVTMASK_STRING,					asynParamUInt32Digital,		&P_ModevtMask);
	createParam( P_ISEGHAL_SERVICE_MODCTRL_STRING,						asynParamUInt32Digital,		&P_ModCtrl);
	createParam( P_ISEGHAL_SERVICE_CHANSTATUS_STRING,					asynParamUInt32Digital,		&P_ChanStatus);
	createParam( P_ISEGHAL_SERVICE_CHANEVTSTATUS_STRING,			asynParamUInt32Digital,		&P_ChanEvtStatus);
	createParam( P_ISEGHAL_SERVICE_CHANEVTMASK_STRING,				asynParamUInt32Digital,		&P_ChanEvtMask);
	createParam( P_ISEGHAL_SERVICE_CHANCTRL_STRING,						asynParamUInt32Digital,		&P_ChanCtrl);
	// System items																																										
	createParam( P_ISEGHAL_SERVICE_SYSCRTNUM_STRING,					asynParamInt32,						&P_SysCrtNum);
	createParam( P_ISEGHAL_SERVICE_SYSCRTLIST_STRING,					asynParamInt32,						&P_SysCrtList);
	createParam( P_ISEGHAL_SERVICE_SYSMODNUM_STRING,					asynParamInt32,						&P_SysModNum);
	createParam( P_ISEGHAL_SERVICE_SYSCCTR_STRING,						asynParamInt32,						&P_SysCCtr);
	createParam( P_ISEGHAL_SERVICE_SYSLOGL_STRING,						asynParamInt32,						&P_SysLogL);
	createParam( P_ISEGHAL_SERVICE_SYSLOGPATH_STRING,					asynParamOctet,						&P_SysLogPath);
	createParam( P_ISEGHAL_SERVICE_SYSLINMODE_STRING,					asynParamInt32,						&P_SysLinMode);
	createParam( P_ISEGHAL_SERVICE_SYSSCONFIG_STRING,					asynParamInt32,						&P_SysSConfig);
	createParam( P_ISEGHAL_SERVICE_SYSETHNAME_STRING,					asynParamOctet,						&P_SysEthName);
	createParam( P_ISEGHAL_SERVICE_SYSETHADDR_STRING,					asynParamOctet,						&P_SysEthAddr);
	createParam( P_ISEGHAL_SERVICE_SYSSVERS_STRING,						asynParamOctet,						&P_SysSVers);
	createParam( P_ISEGHAL_SERVICE_SYSNETTOUT_STRING,					asynParamInt32,						&P_SysNetTout);
	createParam( P_ISEGHAL_SERVICE_SYSSSNAME_STRING,					asynParamOctet,						&P_SysSSname);
	// Can Line items
	createParam( P_ISEGHAL_SERVICE_LNBRATE_STRING,						asynParamInt32,						&P_LnBRate);
	createParam( P_ISEGHAL_SERVICE_LNMODNUM_STRING,						asynParamInt32,						&P_LnModNum);
	createParam( P_ISEGHAL_SERVICE_LNCRTNUM_STRING,						asynParamInt32,						&P_LnCrtNum);
	createParam( P_ISEGHAL_SERVICE_LNCRTLIST_STRING,					asynParamInt32,						&P_LnCrtList);
	createParam( P_ISEGHAL_SERVICE_LNMODLIST_STRING,					asynParamInt32,						&P_LnModList);
	createParam( P_ISEGHAL_SERVICE_LNLOGL_STRING,							asynParamInt32,						&P_LnLogL);
	// Crate items
	createParam( P_ISEGHAL_SERVICE_CRTSTAT_STRING,						asynParamInt32,						&P_CrtStat);
	createParam( P_ISEGHAL_SERVICE_CRTALIVE_STRING,						asynParamInt32,						&P_CrtAlive);
	createParam( P_ISEGHAL_SERVICE_CRTPON_STRING,							asynParamInt32,						&P_CrtPOn);
	createParam( P_ISEGHAL_SERVICE_CRTFSPEEP_STRING,					asynParamInt32,						&P_CrtFSpeed);
	createParam( P_ISEGHAL_SERVICE_CRTSN_STRING,							asynParamInt32,						&P_CrtSn);
	createParam( P_ISEGHAL_SERVICE_CRTDEVC_STRING,						asynParamInt32,						&P_CrtDevC);
	createParam( P_ISEGHAL_SERVICE_CRTFWRLS_STRING,						asynParamOctet,						&P_CrtFWRls);
	createParam( P_ISEGHAL_SERVICE_CRTFWNAME_STRING,					asynParamOctet,						&P_CrtFWName);
	createParam( P_ISEGHAL_SERVICE_CRTTEMP_STRING,						asynParamFloat64,					&P_CrtTemp);
	createParam( P_ISEGHAL_SERVICE_CRTSUPPLY_STRING,					asynParamFloat64,					&P_CrtSupply);
	createParam( P_ISEGHAL_SERVICE_CRTARTICLE_STRING,					asynParamOctet,						&P_CrtArticle);
	// device/module items
	createParam( P_ISEGHAL_SERVICE_MODSTAT_STRING,						asynParamInt32,						&P_ModStat);
	createParam( P_ISEGHAL_SERVICE_MODALIVE_STRING,						asynParamInt32,						&P_ModAlive);
	createParam( P_ISEGHAL_SERVICE_MODSN_STRING,							asynParamInt32,						&P_ModSn);
	createParam( P_ISEGHAL_SERVICE_MODDEVC_STRING,						asynParamInt32,						&P_ModDevC);
	createParam( P_ISEGHAL_SERVICE_MODFWRLS_STRING,						asynParamOctet,						&P_ModFWRls);
	createParam( P_ISEGHAL_SERVICE_MODFWNAME_STRING,					asynParamOctet,						&P_ModFWName);
	createParam( P_ISEGHAL_SERVICE_MODTEMP_STRING,						asynParamFloat64,					&P_ModTemp);
	createParam( P_ISEGHAL_SERVICE_MODSUPPLY_STRING,					asynParamFloat64,					&P_ModSupply);
	createParam( P_ISEGHAL_SERVICE_MODSRATE_STRING,						asynParamInt32,						&P_ModSRate);
	createParam( P_ISEGHAL_SERVICE_MODDFILTER_STRING,					asynParamInt32,						&P_ModDFilter);
	createParam( P_ISEGHAL_SERVICE_MODVRAMP_STRING,						asynParamFloat64,					&P_ModVRamp);
	createParam( P_ISEGHAL_SERVICE_MODCRAMP_STRING,						asynParamFloat64,					&P_ModCRamp);
	createParam( P_ISEGHAL_SERVICE_MODVLIM_STRING,						asynParamFloat64,					&P_ModVLim);
	createParam( P_ISEGHAL_SERVICE_MODILIM_STRING,						asynParamFloat64,					&P_ModILim);
	createParam( P_ISEGHAL_SERVICE_MODARTICLE_STRING,					asynParamOctet,						&P_ModArticle);
	createParam( P_ISEGHAL_SERVICE_MODDOCLR_STRING,						asynParamInt32,						&P_ModDoClr);
	createParam( P_ISEGHAL_SERVICE_MODFADJ_STRING,						asynParamInt32,						&P_ModFAdj);
	createParam( P_ISEGHAL_SERVICE_MODKEN_STRING,							asynParamInt32,						&P_ModKEn);
	// MICC option
	createParam( P_ISEGHAL_SERVICE_MODHVOK_STRING,						asynParamOctet,						&P_ModHVOk);
	// Channel items
	createParam( P_ISEGHAL_SERVICE_CHANTEMP_STRING, 					asynParamFloat64,					&P_ChanTemp);
	createParam( P_ISEGHAL_SERVICE_CHANVSET_STRING, 					asynParamFloat64,					&P_ChanVset);
	createParam( P_ISEGHAL_SERVICE_CHANISET_STRING,						asynParamFloat64,					&P_ChanISet);
	createParam( P_ISEGHAL_SERVICE_CHANVMOM_STRING,						asynParamFloat64,					&P_ChanVMom);
	createParam( P_ISEGHAL_SERVICE_CHANIMOM_STRING,						asynParamFloat64,					&P_ChanIMom);
	createParam( P_ISEGHAL_SERVICE_CHANVBOUNDS_STRING,				asynParamFloat64,					&P_ChanVBounds);
	createParam( P_ISEGHAL_SERVICE_CHANIBOUNDS_STRING,				asynParamFloat64,					&P_ChanIBounds);
	createParam( P_ISEGHAL_SERVICE_CHANVNOM_STRING,						asynParamFloat64,					&P_ChanVNom);
	createParam( P_ISEGHAL_SERVICE_CHANINOM_STRING,						asynParamFloat64,					&P_ChanINom);
	createParam( P_ISEGHAL_SERVICE_CHANDTRIPACT_STRING,				asynParamInt32,						&P_ChanDTripAct);
	createParam( P_ISEGHAL_SERVICE_CHANDTRIPTIME_STRING,			asynParamInt32,						&P_ChanDTripTime);
	createParam( P_ISEGHAL_SERVICE_CHANEXTINACT_STRING,    		asynParamInt32,						&P_ChanExtInAct);
	createParam( P_ISEGHAL_SERVICE_CHANTEMPTRIP_STRING,				asynParamInt32,						&P_ChanTempTrip);
	// Option Voltage controlled by temperature (VCT);
	createParam( P_ISEGHAL_SERVICE_CHANTEMPEXT_STRING,				asynParamFloat64,					&P_ChanTempEXT);
	createParam( P_ISEGHAL_SERVICE_CHANVCTCOEF_STRING,    		asynParamFloat64,					&P_ChanVctCoef);
	// Option STACK
	createParam( P_ISEGHAL_SERVICE_CHANRES_STRING,      			asynParamFloat64,					&P_ChanRes);
	createParam( P_ISEGHAL_SERVICE_CHANVRAMPPRIO_STRING,   		asynParamInt32,						&P_ChanVRampPrio);
	createParam( P_ISEGHAL_SERVICE_CHANVBOT_STRING,    				asynParamFloat64,					&P_ChanVBot);
	// Option Reversible
	createParam( P_ISEGHAL_SERVICE_CHANOUTMOD_STRING,      		asynParamInt32,						&P_ChanOutMod);
	createParam( P_ISEGHAL_SERVICE_CHANOUTMODLIST_STRING, 		asynParamInt32,						&P_ChanOutModList);
	createParam( P_ISEGHAL_SERVICE_CHANOUTPOL_STRING,    			asynParamInt32,						&P_ChanOutPol);
	createParam( P_ISEGHAL_SERVICE_CHANOUTPOLLIST_STRING,			asynParamInt32,						&P_ChanOutPolList);
	createParam( P_ISEGHAL_SERVICE_CHANVMOD_STRING,    				asynParamInt32,						&P_ChanVMod);
	createParam( P_ISEGHAL_SERVICE_CHANVMODLIST_STRING,				asynParamInt32,						&P_ChanVModList);
	createParam( P_ISEGHAL_SERVICE_CHANCMOD_STRING,						asynParamInt32,						&P_ChanCMod);
	createParam( P_ISEGHAL_SERVICE_CHANCMODLIST_STRING,				asynParamInt32,						&P_ChanCModList);
	createParam( P_ISEGHAL_SERVICE_CHANVRAMPSPUP_STRING,			asynParamFloat64,					&P_ChanVRampSpUp);
	createParam( P_ISEGHAL_SERVICE_CHANVRAMSPDN_STRING,				asynParamFloat64,					&P_ChanVRAMSpDn);
	createParam( P_ISEGHAL_SERVICE_CHANVRAMPSPMIN_STRING,			asynParamFloat64,					&P_ChanVRampSpMin);
	createParam( P_ISEGHAL_SERVICE_CHANVRAMPSPMAX_STRING,			asynParamFloat64,					&P_ChanVRampSpMax);
	createParam( P_ISEGHAL_SERVICE_CHANCRAMPSPUP_STRING,			asynParamFloat64,					&P_ChanCRampSpUp);
	createParam( P_ISEGHAL_SERVICE_CHANCRAMPSPDN_STRING,   		asynParamFloat64,					&P_ChanCRampSpDn);
	createParam( P_ISEGHAL_SERVICE_CHANCRAMPSPMIN_STRING,			asynParamFloat64,					&P_ChanCRampSpMin);
	createParam( P_ISEGHAL_SERVICE_CHANCRAMPSPMAX_STRING,			asynParamFloat64,					&P_ChanCRampSpMax);
	//Items for 0MPV (W-IE-NE-R); Modules
	createParam( P_ISEGHAL_SERVICE_WNCHANTVMOM_STRING,				asynParamFloat64,					&P_WnChanTVMom);
	createParam( P_ISEGHAL_SERVICE_WNCHANVRRATE_STRING,				asynParamFloat64,					&P_WnChanVRRate);
	createParam( P_ISEGHAL_SERVICE_WNCHANVFRATE_STRING,				asynParamFloat64,					&P_WnChanVFRate);
	createParam( P_ISEGHAL_SERVICE_WNCHANVLTRIP_STRING,				asynParamFloat64,					&P_WnChanVLTrip);
	createParam( P_ISEGHAL_SERVICE_WNCHANVTRIP_STRING,				asynParamFloat64,					&P_WnChanVTrip);
	createParam( P_ISEGHAL_SERVICE_WNCHANTVTRIP_STRING,				asynParamFloat64,					&P_WnChanTVTrip);
	createParam( P_ISEGHAL_SERVICE_WNCHANVCTRIP_STRING,				asynParamFloat64,					&P_WnChanVCtrIP);
	createParam( P_ISEGHAL_SERVICE_WNCHANVPTRIP_STRING,				asynParamFloat64,					&P_WnChanVPTrip);
	createParam( P_ISEGHAL_SERVICE_WNCHANVTRIPMAX_STRING,			asynParamFloat64,					&P_WnChanVTripMax);
	createParam( P_ISEGHAL_SERVICE_WNCHANTVTRIPMAX_STRING,		asynParamFloat64,					&P_WnChanTVTripMax);
	createParam( P_ISEGHAL_SERVICE_WNCHANCTRIPMAX_STRING,			asynParamFloat64,					&P_WnChanCTripMax);
	createParam( P_ISEGHAL_SERVICE_WNCHANTEMPTRIPMAX_STRING,	asynParamFloat64,					&P_WnChanTTripMax);
	createParam( P_ISEGHAL_SERVICE_WNCHANPTRIPMAX_STRING,			asynParamFloat64,					&P_WnChanPTripMax);
	createParam( P_ISEGHAL_SERVICE_WNCHANVLTRIPTIME_STRING,		asynParamInt32,						&P_WnChanVLTripTime);
	createParam( P_ISEGHAL_SERVICE_WNCHANVTRIPTIME_STRING,		asynParamInt32,						&P_WnChanVTripTime);
	createParam( P_ISEGHAL_SERVICE_WNCHANTVTRIPTIME_STRING,		asynParamInt32,						&P_WnChanTVTripTime);
	createParam( P_ISEGHAL_SERVICE_WNCHANCTRIPTIME_STRING,		asynParamInt32,						&P_WnChanCtrIPTime);
	createParam( P_ISEGHAL_SERVICE_WNCHANTEMPTRIPTIME_STRING,	asynParamInt32,						&P_WnChanTempTripTime);
	createParam( P_ISEGHAL_SERVICE_WNCHANPTRIPTIME_STRING,		asynParamInt32,						&P_WnChanPTripTime);
	createParam( P_ISEGHAL_SERVICE_WNCHANTOUTRIPTIME_STRING,	asynParamInt32,						&P_WnChanTOutripTime);
	createParam( P_ISEGHAL_SERVICE_WNCHANGROUP_STRING,				asynParamInt32,						&P_WnChanGROUp);
	createParam( P_ISEGHAL_SERVICE_WNCHANNAME_STRING,					asynParamOctet,						&P_WnChanName);
	createParam( P_ISEGHAL_SERVICE_WNCHANTRIPACT_STRING,			asynParamInt32,						&P_WnChanTripAct);
	createParam( P_ISEGHAL_SERVICE_WNCHANVLTRIPACT_STRING,		asynParamInt32,						&P_WnChanVLTripAct);
	createParam( P_ISEGHAL_SERVICE_WNCHANVTRIPACT_STRING,			asynParamInt32,						&P_WnChanVTripAct);
	createParam( P_ISEGHAL_SERVICE_WNCHANTVTRIPACT_STRING,		asynParamInt32,						&P_WnChanTVTripAct);
	createParam( P_ISEGHAL_SERVICE_WNCHANCTRIPACT_STRING,			asynParamInt32,						&P_WnChanCtrIPAct);
	createParam( P_ISEGHAL_SERVICE_WNCHANTEMPTRIPACT_STRING,	asynParamInt32,						&P_WnChanTempTripAct);
	createParam( P_ISEGHAL_SERVICE_WNCHANPTRIPACT_STRING,			asynParamInt32,						&P_WnChanPTripAct);
	createParam( P_ISEGHAL_SERVICE_WNCHANTOUTTRIPACT_STRING,	asynParamInt32,						&P_WnChanTOutTripAct);
	createParam( P_ISEGHAL_SERVICE_WNCHANUSRCONFIG_STRING,		asynParamInt32,						&P_WnChanUsRConfig);

}

asynStatus drvAsynIseghalService::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeFloat64( asynUser *pasynUser, epicsFloat64 value ){ return asynSuccess; }
asynStatus drvAsynIseghalService::readFloat64( asynUser *pasynUser, epicsFloat64 *value ){ return asynSuccess; }
asynStatus drvAsynIseghalService::readInt32(asynUser *pasynUser, epicsInt32 *value){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeInt32(asynUser *pasynUser, epicsInt32 value){ return asynSuccess; }
asynStatus drvAsynIseghalService::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason){ return asynSuccess; }
asynStatus drvAsynIseghalService::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual){ return asynSuccess; }


asynStatus drvAsynIseghalService::drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize ){ return asynSuccess; }

/**
* @brief       Connect driver to device
* @param [in]  pasynUser  pasynUser structure that encodes the reason and address.
* @return      in case of no error occured asynSuccess is returned. Otherwise
*              asynError or asynTimeout is returned. A error message is stored
*              in pasynUser->errorMessage.
*/
asynStatus drvAsynIseghalService::connect(asynUser *pasynUser) {
	
	if( devConnected( deviceName_ ) )	{
		epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "%s: Link to %s already open!", deviceName_, interface_ );
    return asynError;
	}
	
  asynPrint( pasynUser, ASYN_TRACEIO_DRIVER,
             "%s: Open connection to %s\n", deviceName_, interface_ );
	
	if( !devConnect(deviceName_, interface_) ) {
    epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "%s: Can't open %s: %s", deviceName_, interface_, strerror( errno ) );
    return asynError;
	}
	/* check devices is responsive.
	* modelBA: iseg iCS based HV system controller model: cc24 for crate type 
	* icsmini based system will expose the crate controller module model i.e MICC.	
	*/
	
	char modelBA[15];
	// icsmini default
	strcpy(modelBA, "0.0.Article");
	
	if( strcmp( deviceModel_, "cc24" ) == 0 ) {
		strcpy(modelBA, "0.100.Article");
	} 
	
	
  IsegItem model = iseg_getItem( deviceName_, modelBA );
  if( strcmp( model.quality, ISEG_ITEM_QUALITY_OK ) != 0 ) {

    epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "\033[31;1m%s: Error while reading from device (Q: %s): %s\033[0m", deviceName_, model.quality, strerror( errno ) );
    return asynError;
  }
	else {
				printf("\033[0;32m iCS controller model: %s\n\033[0m", model.value);
	}
	pasynManager->exceptionConnect( pasynUser );
	
	return asynSuccess; 

}

asynStatus drvAsynIseghalService::disconnect(asynUser *pasynUser) {
	
	asynPrint( pasynUser, ASYN_TRACEIO_DRIVER,
             "%s: disconnect %s\n", deviceName_, interface_ );
						 
	if(!devDisconnect( deviceName_ )) {
		epicsSnprintf( pasynUser->errorMessage,pasynUser->errorMessageSize,
                   "%s: cannot diconnect from %s ", deviceName_, interface_ );
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