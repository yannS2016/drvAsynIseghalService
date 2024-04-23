//******************************************************************************
// Copyright (C) 2014 Yann Stephen Mandza <yann.mandza@ess.se>
//                    - European Spallation Source - ESS
//
// This file is part of drvAsynIseghalService
//
// drvAsynIseghalService is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// drvAsynIseghalService is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// version 1.0.0; Feb. 14, 2024
//******************************************************************************

#ifndef __ASYN_ISEGHAL_SERVICE_H__
#define __ASYN_ISEGHAL_SERVICE_H__

#include <map>
#include <vector>
#include "asynPortDriver.h"
#include <epicsMutex.h>
/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
//_____ D E F I N I T I O N S __________________________________________________

// These are the drvInfo strings that are used to identify the parameters.
// They are used by asyn clients, including standard asyn device support
#define P_ISEGHAL_SERVICE_SYSSTATUS_STRING				"SystemStatus"					/*asynUInt32Digital,  r */
#define P_ISEGHAL_SERVICE_LNSTATUS_STRING					"LineStatus"						/*asynUInt32Digital,  r */
#define P_ISEGHAL_SERVICE_CRTSTATUS_STRING				"CrateStatus"						/* asynUInt32Digital,  r/w */
#define P_ISEGHAL_SERVICE_CRTEVTSTATUS_STRING			"CrateEventStatus"			/* asynUInt32Digital,  r/w */
#define P_ISEGHAL_SERVICE_CRTEVTMASK_STRING				"CrateEventMask"				/* asynUInt32Digital,  r/w */
#define P_ISEGHAL_SERVICE_CRTCTRL_STRING					"CrateControl"					/* asynUInt32Digital,  r/w */
																									
#define P_ISEGHAL_SERVICE_MODSTATUS_STRING				"ModuleStatus"					/* asynUInt32Digital,  r */
#define P_ISEGHAL_SERVICE_MODEVTSTATUS_STRING			"ModuleEventStatus"			/* asynUInt32Digital,  r/w */
#define P_ISEGHAL_SERVICE_MODEVTMASK_STRING				"ModuleEventMask"				/* asynUInt32Digital,  r/w */
#define P_ISEGHAL_SERVICE_MODCTRL_STRING					"ModuleControl"					/* asynUInt32Digital,  r/w */
#define P_ISEGHAL_SERVICE_CHANSTATUS_STRING				"ChannelStatus"					/* asynUInt32Digital,  r/w */
#define P_ISEGHAL_SERVICE_CHANEVTSTATUS_STRING		"ChannelEventStatus"		/* asynUInt32Digital,  r/w */
#define P_ISEGHAL_SERVICE_CHANEVTMASK_STRING			"ChannelEventMask"			/* asynUInt32Digital,  r/w */
#define P_ISEGHAL_SERVICE_CHANCTRL_STRING					"ChannelControl"				/* asynUInt32Digital,  r/w */

// System items
#define P_ISEGHAL_SERVICE_SYSCRTNUM_STRING				"SystemCrateNumber"			/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_SYSCRTLIST_STRING				"SystemCrateList"				/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_SYSMODNUM_STRING				"SystemModuleNumber"		/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_SYSCCTR_STRING					"CycleCounter"					/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_SYSLOGL_STRING					"SystemLogLevel"				/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_SYSLOGPATH_STRING				"LogPath"         			/* asynOctetWrite,r/w */
#define P_ISEGHAL_SERVICE_SYSLINMODE_STRING				"LiveInsertionMode"			/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_SYSSCONFIG_STRING				"SaveConfiguration"			/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_SYSETHNAME_STRING				"EthName"								/* asynOctetRead,  r/w */
#define P_ISEGHAL_SERVICE_SYSETHADDR_STRING				"EthAddress"						/* asynOctetRead,  r/w */
#define P_ISEGHAL_SERVICE_SYSSVERS_STRING					"ServerVersion"					/* asynOctetRead,  r/w */
#define P_ISEGHAL_SERVICE_SYSNETTOUT_STRING				"NetworkTimeout"				/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_SYSSSNAME_STRING				"SessionName"						/* asynOctetRead,  r/w */

// Can Line items
#define P_ISEGHAL_SERVICE_LNBRATE_STRING					"BitRate"								/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_LNMODNUM_STRING					"LineModuleNumber"			/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_LNCRTNUM_STRING					"LineCrateNumber"				/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_LNCRTLIST_STRING				"LineCrateList"					/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_LNMODLIST_STRING				"ModuleList"						/*asynInt32,       r/w*/
#define P_ISEGHAL_SERVICE_LNLOGL_STRING						"LineLogLevel"					/* asynInt32,      r/w */

// Crate items
#define P_ISEGHAL_SERVICE_CRTSTAT_STRING					"Connected"							/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_CRTALIVE_STRING					"Alive"									/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_CRTPON_STRING						"PowerOn"         			/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_CRTFSPEEP_STRING				"FanSpeed"							/*** asynFloat64,  r/w */
#define P_ISEGHAL_SERVICE_CRTSN_STRING						"SerialNumber"					/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_CRTDEVC_STRING					"DeviceClass"						/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_CRTFWRLS_STRING					"CrateFirmwareRelease"	/* asynOctetRead,  r/w */
#define P_ISEGHAL_SERVICE_CRTFWNAME_STRING				"CrateFirmwareName"			/* asynOctetRead,  r/w */
#define P_ISEGHAL_SERVICE_CRTTEMP_STRING					"CrateTemperature"			/* asynFloat64,    r */ 
#define P_ISEGHAL_SERVICE_CRTSUPPLY_STRING				"CrateSupply"           /* asynFloat64,r */
#define P_ISEGHAL_SERVICE_CRTARTICLE_STRING				"CrateArticle"					/* asynOctetRead,  r/w */

// device/module items
#define P_ISEGHAL_SERVICE_MODSTAT_STRING					"ModuleConnected"   		/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_MODALIVE_STRING					"ModuleAlive"        		/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_MODSN_STRING						"ModuleSerialNumber"		/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_MODDEVC_STRING					"ModuleDeviceClass"			/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_MODFWRLS_STRING					"ModuleFirmwareRelease"	/* asynOctetRead,  r/w */
#define P_ISEGHAL_SERVICE_MODFWNAME_STRING				"ModuleFirmwareName"		/* asynOctetRead,  r/w */
#define P_ISEGHAL_SERVICE_MODTEMP_STRING					"ModuleTemperature" 		/* asynFloat64,    r */ 
#define P_ISEGHAL_SERVICE_MODSUPPLY_STRING				"ModuleSupply"          /* asynFloat64,    r */
#define P_ISEGHAL_SERVICE_MODSRATE_STRING					"SampleRate"     				/* asynInt32,      r */
#define P_ISEGHAL_SERVICE_MODDFILTER_STRING				"DigitalFilter"    			/* asynInt32,      r */
#define P_ISEGHAL_SERVICE_MODVRAMP_STRING					"VoltageRampSpeed"			/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_MODCRAMP_STRING					"CurrentRampSpeed"  		/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_MODVLIM_STRING					"VoltageLimit"      		/* asynFloat64,    r */  
#define P_ISEGHAL_SERVICE_MODILIM_STRING					"CurrentLimit"    			/* asynFloat64,    r */  
#define P_ISEGHAL_SERVICE_MODARTICLE_STRING				"ModuleArticle"					/* asynOctetRead,  r/w */
// MICC option
#define P_ISEGHAL_SERVICE_MODHVOK_STRING					"HighVoltageOk"					/* asynOctetWrite, r/w */
#define P_ISEGHAL_SERVICE_MODDOCLR_STRING					"DoClear"								/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_MODFADJ_STRING					"FineAdjustment"				/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_MODKEN_STRING						"KillEnable" 						/* asynInt32,      r/w */  

// Channel items
#define P_ISEGHAL_SERVICE_CHANTEMP_STRING         "Temperature"     			/* asynFloat64,    r */  
#define P_ISEGHAL_SERVICE_CHANVSET_STRING         "0.0.0.VoltageSet"       			/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANISET_STRING         "0.0.0.CurrentSet"     				/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANVMOM_STRING         "0.0.0.VoltageMeasure"   			/* asynFloat64,    r */  
#define P_ISEGHAL_SERVICE_CHANIMOM_STRING         "0.0.0.CurrentMeasure"    		/* asynFloat64,    r */  
#define P_ISEGHAL_SERVICE_CHANVBOUNDS_STRING      "VoltageBounds" 				/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANIBOUNDS_STRING      "CurrentBounds"  				/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANVNOM_STRING         "VoltageNominal" 				/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANINOM_STRING         "CurrentNominal"  			/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANDTRIPACT_STRING     "DelayedTripAction" 		/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_CHANDTRIPTIME_STRING    "DelayedTripTime"				/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_CHANEXTINACT_STRING     "ExternalInhibitAction"	/* asynInt32,      r/w */
#define P_ISEGHAL_SERVICE_CHANTEMPTRIP_STRING     "TemperatureTrip" 			/* asynInt32,      r/w */

// Option Voltage controlled by temperature (VCT)
#define P_ISEGHAL_SERVICE_CHANTEMPEXT_STRING      "TemperatureExternal"		/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANVCTCOEF_STRING      "VctCoefficient"				/* asynFloat64,    r/w */

// Option STACK
#define P_ISEGHAL_SERVICE_CHANRES_STRING      		"Resistance"      			/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANVRAMPPRIO_STRING    "VoltageRampPriority"		/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANVBOT_STRING    			"VoltageBottom"					/* asynFloat64,    r/w */

// Option Reversible
#define P_ISEGHAL_SERVICE_CHANOUTMOD_STRING      	"OutputMode"      			/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANOUTMODLIST_STRING   "OutputModeList" 				/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANOUTPOL_STRING    		"OutputPolarity"				/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANOUTPOLLIST_STRING		"OutputPolarityList"		/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANVMOD_STRING    			"VoltageMode"						/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANVMODLIST_STRING			"VoltageModeList"				/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANCMOD_STRING					"CurrentMode"      			/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANCMODLIST_STRING			"CurrentModeList"				/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANVRAMPSPUP_STRING		"VoltageRampSpeedUp"		/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANVRAMSPDN_STRING			"VoltageRampSpeedDown"	/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANVRAMPSPMIN_STRING		"VoltageRampSpeedMin"		/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANVRAMPSPMAX_STRING		"VoltageRampSpeedMax"		/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANCRAMPSPUP_STRING		"CurrentRampSpeedUp"		/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANCRAMPSPDN_STRING    "CurrentRampSpeedDown"	/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANCRAMPSPMIN_STRING		"CurrentRampSpeedMin"		/* asynFloat64,    r/w */
#define P_ISEGHAL_SERVICE_CHANCRAMPSPMAX_STRING		"CurrentRampSpeedMax"		/* asynFloat64,        r/w */

//Items for 0MPV (W-IE-NE-R) Modules
#define P_ISEGHAL_SERVICE_WNCHANTVMOM_STRING				"VoltageTerminalMeasure"		/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANVRRATE_STRING				"VoltageRiseRate"						/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANVFRATE_STRING				"VoltageFallRate"						/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANVLTRIP_STRING				"VoltageLowTrip"						/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANVTRIP_STRING				"VoltageTrip"								/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANTVTRIP_STRING				"VoltageTerminalTrip"				/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANVCTRIP_STRING				"CurrentTrip"								/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANVPTRIP_STRING				"PowerTrip"									/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANVTRIPMAX_STRING			"VoltageTripMax"						/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANTVTRIPMAX_STRING		"VoltageTerminalTripMax"		/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANCTRIPMAX_STRING			"CurrentTripMax"						/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANTEMPTRIPMAX_STRING	"TemperatureTripMax"				/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANPTRIPMAX_STRING			"PowerTripMax"							/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANVLTRIPTIME_STRING		"VoltageLowTripTime"				/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANVTRIPTIME_STRING		"VoltageTripTime"						/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANTVTRIPTIME_STRING		"VoltageTerminalTripTime"		/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANCTRIPTIME_STRING		"CurrentTripTime"						/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANTEMPTRIPTIME_STRING	"TemperatureTripTime"				/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANPTRIPTIME_STRING		"PowerTripTime"							/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANTOUTRIPTIME_STRING	"TimeoutTripTime"						/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANGROUP_STRING				"Group"											/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANNAME_STRING					"Name"											/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANTRIPACT_STRING			"TripAction"								/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANVLTRIPACT_STRING		"VoltageLowTripAction"			/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANVTRIPACT_STRING			"VoltageTripAction"					/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANTVTRIPACT_STRING		"VoltageTerminalTripAction"	/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANCTRIPACT_STRING			"CurrentTripAction"					/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANTEMPTRIPACT_STRING	"TemperatureTripAction"			/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANPTRIPACT_STRING			"PowerTripAction"						/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANTOUTTRIPACT_STRING	"TimeoutTripAction"					/* asynFloat64,        r/w */
#define P_ISEGHAL_SERVICE_WNCHANUSRCONFIG_STRING		"UserConfigFlags"						/* asynFloat64,        r/w */


/* maximum number of iseghal supported items 
	* system - system information
	* line - hardware interface or virtual line (0..15)
	* device - module (0..63), crate (1000, 2000..2015)
	* channel - high voltage channels (0..63)
*/
// default: 5 can line max , 21 devices max, 10 channels max
#define NITEMS 1050


/* asynPortDriver for ISEG  iCS based HV systems (CC24 and iCSmini) using iseghal service library
	*
	* This asynPortDriver is the device support for ISEG Spezialelektronik GmbH iCS HV system starting at version 2.8.0 using the isegHALService
	* to provide a multiple devices access to modules in an ECH or MPOD crate in combination with CC24 (ECHxxx & wiener MPOD crate access) 
	* or iCSmini controller ( ECHxx crate with MMS & MMC slots). These compatible HV-modules (EBS, EDS, EHS, ESS,NHS, NHQ, HPS, FPS, SHQ ) are also supported. 
	*
*/
class drvAsynIseghalService : public asynPortDriver {
	public:
		drvAsynIseghalService( const char *portName, const char *interface, const char *icsCtrtype, epicsInt16 autoConnect );
		// These are the methods that we override from asynPortDriver
		virtual asynStatus readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask );
		virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );
		virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value );
		virtual asynStatus readFloat64( asynUser *pasynUser, epicsFloat64 *value );
		virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
		virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
		virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);
		virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
		virtual	asynStatus connect(asynUser *pasynUser);
		virtual	asynStatus disconnect(asynUser *pasynUser);
		
		asynUser          *self_;	
		epicsMutexId      hookmutexId;
		int devConnect( std::string const& name, std::string const& interface );
 		int devConnected( std::string const& name );
		int devDisconnect( std::string const& name );
		int hasIsegHalItem (const char *item);
	protected:
		// Values used for pasynUser->reason, and indexes into the parameter library.
		int P_SysStatus;			//index for Parameter "SystemStatus"
		int P_LineStatus;			//index for Parameter "LineStatus"
		int P_CrtStatus;			//index for Parameter "CrateStatus"
		int P_CrtEvtStatus;		//index for Parameter "CrateEventStatus"
		int P_CrtEvtMask;			//index for Parameter "CrateEventMask"
		int P_CrtCtrl;				//index for Parameter "CrateControl"
													
		int P_ModStatus;			//index for Parameter "ModuleStatus"
		int P_ModevtStatus;		//index for Parameter "ModuleEventStatus"
		int P_ModevtMask;			//index for Parameter "ModuleEventMask"
		int P_ModCtrl;				//index for Parameter "ModuleControl"
		int P_ChanStatus;			//index for Parameter "ChannelStatus"
		int P_ChanEvtStatus;	//index for Parameter "ChannelEventStatus"
		int P_ChanEvtMask;		//index for Parameter "ChannelEventMask"
		int P_ChanCtrl;				//index for Parameter "ChannelControl"
													
		// System items       
		int P_SysCrtNum;			//index for Parameter "SystemCrateNumber"
		int P_SysCrtList;			//index for Parameter "SystemCrateList"
		int P_SysModNum;			//index for Parameter "SystemModuleNumber"
		int P_SysCCtr;				//index for Parameter "CycleCounter"
		int P_SysLogL;				//index for Parameter "SystemLogLevel"
		int P_SysLogPath;			//index for Parameter "LogPath"
		int P_SysLinMode;			//index for Parameter "LiveInsertionMode"
		int P_SysSConfig;			//index for Parameter "SaveConfiguration"
		int P_SysEthName;			//index for Parameter "EthName"
		int P_SysEthAddr;			//index for Parameter "EthAddress"
		int P_SysSVers;				//index for Parameter "ServerVersion"
		int P_SysNetTout;			//index for Parameter "NetworkTimeout"
		int P_SysSSname;			//index for Parameter "SessionName"
													
		// Can Line items     
		int P_LnBRate;				//index for Parameter "BitRate"
		int P_LnModNum;				//index for Parameter "LineModuleNumber"
		int P_LnCrtNum;				//index for Parameter "LineCrateNumber"
		int P_LnCrtList;			//index for Parameter "LineCrateList"
		int P_LnModList;			//index for Parameter "ModuleList"
		int P_LnLogL;					//index for Parameter "LineLogLevel"
													
		// Crate items        
		int P_CrtStat;				//index for Parameter "Connected"
		int P_CrtAlive;				//index for Parameter "Alive"
		int P_CrtPOn;					//index for Parameter "PowerOn"
		int P_CrtFSpeed;			//index for Parameter "FanSpeed"
		int P_CrtSn;					//index for Parameter "SerialNumber"
		int P_CrtDevC;				//index for Parameter "DeviceClass"
		int P_CrtFWRls;				//index for Parameter "CrateFirmwareRelease"
		int P_CrtFWName;			//index for Parameter "CrateFirmwareName"
		int P_CrtTemp;				//index for Parameter "CrateTemperature"
		int P_CrtSupply;			//index for Parameter "Supply"
		int P_CrtArticle;			//index for Parameter "CrateArticle"
													
		// device/module items
		int P_ModStat;				//index for Parameter "ModuleConnected"
		int P_ModAlive;				//index for Parameter "ModuleAlive"
		int P_ModSn;					//index for Parameter "ModuleSerialNumber"
		int P_ModDevC;				//index for Parameter "ModuleDeviceClass"
		int P_ModFWRls;				//index for Parameter "ModuleFirmwareRelease"
		int P_ModFWName;			//index for Parameter "ModuleFirmwareName"
		int P_ModTemp;				//index for Parameter "ModuleTemperature"
		int P_ModSupply;			//index for Parameter "Supply"
		int P_ModSRate;				//index for Parameter "SampleRate"
		int P_ModDFilter;			//index for Parameter "DigitalFilter"
		int P_ModVRamp;				//index for Parameter "VoltageRampSpeed"
		int P_ModCRamp;				//index for Parameter "CurrentRampSpeed"
		int P_ModVLim;				//index for Parameter "VoltageLimit"
		int P_ModILim;				//index for Parameter "CurrentLimit"
		int P_ModArticle;			//index for Parameter "ModuleArticle"
		// MICC option        
		int P_ModHVOk;				//index for Parameter "HighVoltageOk"
		int P_ModDoClr;				//index for Parameter "DoClear"
		int P_ModFAdj;				//index for Parameter "FineAdjustment"
		int P_ModKEn;					//index for Parameter "KillEnable"
													
		// Channel items      
		int P_ChanTemp;       //index for Parameter "Temperature"
		int P_ChanVset;       //index for Parameter "VoltageSet"
		int P_ChanISet;       //index for Parameter "CurrentSet"
		int P_ChanVMom;       //index for Parameter "VoltageMeasure"
		int P_ChanIMom;       //index for Parameter "CurrentMeasure"
		int P_ChanVBounds;    //index for Parameter "VoltageBounds"
		int P_ChanIBounds;    //index for Parameter "CurrentBounds"
		int P_ChanVNom;       //index for Parameter "VoltageNominal"
		int P_ChanINom;       //index for Parameter "CurrentNominal"
		int P_ChanDTripAct;   //index for Parameter "DelayedTripAction"
		int P_ChanDTripTime;	//index for Parameter "DelayedTripTime"
		int P_ChanExtInAct;		//index for Parameter "ExternalInhibitAction"
		int P_ChanTempTrip;		//index for Parameter "TemperatureTrip"
		// Option Voltage controlled by temperature (VCT)
		int P_ChanTempEXT;		//index for Parameter "TemperatureExternal"
		int P_ChanVctCoef;		//index for Parameter "VctCoefficient"
		// Option STACK
		int P_ChanRes;				//index for Parameter "Resistance"
		int P_ChanVRampPrio;	//index for Parameter "VoltageRampPriority"
		int P_ChanVBot;				//index for Parameter "VoltageBottom"
		// Option Reversible
		int P_ChanOutMod;			//index for Parameter "OutputMode"
		int P_ChanOutModList;	//index for Parameter "OutputModeList"
		int P_ChanOutPol;			//index for Parameter "OutputPolarity"
		int P_ChanOutPolList;	//index for Parameter "OutputPolarityList"
		int P_ChanVMod;    		//index for Parameter "VoltageMode"
		int P_ChanVModList;		//index for Parameter "VoltageModeList"
		int P_ChanCMod;				//index for Parameter "CurrentMode"
		int P_ChanCModList;		//index for Parameter "CurrentModeList"
		int P_ChanVRampSpUp;	//index for Parameter "VoltageRampSpeedUp"
		int P_ChanVRAMSpDn;		//index for Parameter "VoltageRampSpeedDown"
		int P_ChanVRampSpMin;	//index for Parameter "VoltageRampSpeedMin"
		int P_ChanVRampSpMax;	//index for Parameter "VoltageRampSpeedMax"
		int P_ChanCRampSpUp;	//index for Parameter "CurrentRampSpeedUp"
		int P_ChanCRampSpDn;	//index for Parameter "CurrentRampSpeedDown"
		int P_ChanCRampSpMin;	//index for Parameter "CurrentRampSpeedMin"
		int P_ChanCRampSpMax;	//index for Parameter "CurrentRampSpeedMax"

		//Option 0MPV/MPOD (W-IE-NE-R) Modules
		int P_WnChanTVMom;				//index for Parameter "VoltageTerminalMeasure"
		int P_WnChanVRRate;				//index for Parameter "VoltageRiseRate"
		int P_WnChanVFRate;				//index for Parameter "VoltageFallRate"
		int P_WnChanVLTrip;				//index for Parameter "VoltageLowTrip"
		int P_WnChanVTrip;				//index for Parameter "VoltageTrip"
		int P_WnChanTVTrip;				//index for Parameter "VoltageTerminalTrip"
		int P_WnChanVCtrIP;				//index for Parameter "CurrentTrip"
		int P_WnChanVPTrip;				//index for Parameter "PowerTrip"
		int P_WnChanVTripMax;			//index for Parameter "VoltageTripMax"
		int P_WnChanTVTripMax;		//index for Parameter "VoltageTerminalTripMax"
		int P_WnChanCTripMax;			//index for Parameter "CurrentTripMax"
		int P_WnChanTTripMax;			//index for Parameter "TemperatureTripMax"
		int P_WnChanPTripMax;			//index for Parameter "PowerTripMax"
		int P_WnChanVLTripTime;		//index for Parameter "VoltageLowTripTime"
		int P_WnChanVTripTime;		//index for Parameter "VoltageTripTime"
		int P_WnChanTVTripTime;		//index for Parameter "VoltageTerminalTripTime"
		int P_WnChanCtrIPTime;		//index for Parameter "CurrentTripTime"
		int P_WnChanTempTripTime;	//index for Parameter "TemperatureTripTime"
		int P_WnChanPTripTime;		//index for Parameter "PowerTripTime"
		int P_WnChanTOutripTime;	//index for Parameter "TimeoutTripTime"
		int P_WnChanGROUp;				//index for Parameter "Group"
		int P_WnChanName;					//index for Parameter "Name"
		int P_WnChanTripAct;			//index for Parameter "TripAction"
		int P_WnChanVLTripAct;		//index for Parameter "VoltageLowTripAction"
		int P_WnChanVTripAct;			//index for Parameter "VoltageTripAction"
		int P_WnChanTVTripAct;		//index for Parameter "VoltageTerminalTripAction"
		int P_WnChanCtrIPAct;			//index for Parameter "CurrentTripAction"
		int P_WnChanTempTripAct;	//index for Parameter "TemperatureTripAction"
		int P_WnChanPTripAct;			//index for Parameter "PowerTripAction"
		int P_WnChanTOutTripAct;	//index for Parameter "TimeoutTripAction"
		int P_WnChanUsRConfig;		//index for Parameter "UserConfigFlags"
		int items[NITEMS];

#define FIRST_ISEGHAL_SERVICE_CMD P_SysStatus
#define LAST_ISEGHAL_SERVICE_CMD  P_WnChanUsRConfig

	private:
	
       
		char		*deviceSession_;
		char		*interface_;
		char		*deviceModel_; 					
		int 		itemIndex;
		epicsUInt32					pollTime_;
		std::vector< std::string > session_;
		std::map<std::string, epicsUInt32> isegHalItemsLookup;
		bool initStatus_;

	

};

#define NUM_ISEGHAL_SERVICE_PARAMS (&LAST_ISEGHAL_SERVICE_CMD - &FIRST_ISEGHAL_SERVICE_CMD + 1 + NITEMS)


#endif
