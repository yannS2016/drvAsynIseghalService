//******************************************************************************
// Copyright (C) 2014 Yann Stephen Mandza <yann.mandza@ess.se>
//                    - European Spallation Source - ESS
//
// This file is part of drvAsynIsegHalService
//
// drvAsynIsegHalService is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// drvAsynIsegHalService is distributed in the hope that it will be useful,
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

#ifndef __DRVASYNISEGHALSERVICE_H__
#define __DRVASYNISEGHALSERVICE_H__

#include <map>
#include <vector>
#include <list>
#include <epicsThread.h>
#include <asynPortDriver.h>
//#include <drvIsegHalPollerThread.h>

/* isegHAL includes */
#include <isegremoteapi.h>

/* maximum number of iseghal supported items
  * system - system information
  * line - hardware interface or virtual line (0..15)
  * device - module (0..63), crate (1000, 2000..2015)
  * channel - high voltage channels (0..63)
*/
// default: 5 can line max , 21 devices max, 10 channels max
#define NITEMS 1050
#define READ_BUF_LEN   40
#define WRITE_BUF_LEN   8
#define ITEM_TYPE_LEN   4
#define ITEM_FQN_LEN    34
#define ITEM_ADDR_LEN   6

#define DEFAULT_PORT_RECONNECT 		30 // 10 min.
#define POLLER_AUTOCNNECT_SLEEP 	30 // second.
#define ISEG_ITEM_VALUE_MAX_LEN 	200

typedef enum {

  UINT32DIGITALTYPE = 0,
  FLOAT64TYPE 			= 1,
  INT32TYPE 				= 2,
  OCTECTTYPE 				= 3
} drvIsegHalPoller_uflags_t;

typedef struct {

  drvIsegHalPoller_uflags_t uflags;
  void *intrHandle;
	char prevItemVal[READ_BUF_LEN];
} intrUser_data_t;

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

static void  drvIsegHalPollerThreadCallackBack(asynUser *pasynUser);
/* asynPortDriver for ISEG  iCS based HV systems (CC24 and iCSmini) using iseghal service library
  *
  * This asynPortDriver is the device support for ISEG Spezialelektronik GmbH iCS HV system starting at version 2.8.0 using the isegHALService
  * to provide a multiple devices access to modules in an ECH or MPOD crate in combination with CC24 (ECHxxx & wiener MPOD crate access)
  * or iCSmini controller ( ECHxx crate with MMS & MMC slots). These compatible HV-modules (EBS, EDS, EHS, ESS,NHS, NHQ, HPS, FPS, SHQ ) are also supported.
  *
*/
class drvAsynIsegHalService : public asynPortDriver {
  public:
    drvAsynIsegHalService( const char *portName, const char *interface, const char *icsCtrtype, epicsInt16 autoConnect );
    // These are the methods that we override from asynPortDriver
    virtual asynStatus readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask );
    virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );
    virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value );
    virtual asynStatus readFloat64( asynUser *pasynUser, epicsFloat64 *value );
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    virtual asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize );
    virtual asynStatus connect(asynUser *pasynUser);
    virtual asynStatus disconnect(asynUser *pasynUser);

    int devConnect( std::string const& name, std::string const& interface );
    int devConnected( std::string const& name );
    int devDisconnect( std::string const& name );
    int isValidIsegHalItem (const char *item);
		asynStatus getIsegHalItem (asynUser *isegHalUser, IsegItem *item);

    asynUser *exitUser_;

  protected:

  private:
    char  *session_;
    char  *interface_;
    char  *deviceModel_;
    int   itemReason_;
		bool  devInitOk_;
		bool  reconStatus_;
		int   reconAttempt_;
    std::vector< std::string > openedSessions;
    std::map<epicsUInt32, std::string> isegHalItemsLookup;
};


/* @brief   thread monitoring set values from isegHAL
*
* This thread checks regulary the value of all set-parameters
* and updates the corresponding records if the values
* within the EPICS db and the isegHAL are out of sync.
*/
class drvIsegHalPollerThread: public epicsThreadRunable {
  public:
    drvIsegHalPollerThread(drvAsynIsegHalService *portD);
    virtual ~drvIsegHalPollerThread();
    virtual void run();
    epicsThread thread;

    inline void changeInterval( double val ) { _pause = val; }
		inline void changeqRequestInterval( double val ) { _qRequestInterval = val; }
    inline double getInterval(){ return _pause; }

    inline void setDbgLvl( int dbglvl ) { _debug = dbglvl; }
    inline void disable() { _run = false; }
    inline void enable() { _run = true; }

    asynStatus pLock();
    asynStatus pUnlock();

  private:

    bool _run;
    double _pause;
    double _qRequestInterval;
		epicsMutexId _pollMutexId;
    unsigned _debug;
    std::list<asynUser *> _pasynIntrUser;
    std::list<intrUser_data_t *> _intrUser_data_gbg;
};

#endif
