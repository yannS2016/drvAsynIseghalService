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

#ifndef __DRVASYNISEGHALSERVICE_H__
#define __DRVASYNISEGHALSERVICE_H__

#include <map>
#include <vector>
#include <epicsThread.h>
#include <asynPortDriver.h>
#include <drvIsegHalPollerThread.h>

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
    virtual asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize );
    virtual asynStatus connect(asynUser *pasynUser);
    virtual asynStatus disconnect(asynUser *pasynUser);

		asynStatus getIsegHalItem (asynUser *isegHalUser, IsegItem *item);

    int devConnect( std::string const& name, std::string const& interface );
    int devConnected( std::string const& name );
    int devDisconnect( std::string const& name );
    int hasIsegHalItem (const char *item);
    char *getSessionName ();
    asynStandardInterfaces getAsynStdIface();

		void conManager(std::string const& name);
    void iseghalPollerTask(void);
    bool iseghalExiting_;
    asynUser *exitUser_;

  protected:
    // Values used for pasynUser->reason, and indexes into the parameter library.
    // system items are not CAN address dependant.
    int P_SysStatus;      //index for Parameter "SystemStatus"
    int P_SysCrtNum;      //index for Parameter "SystemCrateNumber"
    int P_SysCrtList;     //index for Parameter "SystemCrateList"
    int P_SysModNum;      //index for Parameter "SystemModuleNumber"
    int P_SysCCtr;        //index for Parameter "CycleCounter"
    int P_SysLogL;        //index for Parameter "SystemLogLevel"
    int P_SysLogPath;     //index for Parameter "LogPath"
    int P_SysLinMode;     //index for Parameter "LiveInsertionMode"
    int P_SysSConfig;     //index for Parameter "SaveConfiguration"
    int P_SysEthName;     //index for Parameter "EthName"
    int P_SysEthAddr;     //index for Parameter "EthAddress"
    int P_SysSVers;       //index for Parameter "ServerVersion"
    int P_SysNetTout;     //index for Parameter "NetworkTimeout"
    int P_SysSSname;      //index for Parameter "SessionName"

    int iseghalItems[NITEMS];

  private:

    char  *session_;
    char  *interface_;
    char  *deviceModel_;
    int   itemReason_;
		bool  devInitOk_;
		bool  conMan_;
		int   iseghalReconAttempt_;
    std::vector< std::string > openedSessions;
    std::map<epicsUInt32, std::string> isegHalItemsLookup;
};


#endif
