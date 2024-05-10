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

#ifndef __DRVISEGHALPOLLERTHREAD_H__
#define __DRVISEGHALPOLLERTHREAD_H__

#include <map>
#include <vector>
#include <list>
#include <epicsThread.h>

typedef enum {
  UINT32DIGITALTYPE = 0,
  FLOAT64TYPE = 1,
  INT32TYPE = 2,
  OCTECTTYPE = 3
} drvIsegHalPoller_uflags_t;

typedef struct {
  drvIsegHalPoller_uflags_t uflags;
  void *intrHandle;
} intrUser_data_t;


class drvAsynIsegHalService;

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

    inline void changeIntervall( double val ) { _pause = val; }
    inline double getIntervall(){ return _pause; }

    inline void setDbgLvl( int dbglvl ) { _debug = dbglvl; }
    inline void disable() { _run = false; }
    inline void enable() { _run = true; }
    bool _drvIsegHalPollerThreadExiting;

  private:
    bool _run;
    double _pause;
    unsigned _debug;
    std::list<asynUser *> _pasynIntrUser;
    std::list<intrUser_data_t *> _intrUser_data_gbg;
};

#endif
