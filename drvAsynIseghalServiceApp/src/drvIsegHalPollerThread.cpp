/* drvIsegHalPollerThread.cpp
 *
 * Poller thread for drvAsynIseghalService Port Driver.
 *
 * Yann Stephen Mandza
 * May 01, 2024
*/

// ANSI C/C++ includes
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// EPICS includes
#include <epicsEvent.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTypes.h>
#include <cantProceed.h>
#include <epicsExit.h>
// ASYN includes
#include <asynDriver.h>
#include <asynStandardInterfaces.h>

// local includes
#include "drvIsegHalPollerThread.h"
#include "drvAsynIseghalService.h"

/* isegHAL includes */
#include <isegclientapi.h>

typedef std::list<asynUser *>::iterator intrUserItr;
typedef std::list<intrUser_data_t *>::iterator intrUserDataItr;

drvAsynIseghalService *drvAsynIseghalService_= NULL;

/* Called by epicsAtExit to shutdown iseghal session */
static void drvIsegHalPollerThreadShutdown( void* pdrv)
{
  asynStatus status;
  drvIsegHalPollerThread *pPvt = (drvIsegHalPollerThread *) pdrv;
  pPvt->_drvIsegHalPollerThreadExiting = true;
  printf("\033[0;33m%s : %s : Shutting down...\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__ );
  delete pPvt;
}

static void  drvIsegHalPollerThreadCallackBack(asynUser *pasynUser)
{
  static const char *functionName="drvIsegHalPollerThreadCallackBack";

  intrUser_data_t *intrUser = (intrUser_data_t *)pasynUser->userData;
  drvIsegHalPoller_uflags_t ifaceType = (drvIsegHalPoller_uflags_t)intrUser->uflags;
  printf("\033[0;33m%s : %s : iface type: %d\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, (epicsInt16)ifaceType );

  if(ifaceType == UINT32DIGITALTYPE) {
    asynUInt32DigitalInterrupt *pUInt32D = (asynUInt32DigitalInterrupt*)intrUser->intrHandle;
    epicsUInt32 uInt32Value = rand() % 50;
    pUInt32D->callback(pUInt32D->userPvt, pasynUser, uInt32Value);
  }
  else if (ifaceType == FLOAT64TYPE)
  {
    asynFloat64Interrupt *pFloat64 = (asynFloat64Interrupt*)intrUser->intrHandle;
    epicsFloat64 float64Value = (epicsFloat64)((rand() % 50)*0.5);
    pFloat64->callback(pFloat64->userPvt, pasynUser, float64Value);
  }
  else if (ifaceType == INT32TYPE)
  {
    asynInt32Interrupt *pInt32 = (asynInt32Interrupt*)intrUser->intrHandle;
    epicsInt32 int32Value = rand() % 50;
    pInt32->callback(pInt32->userPvt, pasynUser, int32Value);
  } else {
      asynPrint(pasynUser, ASYN_TRACE_ERROR,
          "%s Undefined Interface\n",functionName);
      return;
  }
}

/*
 * @brief  C'tor of drvIsegHalPollerThread
*/
drvIsegHalPollerThread::drvIsegHalPollerThread(drvAsynIseghalService *portD)
  : thread( *this, "drvIsegHalPollerThread", epicsThreadGetStackSize( epicsThreadStackSmall ), 50 ),
    _run( true ),
    _pause(2.),
    _debug(0)
{
  if(!portD) return;
  drvAsynIseghalService_ = portD;
  _pasynIntrUser.clear();
  epicsAtExit(drvIsegHalPollerThreadShutdown, (void*)this);
  printf("\033[0;33m%s : %s : Init completed...\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__ );
}

//------------------------------------------------------------------------------
//! @brief       D'tor of drvIsegHalPollerThreaddbl
//------------------------------------------------------------------------------
drvIsegHalPollerThread::~drvIsegHalPollerThread()
{

  // Clean space used for intr users
  intrUserItr _intrUserItr = _pasynIntrUser.begin();
  for( ; _intrUserItr != _pasynIntrUser.end(); ++_intrUserItr ) {
    pasynManager->freeAsynUser((asynUser *)(*_intrUserItr));
  }
  // Clean intr data allocated storage
  intrUserDataItr _intrUserDataItr = _intrUser_data_gbg.begin();
  for( ; _intrUserDataItr != _intrUser_data_gbg.end(); ++_intrUserDataItr ) {
    free((intrUser_data_t *)(*_intrUserDataItr));
  }

 _pasynIntrUser.clear();
 _intrUser_data_gbg.clear();
 drvAsynIseghalService_ = NULL;

 printf("\033[0;33m%s : %s : Cleaning up...\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__ );

}

void drvIsegHalPollerThread::run()
{

  ELLLIST *pclientList;
  interruptNode *pnode;
  asynStatus status;
  asynUser *pasynUser;
  epicsInt16 intrClient =

  // See if there are any asynUInt32Digital callbacks registered to be called
  pasynManager->interruptStart(drvAsynIseghalService_->getAsynStdIface().uInt32DigitalInterruptPvt, &pclientList);
  pnode = (interruptNode *)ellFirst(pclientList);
  asynUInt32DigitalInterrupt *pUInt32D;
  while (pnode) {
    pUInt32D = (asynUInt32DigitalInterrupt *)pnode->drvPvt;
    pasynUser = pasynManager->duplicateAsynUser(pUInt32D->pasynUser, drvIsegHalPollerThreadCallackBack,0);
    intrUser_data_t *_intrUser = (intrUser_data_t *)mallocMustSucceed(sizeof(intrUser_data_t), "Failed to alloc UInt32D Intr User data");
    pasynUser->reason = pUInt32D->pasynUser->reason;
    _intrUser->uflags = UINT32DIGITALTYPE;
    _intrUser->intrHandle = (void*)pUInt32D;
    pasynUser->userData = (void*)_intrUser;
    _pasynIntrUser.push_back(pasynUser);
    _intrUser_data_gbg.push_back(_intrUser);

    // to be sure that each asynUser is only added once
    _pasynIntrUser.sort();
    _pasynIntrUser.unique();
    _intrUser_data_gbg.sort();
    _intrUser_data_gbg.unique();
    intrClient++;
    pnode = (interruptNode *)ellNext(&pnode->node);
  }
  pasynManager->interruptEnd(drvAsynIseghalService_->getAsynStdIface().uInt32DigitalInterruptPvt);

  // See if there are any asynInt32 callbacks registered to be called.
  pasynManager->interruptStart(drvAsynIseghalService_->getAsynStdIface().int32InterruptPvt, &pclientList);
  pnode = (interruptNode *)ellFirst(pclientList);
  asynInt32Interrupt *pInt32;
  while (pnode) {
    pInt32 = (asynInt32Interrupt *)pnode->drvPvt;
    pasynUser = pasynManager->duplicateAsynUser(pInt32->pasynUser, drvIsegHalPollerThreadCallackBack,0);
    intrUser_data_t *_intrUser = (intrUser_data_t *)mallocMustSucceed(sizeof(intrUser_data_t), "Failed to alloc UInt32D Intr User data");
    pasynUser->reason = pInt32->pasynUser->reason;
    _intrUser->uflags = INT32TYPE;
    _intrUser->intrHandle = (void*)pInt32;
    pasynUser->userData = (void*)_intrUser;
    _pasynIntrUser.push_back(pasynUser);
    _intrUser_data_gbg.push_back(_intrUser);

    // to be sure that each asynUser is only added once
    _pasynIntrUser.sort();
    _pasynIntrUser.unique();
    _intrUser_data_gbg.sort();
    _intrUser_data_gbg.unique();
    intrClient++;
    pnode = (interruptNode *)ellNext(&pnode->node);
  }
  pasynManager->interruptEnd(drvAsynIseghalService_->getAsynStdIface().int32InterruptPvt);

  // See if there are any asynFloat64 callbacks registered to be called.
  pasynManager->interruptStart(drvAsynIseghalService_->getAsynStdIface().float64InterruptPvt, &pclientList);
  pnode = (interruptNode *)ellFirst(pclientList);
  asynFloat64Interrupt *pFloat64;

  while (pnode) {
    pFloat64 = (asynFloat64Interrupt *)pnode->drvPvt;
    pasynUser = pasynManager->duplicateAsynUser(pFloat64->pasynUser, drvIsegHalPollerThreadCallackBack,0);
    intrUser_data_t *_intrUser = (intrUser_data_t *)mallocMustSucceed(sizeof(intrUser_data_t), "Failed to alloc UFloat64D Intr User data");
    pasynUser->reason = pFloat64->pasynUser->reason;
    _intrUser->uflags = FLOAT64TYPE;
    _intrUser->intrHandle = (void*)pFloat64;
    pasynUser->userData = (void*)_intrUser;
    _pasynIntrUser.push_back(pasynUser);
    _intrUser_data_gbg.push_back(_intrUser);
    // to be sure that each asynUser is only added once
    _pasynIntrUser.sort();
    _pasynIntrUser.unique();
    _intrUser_data_gbg.sort();
    _intrUser_data_gbg.unique();
    intrClient++;
    pnode = (interruptNode *)ellNext(&pnode->node);
  }
  pasynManager->interruptEnd(drvAsynIseghalService_->getAsynStdIface().float64InterruptPvt);

  while(true) {

    if( _drvIsegHalPollerThreadExiting ) {
      printf("\033[0;33m%s : %s : Exiting...\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__ );
      break;
    }

    if( _pause > 0. ) this->thread.sleep( _pause );

    if( !_run ) continue;

    intrUserItr _intrUserItr = _pasynIntrUser.begin();
    for( ; _intrUserItr != _pasynIntrUser.end(); ++_intrUserItr ) {
      asynStatus status = pasynManager->queueRequest((asynUser *)(*_intrUserItr), (asynQueuePriority)0, 0);
      if (status != asynSuccess) {
        asynPrint((asynUser *)(*_intrUserItr), ASYN_TRACE_ERROR,"drvAsynIseghalService::iseghalPollerTask  ERROR calling queueRequest\n"
              "status=%d, error=%s\n",status, ((asynUser *)(*_intrUserItr))->errorMessage);
      }
    }
  }
}
