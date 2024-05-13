/* drvIsegHalPollerThread.cpp
 *
 * Poller thread for drvAsynIsegHalService Port Driver.
 *
 * Yann Stephen Mandza
 * May 01, 2024
*/

// ANSI C/C++ includes
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

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
#include "drvAsynIsegHalService.h"

/* isegHAL includes */
#include <isegclientapi.h>

typedef std::list<asynUser *>::iterator intrUserItr;
typedef std::list<intrUser_data_t *>::iterator intrUserDataItr;

drvAsynIsegHalService *drvAsynIsegHalService_= NULL;

/* Called by epicsAtExit to shutdown iseghal session */
static void drvIsegHalPollerThreadShutdown( void* pdrv)
{
  drvIsegHalPollerThread *pPvt = (drvIsegHalPollerThread *) pdrv;
	pPvt->pLock();
  pPvt->_drvIsegHalPollerThreadExiting = true;
	pPvt->pUnlock();
  printf("\033[0;36m%s:%s Shutting down...\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__ );
  delete pPvt;
}

/*
  * @brief   Interrupt Users callback for Clean device access in port driver
  *          each registered Interrupt as a duplicate asynuser  thats registered this callback for device
	*          access within the port thread: this feature is used because the device handle does not allow
	*          access to the Qt SSL socket outside the creating thread, aka. Asyn Port Thread.
	*          Thus, queued requests from their duplicates use this method to cleanly access the device
  *
  * @param  [in]  pasynUser  pasynUser structure that encodes the reason and address.
	*
*/
static void  drvIsegHalPollerThreadCallackBack(asynUser *pasynUser)
{
  static const char *functionName="drvIsegHalPollerThreadCallackBack";

  intrUser_data_t *intrUser = (intrUser_data_t *)pasynUser->userData;
  drvIsegHalPoller_uflags_t ifaceType = (drvIsegHalPoller_uflags_t)intrUser->uflags;
	char *prevItemVal = (char *)intrUser->prevItemVal;
	epicsUInt16 mask;
	// maybe makes this part of asynuser data?
  IsegItem item = EmptyIsegItem;
	asynStatus status = asynSuccess;

	if(drvAsynIsegHalService_)
		drvAsynIsegHalService_->lock();
		status = drvAsynIsegHalService_->getIsegHalItem(pasynUser, &item);
		drvAsynIsegHalService_->unlock();

	if(status == asynSuccess) {
		pasynUser->alarmStatus 		= 0;
		pasynUser->alarmSeverity	= 0;
	}

	switch(ifaceType) {

		case FLOAT64TYPE:
			{
				asynFloat64Interrupt *pFloat64 = (asynFloat64Interrupt*)intrUser->intrHandle;
				if ( strcmp( item.value, prevItemVal ) != 0 ) {
					epicsFloat64 float64Value;
					float64Value = (epicsFloat64)strtod (item.value, NULL);
					if(status != asynSuccess) float64Value = NAN;
					//printf("\033[0;36m%s:(%s) item value %s converted %lf\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, item.value,float64Value);
					pFloat64->callback(pFloat64->userPvt, pasynUser, float64Value);
				}
				break;
			}

		case UINT32DIGITALTYPE:
			{
				asynUInt32DigitalInterrupt *pUInt32D = (asynUInt32DigitalInterrupt*)intrUser->intrHandle;
				if ( strcmp( item.value, prevItemVal ) != 0 ) {
					epicsUInt32 uInt32Value;
					uInt32Value =  (epicsUInt32)atoi(item.value) ;
					mask = pUInt32D->mask;
					if (mask != 0 ) uInt32Value &= mask;
					if(status != asynSuccess) uInt32Value = NAN;
					//printf("\033[0;36m%s : (%s) item value %s converted %d\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, item.value, uInt32Value );
					pUInt32D->callback(pUInt32D->userPvt, pasynUser, uInt32Value);
				}
				break;
			}

		case INT32TYPE:
			{
				asynInt32Interrupt *pInt32 = (asynInt32Interrupt*)intrUser->intrHandle;
				if ( strcmp( item.value, prevItemVal ) != 0 ) {
					epicsInt32 int32Value;
					int32Value = (epicsInt32)atoi(item.value);
					if(status != asynSuccess) int32Value = NAN;
					//printf("\033[0;36m%s:(%s) item value %s converted %d\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__, item.value,int32Value );
					pInt32->callback(pInt32->userPvt, pasynUser, int32Value);
				}
				break;
			}

		default:
				asynPrint(pasynUser, ASYN_TRACE_ERROR,
          "%s Undefined Interface\n",functionName);
				break;
	}
	// store intUser prev value.
	strcpy(prevItemVal, item.value);
}

/*
 * @brief  C'tor of drvIsegHalPollerThread
*/
drvIsegHalPollerThread::drvIsegHalPollerThread(drvAsynIsegHalService *portD)
  : thread( *this, "drvIsegHalPollerThread", epicsThreadGetStackSize( epicsThreadStackSmall ), 50 ),
    _run( true ),
    _pause(2.),
    _debug(0),
		_qRequestInterval(0.005)
{
  if(!portD) return;

  _pollMutexId = epicsMutexCreate();
  if (!_pollMutexId) {
		  printf( "\033[0;33m%s : ( %s ): ERROR: epicsMutexCreate failure\n\033[0m", epicsThreadGetNameSelf( ), __FUNCTION__ );
			return;
  }

  drvAsynIsegHalService_ = portD;
  _pasynIntrUser.clear();
  epicsAtExit(drvIsegHalPollerThreadShutdown, (void*)this);
  printf("\033[0;36m%s:(%s) : Poller Initialization completed \n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__ );
}
asynStatus drvIsegHalPollerThread::pLock() {
    int status;
    status = epicsMutexLock(_pollMutexId);
    if (status) return asynError;
    else return asynSuccess;
}
asynStatus drvIsegHalPollerThread::pUnlock() {
	epicsMutexUnlock(_pollMutexId);
	return asynSuccess;
}

/*
	* @brief  D'tor of drvIsegHalPollerThreaddbl
*/
drvIsegHalPollerThread::~drvIsegHalPollerThread()
{

	// give some time for already queued requests to complete.
	epicsThreadSleep(5);

  // Clean up space used for intr users
  intrUserItr _intrUserItr = _pasynIntrUser.begin();
  for( ; _intrUserItr != _pasynIntrUser.end(); ++_intrUserItr ) {
    pasynManager->freeAsynUser((asynUser *)(*_intrUserItr));
  }
  // Clean up intr data allocated storage
  intrUserDataItr _intrUserDataItr = _intrUser_data_gbg.begin();
  for( ; _intrUserDataItr != _intrUser_data_gbg.end(); ++_intrUserDataItr ) {
    free((intrUser_data_t *)(*_intrUserDataItr));
  }

 _pasynIntrUser.clear();
 _intrUser_data_gbg.clear();
 //drvAsynIsegHalService_ = NULL;

 printf("\033[0;36m%s:%s Poller Cleaning completed!\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__ );

}

/*
  * @brief   Poller thread runtime method
  *
	* When this poller object is instantiated in the drvAsynIsegHalService instance after the IOC gets in running state
	* We build a local list of duplicate interrupt users by registering a special callback for each.
	* After the thread is started, we queue each request to the port thread to cleanly access the device.
	*
*/
void drvIsegHalPollerThread::run()
{
  ELLLIST *pclientList;
  interruptNode *pnode;
  asynStatus status;
  asynUser *pasynUser;

  // See if there are any asynUInt32Digital callbacks registered to be called
  pasynManager->interruptStart(drvAsynIsegHalService_->getAsynStdIface().uInt32DigitalInterruptPvt, &pclientList);
  pnode = (interruptNode *)ellFirst(pclientList);
  asynUInt32DigitalInterrupt *pUInt32D;
  while (pnode) {
    pUInt32D = (asynUInt32DigitalInterrupt *)pnode->drvPvt;
    pasynUser = pasynManager->duplicateAsynUser(pUInt32D->pasynUser, drvIsegHalPollerThreadCallackBack,0);
    intrUser_data_t *_intrUser = (intrUser_data_t *)mallocMustSucceed(sizeof(intrUser_data_t), "Failed to alloc UInt32D Intr User data");

    pasynUser->reason = pUInt32D->pasynUser->reason;
    _intrUser->uflags = UINT32DIGITALTYPE;
    _intrUser->intrHandle = (void*)pUInt32D;
		_intrUser->prevItemVal[READ_BUF_LEN] = 0;
    pasynUser->userData = (void*)_intrUser;
    _pasynIntrUser.push_back(pasynUser);
    _intrUser_data_gbg.push_back(_intrUser);

    // to be sure that each asynUser is only added once
    _pasynIntrUser.sort();
    _pasynIntrUser.unique();
    _intrUser_data_gbg.sort();
    _intrUser_data_gbg.unique();
    pnode = (interruptNode *)ellNext(&pnode->node);
  }
  pasynManager->interruptEnd(drvAsynIsegHalService_->getAsynStdIface().uInt32DigitalInterruptPvt);

  // See if there are any asynInt32 callbacks registered to be called.
  pasynManager->interruptStart(drvAsynIsegHalService_->getAsynStdIface().int32InterruptPvt, &pclientList);
  pnode = (interruptNode *)ellFirst(pclientList);
  asynInt32Interrupt *pInt32;
  while (pnode) {
    pInt32 = (asynInt32Interrupt *)pnode->drvPvt;
    pasynUser = pasynManager->duplicateAsynUser(pInt32->pasynUser, drvIsegHalPollerThreadCallackBack,0);
    intrUser_data_t *_intrUser = (intrUser_data_t *)mallocMustSucceed(sizeof(intrUser_data_t), "Failed to alloc UInt32D Intr User data");

    pasynUser->reason = pInt32->pasynUser->reason;
    _intrUser->uflags = INT32TYPE;
    _intrUser->intrHandle = (void*)pInt32;
		_intrUser->prevItemVal[READ_BUF_LEN] = 0;
    pasynUser->userData = (void*)_intrUser;
    _pasynIntrUser.push_back(pasynUser);
    _intrUser_data_gbg.push_back(_intrUser);

    // to be sure that each asynUser is only added once
    _pasynIntrUser.sort();
    _pasynIntrUser.unique();
    _intrUser_data_gbg.sort();
    _intrUser_data_gbg.unique();
    pnode = (interruptNode *)ellNext(&pnode->node);
  }
  pasynManager->interruptEnd(drvAsynIsegHalService_->getAsynStdIface().int32InterruptPvt);

  // See if there are any asynFloat64 callbacks registered to be called.
  pasynManager->interruptStart(drvAsynIsegHalService_->getAsynStdIface().float64InterruptPvt, &pclientList);
  pnode = (interruptNode *)ellFirst(pclientList);
  asynFloat64Interrupt *pFloat64;

  while (pnode) {
    pFloat64 = (asynFloat64Interrupt *)pnode->drvPvt;
    pasynUser = pasynManager->duplicateAsynUser(pFloat64->pasynUser, drvIsegHalPollerThreadCallackBack,0);
    intrUser_data_t *_intrUser = (intrUser_data_t *)mallocMustSucceed(sizeof(intrUser_data_t), "Failed to alloc UFloat64D Intr User data");

    pasynUser->reason = pFloat64->pasynUser->reason;
    _intrUser->uflags = FLOAT64TYPE;
    _intrUser->intrHandle = (void*)pFloat64;
		_intrUser->prevItemVal[READ_BUF_LEN] = 0;
    pasynUser->userData = (void*)_intrUser;
    _pasynIntrUser.push_back(pasynUser);
    _intrUser_data_gbg.push_back(_intrUser);

    // to be sure that each asynUser is only added once
    _pasynIntrUser.sort();
    _pasynIntrUser.unique();
    _intrUser_data_gbg.sort();
    _intrUser_data_gbg.unique();
    pnode = (interruptNode *)ellNext(&pnode->node);
  }
  pasynManager->interruptEnd(drvAsynIsegHalService_->getAsynStdIface().float64InterruptPvt);

  while(true) {

    if( _drvIsegHalPollerThreadExiting ) {
      printf("\033[0;36m%s:%s Exiting poller thread...\n\033[0m", epicsThreadGetNameSelf(), __FUNCTION__ );
      break;
    }
		pLock();
    if( _pause > 0. ) this->thread.sleep( _pause );
    if( !_run ) continue;
		pUnlock();

    intrUserItr _intrUserItr = _pasynIntrUser.begin();
		int _yesNo = 0;
    for( ; _intrUserItr != _pasynIntrUser.end(); ++_intrUserItr ) {

			if(pasynManager->isConnected((asynUser *)(*_intrUserItr), &_yesNo) != asynSuccess) continue;

			if(_yesNo) { // may not be needed perhaps the queueRequest 'fail not connected' is good debug message?

				status = pasynManager->queueRequest((asynUser *)(*_intrUserItr), asynQueuePriorityMedium, 0);
				if (status != asynSuccess) {

					asynPrint((asynUser *)(*_intrUserItr), ASYN_TRACE_ERROR,"drvIsegHalPollerThread::run  queueRequest Error "
								"status=%d, error=%s\n",status, ((asynUser *)(*_intrUserItr))->errorMessage);
				}
			}

			epicsThreadSleep(_qRequestInterval);
    }
  }
}
