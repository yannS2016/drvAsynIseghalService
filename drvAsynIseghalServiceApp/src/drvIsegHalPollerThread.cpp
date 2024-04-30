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
#include <initHooks.h>
#include <dbCommon.h>

// ASYN includes
#include <asynDriver.h>
#include <asynUInt32Digital.h>
#include <asynUInt32DigitalSyncIO.h>
#include <asynStandardInterfaces.h>

// local includes
#include "drvIsegHalPollerThread.h"
#include "drvAsynIseghalService.h"
/* isegHAL includes */
#include <isegclientapi.h>
drvAsynIseghalService *drvAsynIseghalService_= NULL;

//------------------------------------------------------------------------------
//! @brief       C'tor of drvIsegHalPollerThread
//------------------------------------------------------------------------------
drvIsegHalPollerThread::drvIsegHalPollerThread(drvAsynIseghalService *portD)
  : thread( *this, "drvIsegHalPollerThread", epicsThreadGetStackSize( epicsThreadStackSmall ), 50 ),
    _run( true ),
    _pause(5.),
    _debug(0)
{
	std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread "<<"\033[0m" << std::endl;
	pasynUserIntr.clear();
	if(!portD) return;

	drvAsynIseghalService_ = portD;

}

//------------------------------------------------------------------------------
//! @brief       D'tor of drvIsegHalPollerThreaddbl
//------------------------------------------------------------------------------
drvIsegHalPollerThread::~drvIsegHalPollerThread() {
 pasynUserIntr.clear();
 drvAsynIseghalService_ = NULL;
}

void drvIsegHalPollerThread::run() {
		std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: Outside" << "\033[0m" << std::endl;
    ELLLIST *pclientList;
    interruptNode *pnode;
		asynStatus status;
    asynUser *pasynUser;

    epicsUInt32 uInt32Value;
    epicsInt32 int32Value;
    epicsInt64 int64Value;
    epicsFloat64 float64Value;
		epicsInt16 intrClient = 0;
    // See if there are any asynUInt32Digital callbacks registered to be called
    pasynManager->interruptStart(drvAsynIseghalService_->getAsynStdIface().uInt32DigitalInterruptPvt, &pclientList);
    pnode = (interruptNode *)ellFirst(pclientList);
    asynUInt32DigitalInterrupt *pUInt32D;
    while (pnode) {
      pUInt32D = (asynUInt32DigitalInterrupt *)pnode->drvPvt;
			pasynUserIntr.push_back( pasynManager->duplicateAsynUser(pUInt32D->pasynUser, drvIsegHalPollerThreadCallackBack,0) );
			// to be sure that each record is only added once
			pasynUserIntr.sort();
			pasynUserIntr.unique();
			intrClient++;
			std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: intr client:" << intrClient << "\033[0m" << std::endl;
      pnode = (interruptNode *)ellNext(&pnode->node);
		}
    pasynManager->interruptEnd(drvAsynIseghalService_->getAsynStdIface().uInt32DigitalInterruptPvt);

    // See if there are any asynInt32 callbacks registered to be called.
    pasynManager->interruptStart(drvAsynIseghalService_->getAsynStdIface().int32InterruptPvt, &pclientList);
    pnode = (interruptNode *)ellFirst(pclientList);
    while (pnode) {
      asynInt32Interrupt *pInt32;
      pInt32 = (asynInt32Interrupt *)pnode->drvPvt;
			pasynUserIntr.push_back( pasynManager->duplicateAsynUser(pInt32->pasynUser, drvIsegHalPollerThreadCallackBack,0) );
			// to be sure that each record is only added once
			pasynUserIntr.sort();
			pasynUserIntr.unique();
			intrClient++;
			std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: intr client:" << intrClient << "\033[0m" << std::endl;
      pnode = (interruptNode *)ellNext(&pnode->node);
    }
    pasynManager->interruptEnd(drvAsynIseghalService_->getAsynStdIface().int32InterruptPvt);

    // See if there are any asynFloat64 callbacks registered to be called.
    pasynManager->interruptStart(drvAsynIseghalService_->getAsynStdIface().float64InterruptPvt, &pclientList);
    pnode = (interruptNode *)ellFirst(pclientList);
    while (pnode) {
      asynFloat64Interrupt *pFloat64;
      pFloat64 = (asynFloat64Interrupt *)pnode->drvPvt;
			pasynUserIntr.push_back( pasynManager->duplicateAsynUser(pFloat64->pasynUser, drvIsegHalPollerThreadCallackBack,0) );
			// to be sure that each record is only added once
			pasynUserIntr.sort();
			pasynUserIntr.unique();
			intrClient++;
			std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: intr client:" << intrClient << "\033[0m" << std::endl;
      pnode = (interruptNode *)ellNext(&pnode->node);
    }
    pasynManager->interruptEnd(drvAsynIseghalService_->getAsynStdIface().float64InterruptPvt);





		while(true) {
			std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: inside loop" << "\033[0m" << std::endl;
			std::list<asynUser *>::iterator it = pasynUserIntr.begin();
			for( ; it != pasynUserIntr.end(); ++it ) {
				asynStatus status = pasynManager->queueRequest((asynUser *)(*it), (asynQueuePriority)0, 0);
				if (status != asynSuccess) {
					asynPrint((asynUser *)(*it), ASYN_TRACE_ERROR,"drvAsynIseghalService::iseghalPollerTask  ERROR calling queueRequest\n"
													"    status=%d, error=%s\n",status, (asynUser *)(*it)->errorMessage);

				}
			}
					epicsThreadSleep(2);
		}
}

void  drvIsegHalPollerThreadCallackBack(asynUser *pasynUser) {
	dbCommon *pr = (dbCommon *)pasynUser->userPvt;
  asynInterface *pasynInterface;
  asynUInt32Digital *pasynUInt32Digital;
	void *uInt32DigitalPvt;
/* 	asynUInt32DigitalInterrupt *pUInt32D = (asynUInt32DigitalInterrupt*)pasynUser->userData;
	//epicsUInt32 value = 6;
	epicsUInt32 uInt32Value = rand() % 50;
	static const char *functionName="pCallackOut";

  pasynInterface = pasynManager->findInterface(pasynUser, asynUInt32DigitalType, 1);
	if(!pasynInterface) {
			asynPrint(pasynUser, ASYN_TRACE_ERROR,
					"%s findInterface failed for asynUInt32\n",
					functionName);
	}
	pasynUInt32Digital = (asynUInt32Digital *)pasynInterface->pinterface;
	uInt32DigitalPvt = pasynInterface->drvPvt;
	// this will call the port readUint32Digital method but from this thread

	//pasynUInt32Digital->read(uInt32DigitalPvt, pasynUser, &value, 0xf);
	pUInt32D->callback(pUInt32D->userPvt, pasynUser, uInt32Value); */
	std::cout << "\033[0;33m " << "( " << __FUNCTION__ << " ) from " << epicsThreadGetNameSelf() << " thread: " << "pasynUser Reason: "<< pasynUser->reason << "\033[0m" << std::endl;

}
