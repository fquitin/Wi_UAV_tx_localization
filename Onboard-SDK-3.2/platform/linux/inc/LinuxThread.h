/*! @file LinuxThread.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Pthread-based threading for DJI Onboard SDK Linux example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef LINUXTHREAD_H
#define LINUXTHREAD_H

#include <string>
#include <cstring>
#include <DJI_HardDriver.h>
#include <DJI_Camera.h>
#include <DJI_Flight.h>
#include <DJI_Flight.h>
#include <DJI_HotPoint.h>
#include <DJI_Follow.h>
#include <DJI_WayPoint.h>
#include <DJI_VirtualRC.h>
#include <../../sample/Linux/Blocking/inc/thread.h>
#include <DJI_API.h>
#include <pthread.h>

#include <string>

using namespace std;
using namespace DJI::onboardSDK;

class LinuxThread
{
  public:
    LinuxThread();
 
    LinuxThread(CoreAPI *api,Flight *FLIGHT,float *pointerRadio,int *pointerNumber,string extension,char* callBack_char,int argc2,char* argv2[], int type);

    bool createThread();
    int stopThread();
    int UHD_SAFE_MAIN(int argc, char *argv[]);
  private:
    CoreAPI *api;
    Flight *FLIGHT;
    int type;
    pthread_t threadID;
    pthread_attr_t attr;

    static void *send_call(void *param);
    static void *read_call(void *param);
    static void *callback_call(void *param);
   static void *save_data(void *param);
    static void *key_call(void *param);
    static void *radio(void *param);

};

#endif // LINUXTHREAD_H
