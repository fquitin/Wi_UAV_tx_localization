/*! @file LinuxInteractive.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Barebones interactive UI for executing Onboard SDK commands.
 *  Calls functions from the new Linux example based on user input.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef THREAD_H
#define THREAD_H

#include <DJI_Type.h>
#include <DJICommonType.h>
#include <DJI_API.h>
#include <DJI_Flight.h>


//Local Mission Planning Suite Headers

/*! Poll at 10Hz, waiting for commands from the user.
    The spin runs indefinitely until the user sends the exit command.
!*/
void test(int a);

#endif // THREAD_H
