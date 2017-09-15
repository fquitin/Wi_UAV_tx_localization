/*! @file main.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  New Linux App for DJI Onboard SDK. 
 *  Provides a number of convenient abstractions/wrappers around core API calls.
 *
 *  Calls are blocking; the calling thread will sleep until the
 *  call returns. Use Core API calls or another sample if you 
 *  absolutely need non-blocking calls. 
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

//System Headers
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <stdio.h> 
#include <math.h>


//DJI Linux Application Headers
#include "LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "LinuxSetup.h"
#include "LinuxCleanup.h"
#include "ReadUserConfig.h"
#include "LinuxMobile.h"
#include "LinuxFlight.h"
#include "LinuxInteractive.h"
#include "LinuxWaypoint.h"
#include "LinuxCamera.h"

//DJI OSDK Library Headers
#include <DJI_Follow.h>
#include <DJI_Flight.h>
#include <DJI_Version.h>
#include <DJI_WayPoint.h>

//Local Mission Planning Suite Headers
//#include <MissionplanHeaders.h>


#define RAD2DEG 57.2957795131
#define DEG2RAD 0.01745329252
using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;
CoreAPI* api;
Flight* flight;
int c;
float radioValue =6;
float *pointerRadio;
char *callBack_char;
char keyboard;
ofstream fileRadio;
int *pointerNumber;
int number=0;
int mode=0;
float distanceMove=0;

//! Main function for the Linux sample. Lightweight. Users can call their own API calls inside the Programmatic Mode else on Line 68. 
int main(int argc, char *argv[])
{
	  //! Instantiate a serialDevice, an API object, flight and waypoint objects and a read thread.
	  //! Instantiate a serialDevice, an API object, flight and waypoint objects and a read thread.
  LinuxSerialDevice* serialDevice = new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
  //cout << " valeur choisie deuxieme " << name<<endl;
  api = new CoreAPI(serialDevice);
  flight = new Flight(api);
  WayPoint* waypointObj = new WayPoint(api);
  Camera* camera = new Camera(api);
  float32_t radialSpeed;
	cout << " Which mode ? 2 for autonomous " << endl;
	cin >> mode;
	cout << " Yaw rate desired ? : " << endl;
	cin >> radialSpeed;
	cout << "indicate an autonomous distance " << endl;
	cin >> distanceMove;
	cout << " name desired " << endl;
	string name;
	cin >> name;
	
	string extension= name+".csv";
	pointerRadio = &radioValue;
	pointerNumber = &number;
	callBack_char = &keyboard;
	fileRadio.open(extension);
    
  LinuxThread read(api,flight,pointerRadio,pointerNumber,extension,callBack_char,argc,argv, 2);
  LinuxThread data(api,flight,pointerRadio,pointerNumber,extension,callBack_char,argc,argv, 4); // create thread for save data
  LinuxThread key(api,flight,pointerRadio,pointerNumber,extension,callBack_char,argc,argv, 5);
  LinuxThread radio(api,flight,pointerRadio,pointerNumber,extension,callBack_char,argc,argv,6);  // create thread to control radio
 
  //! Setup
  int setupStatus = setup(serialDevice, api, &read,&data,&key, &radio);
  if (setupStatus == -1)
  {
    std::cout << "This program will exit now. \n";
    return 0;
  }
  //! Set broadcast Freq Defaults
  unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
  usleep(500000);
  //! Mobile Mode
  if (argv[1] && !strcmp(argv[1],"-mobile"))
  {
    std::cout << "Listening to Mobile Commands\n";
    mobileCommandSpin(api, flight, waypointObj, camera, argv, argc);
  }
  //! Interactive Mode
  else if (argv[1] && !strcmp(argv[1], "-interactive"))
  {
    if (argc > 3)
      interactiveSpin(api, flight, waypointObj, camera, std::string(argv[2]), std::string(argv[3]));
    else if (argc == 3)
      interactiveSpin(api, flight, waypointObj, camera, std::string(argv[2]), std::string(""));
    else
      interactiveSpin(api, flight, waypointObj, camera, std::string(""), std::string(""));
  }
  //! Programmatic Mode - execute everything here without any interactivity. Useful for automation.
  else if (argv[1] && !strcmp(argv[1], "-programmatic"))
  {
    /*! Set a blocking timeout - this is the timeout for blocking API calls
        to wait for acknowledgements from the aircraft. Do not set to 0.
    !*/
    int blockingTimeout = 1; //Seconds
    
    //! Monitored Takeoff
    ackReturnData takeoffStatus = monitoredTakeoff(api, flight, blockingTimeout);

    //! Set broadcast Freq Defaults
    unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
    
    //! If the aircraft took off, continue to do flight control tasks 
    if (takeoffStatus.status == 1)
    {

      /*! This is where you can add your own flight functionality.
          Check out LinuxWaypoint and LinuxFlight for available APIs. 
          You can always execute direct Onboard SDK Library API calls
          through the api object available in this example.
      !*/ 
		PositionData curPosition;
		PositionData originPosition;
 		DJI::Vector3dData curLocalOffset; 
		DJI::EulerAngle curEuler;
	  std::cout << "\n avant le ecole\n";
    float speedX=0.5;
    float speedY=0.5;
    float speedZ=0.5;
    float movex=0;
    float movey=0;
    float time=0;
    float desiredAngle;
    int status21;
   	curPosition = api->getBroadcastData().pos;
		originPosition = curPosition;
    int currentValueX;
    int currentValueY;
    int currentValueZ;
    int currentValueYAW;
    int done=0;
    int tour=0;
    float oldRadioValue=0;
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
    char inputchar;
    ackReturnData takeControlStatus;
    ackReturnData releaseControlStatus;
    ackReturnData takeoff;
    ackReturnData land ;
    Angle yaw;
    Angle oldYaw;
    Angle old;
    yaw=flight->getYaw();
    int status6;
    int status7;
    int status8;
    int quarter=0;
    float posBeforeMove=0;
    
    fileRadio << "Number" <<","<<"yaw"  <<","<< "radioValue" <<","<< "Position x"  <<","<< "Position y"  <<","<< "Position z"<<"\n";
    yaw=flight->getYaw();
      yaw=RAD2DEG*yaw+180;
      oldYaw=yaw;
      old=yaw;
    //moveByPositionOffset(api,flight,0,0,1,0,1500,3,10);
    if ( mode==1) {
     while (tour==0){
    cout << endl;
  cout << "| Available commands:                                            |" << endl;
  cout << "| [a] Request Control                                            |" << endl;
  cout << "| [i] Information                                                |" << endl;
  cout << "| [g] Move by offset x=3                                         |" << endl;
  cout << "| [k] Move from angle yaw                                        |" << endl;
  cout << "| [r] Release Control                                            |" << endl;
  cout << "| [n] Tour                                                       |" << endl;
  cout << "| [p] Tour with angle and move                                   |" << endl;
  cout << "| [l] Line                                                       |" << endl;
  cout << "| [t] Takeoff                                                    |" << endl;
  cout << "| [x] Landing And Exit                                           |" << endl;
     cin >> inputchar;	

switch (inputchar)
    {
	  case 'a':
        takeControlStatus = takeControl(api);
        break;
       case 'i':
       yaw=flight->getYaw();
       yaw=RAD2DEG*yaw+180; 
       cout << " value of yaw angle  : " << yaw << endl;
       break; 	
      case 'n':
      takeControlStatus = takeControl(api);
      number++;
      yaw=flight->getYaw();
      yaw=RAD2DEG*yaw+180;
      oldYaw=yaw;
      old=yaw;
        while( quarter<5 ) {
		yaw=flight->getYaw();
		yaw=RAD2DEG*yaw+180;	
        int status1=moveWithVelocity(api,flight,0,0,0, radialSpeed ,1500, 3, 0.1);
        if ( oldYaw <90 && yaw >= 90){
			cout<< " First quarter complete with an angle of : "<< yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw <180 && yaw >=180){
			cout<< " Second quarter complete  with an angle of : "<< yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw <270 && yaw >= 270){
			cout<< " Third quarter complete  with an angle of : " << yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw>350  && yaw <=10){
			cout<< " Fourth quarter complete  with an angle of : " << yaw<< endl;
			quarter=quarter+1;
		}
		if(fmod(yaw,5)<1 && abs(yaw-old)>4 ){
			curPosition =flight -> getPosition();
            localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
			fileRadio <<number << "," << yaw  <<","<< radioValue <<","<< curLocalOffset.x  <<","<< curLocalOffset.y  <<","<< curLocalOffset.z<<"\n";
			old=yaw;
		}
        
        oldYaw=yaw;
		
	} quarter=0;
	releaseControlStatus = releaseControl(api);
        break;
        case 'p':
        oldRadioValue=radioValue;
         takeControlStatus = takeControl(api);
      number++;
      yaw=flight->getYaw();
      yaw=RAD2DEG*yaw+180;
      oldYaw=yaw;
      old=yaw;
        while( quarter<5 ) {
		yaw=flight->getYaw();
		yaw=RAD2DEG*yaw+180;	
        int status1=moveWithVelocity(api,flight,0,0,0, radialSpeed ,1500, 3, 0.1);
        if ( oldYaw <90 && yaw >= 90){
			cout<< " First quarter complete with an angle of : "<< yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw <180 && yaw >=180){
			cout<< " Second quarter complete  with an angle of : "<< yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw <270 && yaw >= 270){
			cout<< " Third quarter complete  with an angle of : " << yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw>350  && yaw <=10){
			cout<< " Fourth quarter complete  with an angle of : " << yaw<< endl;
			quarter=quarter+1;
		}
		if(fmod(yaw,5)<1 && abs(yaw-old)>4 ){
			curPosition =flight -> getPosition();
            localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
			fileRadio <<number << "," << yaw  <<","<< radioValue <<","<< curLocalOffset.x  <<","<< curLocalOffset.y  <<","<< curLocalOffset.z<<"\n";
			old=yaw;
		}
        
        oldYaw=yaw;
		
	} quarter=0;
	usleep(1000000); 
	if ( radioValue != oldRadioValue){
		desiredAngle=radioValue;
		desiredAngle=360-desiredAngle;
		cout << " radioValue in p : " << radioValue << endl;
		//desiredAngle=85;
		if (desiredAngle > 180){
		 desiredAngle=desiredAngle-360;
		}
		oldRadioValue=radioValue;
		status6=moveByPositionOffset(api,flight,0,0,0,desiredAngle,1500,3,10);
		usleep(1000000);
		cout<< " If the angle is right, indicate a distance " << endl;
		cin >> distanceMove;
		if (distanceMove !=0){
			cout << " valeur a bouger en w x :" << distanceMove*cos(DEG2RAD*desiredAngle) << " et la valeur en y " << distanceMove*sin(DEG2RAD*desiredAngle) << endl;
			status7=moveByPositionOffset(api,flight,distanceMove*cos(DEG2RAD*desiredAngle),distanceMove*sin(DEG2RAD*desiredAngle),0,desiredAngle,15000000,3,30);
			//status7=moveByPositionOffset(api,flight,distance*cos(DEG2RAD*desiredAngle),0,0,desiredAngle,1500,3,10);
			//status7=moveByPositionOffset(api,flight,0,distance*sin(DEG2RAD*desiredAngle),0,desiredAngle,1500,3,10);
		}
		
	}
	releaseControlStatus = releaseControl(api);
        break;
      case 'l':
      curPosition = api->getBroadcastData().pos;
  	curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
        while(curLocalOffset.x<3 ){
		int status1=moveWithVelocity(api,flight,speedX,0,0,0 ,1500, 3, 0.1);
		curPosition = api->getBroadcastData().pos;
  	curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
   }
        break;
      case 't':
         takeoff = monitoredTakeoff(api, flight, blockingTimeout);
        break;
        case 'r':
       releaseControlStatus = releaseControl(api);
       break;
     
      case 'x':
      takeControlStatus = takeControl(api);
         land = landing(api, flight,blockingTimeout);
        tour=1;
        break;
      
       case 'g':
          status21 = moveByPositionOffset(api,flight,0,3,0,0,1500000000,3,10);
        break;  
        case 'k':
		  float32_t yawDesired;
		  cout << " valeur de yaw voulue " << endl;
		  cin >> yawDesired;
          int status15 = moveByPositionOffset(api,flight,0,0,0,yawDesired,1500,3,10);
        break;  
        
        
      //! Do aircraft control - Waypoint example. 
     // wayPointMissionExample(api, waypointObj,blockingTimeout);
	}
	
} 
     } //! Land
     
     if (mode ==2){    // autonomous mode
		  cout << endl; 
     
  cout << "| WELCOME                                                        |" << endl;
  cout << "|    IN                                                          |" << endl;
  cout << "|      AUTONOMOUS                                                |" << endl;
  cout << "|           MODE                                                 |" << endl;
  cout << "|                                                                |" << endl;
  cout << "|      ENJOY !!!                                                 |" << endl;
  cout << "|                                                                |" << endl;
  yaw=flight->getYaw();
  moveByPositionOffset(api,flight,0,0,0.8,yaw*RAD2DEG,1500,3,10);
		  while (tour==0){
			  usleep(2000000); 

    if(number != 0){
		 moveByPositionOffset(api,flight,0,0,0.8,yaw*RAD2DEG-120,1500,3,10);
		 usleep(2000000); 

	}
        oldRadioValue=radioValue;
      number++;
      yaw=flight->getYaw();
      yaw=RAD2DEG*yaw+180;
      oldYaw=yaw;
      old=yaw;
        while( quarter<5 ) {
		yaw=flight->getYaw();
		yaw=RAD2DEG*yaw+180;	
        int status1=moveWithVelocity(api,flight,0,0,0, radialSpeed ,1500, 3, 0.1);
        if ( oldYaw <90 && yaw >= 90){
			cout<< " First quarter complete with an angle of : "<< yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw <180 && yaw >=180){
			cout<< " Second quarter complete  with an angle of : "<< yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw <270 && yaw >= 270){
			cout<< " Third quarter complete  with an angle of : " << yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw>350  && yaw <=10){
			cout<< " Fourth quarter complete  with an angle of : " << yaw<< endl;
			quarter=quarter+1;
		}
		if(fmod(yaw,5)<1 && abs(yaw-old)>4 ){
			curPosition =flight -> getPosition();
            localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
			fileRadio <<number << "," << yaw  <<","<< radioValue <<","<< curLocalOffset.x  <<","<< curLocalOffset.y  <<","<< curLocalOffset.z<<"\n";
			old=yaw;
		}
        
        oldYaw=yaw;
		
	} quarter=0;
	usleep(1000000); 
	if ( radioValue != oldRadioValue){
		desiredAngle=radioValue;
		desiredAngle=360-desiredAngle;
		cout << " radioValue in p : " << radioValue << endl;
		//desiredAngle=85;
		if (desiredAngle > 180){
		 desiredAngle=desiredAngle-360;
		}
		oldRadioValue=radioValue;
		status6=moveByPositionOffset(api,flight,0,0,0,desiredAngle,1500,3,10);
		usleep(1000000);
		
		if (distanceMove !=0){
			movex= distanceMove*cos(DEG2RAD*desiredAngle);
			movey = distanceMove*sin(DEG2RAD*desiredAngle);
			time=distanceMove/2;
			cout << " valeur a bouger en w x :" << movex << " et la valeur en y " << movey << endl;
			speedX = movex/time;
			speedY = movey/time;
			if( abs(movex) > abs(movey) ){
				  PositionData curPosition = api->getBroadcastData().pos;
					PositionData originPosition = curPosition;
  DJI::Vector3dData curLocalOffset; 
  
  DJI::EulerAngle curEuler = Flight::toEulerAngle(api->getBroadcastData().q);

  //Convert position offset from first position to local coordinates
  localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
  
  //See how much farther we have to go
  float32_t xOffsetRemaining = movex - curLocalOffset.x;
  float32_t yOffsetRemaining = movey - curLocalOffset.y;
   
				while( abs(xOffsetRemaining) > 2 ){
					int status3=moveWithVelocity(api,flight,speedX,speedY,0, 0 ,1500, 3, 0.1);
					curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
    curPosition = api->getBroadcastData().pos;
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
    
    //See how much farther we have to go
    xOffsetRemaining = movex - curLocalOffset.x;
    yOffsetRemaining = movey - curLocalOffset.y;
    if (keyboard =='r'){
	   releaseControlStatus = releaseControl(api);
	   while( keyboard != 'x'){
	   }
	   takeControlStatus = takeControl(api);
         land = landing(api, flight,blockingTimeout);
        tour=1;
}
				}	 
			//status7=moveByPositionOffset(api,flight,distance*cos(DEG2RAD*desiredAngle),distance*sin(DEG2RAD*desiredAngle),0,desiredAngle,15000000,3,30);
			//status7=moveByPositionOffset(api,flight,distance*cos(DEG2RAD*desiredAngle),0,0,desiredAngle,1500,3,10);
			//status7=moveByPositionOffset(api,flight,0,distance*sin(DEG2RAD*desiredAngle),0,desiredAngle,1500,3,10);
			
		}
		if( abs(movey) > abs(movex) ){
				  PositionData curPosition = api->getBroadcastData().pos;
					PositionData originPosition = curPosition;
  DJI::Vector3dData curLocalOffset; 
  
  DJI::EulerAngle curEuler = Flight::toEulerAngle(api->getBroadcastData().q);

  //Convert position offset from first position to local coordinates
  localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
  
  //See how much farther we have to go
  float32_t xOffsetRemaining = movex - curLocalOffset.x;
  float32_t yOffsetRemaining = movey - curLocalOffset.y;

				while( abs(yOffsetRemaining) > 2 ){
					int status3=moveWithVelocity(api,flight,speedX,speedY,0, 0 ,1500, 3, 0.1);
					curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
    curPosition = api->getBroadcastData().pos;
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
    
    //See how much farther we have to go
    xOffsetRemaining = movex - curLocalOffset.x;
    yOffsetRemaining = movey - curLocalOffset.y;
    if (keyboard =='r'){
	   releaseControlStatus = releaseControl(api);
	   while( keyboard != 'x'){
	   }
	   takeControlStatus = takeControl(api);
         land = landing(api, flight,blockingTimeout);
        tour=1;
}
				}	 
			//status7=moveByPositionOffset(api,flight,distance*cos(DEG2RAD*desiredAngle),distance*sin(DEG2RAD*desiredAngle),0,desiredAngle,15000000,3,30);
			//status7=moveByPositionOffset(api,flight,distance*cos(DEG2RAD*desiredAngle),0,0,desiredAngle,1500,3,10);
			//status7=moveByPositionOffset(api,flight,0,distance*sin(DEG2RAD*desiredAngle),0,desiredAngle,1500,3,10);
			
		}
		
	}
        
        
      //! Do aircraft control - Waypoint example. 
     // wayPointMissionExample(api, waypointObj,blockingTimeout);
	
}
} 
	 }
      ackReturnData landingStatus = landing(api, flight,blockingTimeout);
    }
    else 
    {
      //Try to land directly
      ackReturnData landingStatus = landing(api, flight,blockingTimeout);
    }
  }
  //! No mode specified or invalid mode specified" 
  else
    std::cout << "\n Usage: ./djiosdk-linux-sample [MODE] [TRAJECTORY] [GAIN TUNING]\n"
                 "\n"
                 "[MODE] : \n"
                 "-mobile      : Run in Mobile Data Transparent Transmission mode\n"
                 "-interactive : Run in a Terminal-based UI mode\n"
                 "-programmatic: Run without user input, use if you have put automated\n"
                 "               calls in the designated space in the main function. \n"
                 "               By default this mode will execute an automated waypoint\n"
                 "               mission example, so be careful.\n\n"
                 "[TRAJECTORY] : \n"
                 "path_to_json_file : Optionally, supply a json file with parameters for executing a\n"
                 "                    trajectory planned with the DJI Trajectory SketchUp Plugin\n\n";
                 "[GAIN TUNING] : \n"
                 "path_to_json_file : Optionally, supply a json file with custom controller gains for\n"
                 "                    executing precision missions on loaded aircraft\n\n";
  //! Cleanup
  int cleanupStatus = cleanup(serialDevice, api, flight, &read);
  if (cleanupStatus == -1)
  {
    std::cout << "Unable to cleanly destroy OSDK infrastructure. There may be residual objects in the system memory.\n";
    return 0;
  }
  std::cout << "Program exited successfully." << std::endl;
  //myfile.close();
  return 0;
}


/*



void *Linux::save_data (void *param) {
int i=0;
int blockingTimeout = 1; 
  unsigned short dataFlag;
  TimeStampData timeStamp;
  QuaternionData q;
  CommonData ya;
  CommonData acc;
  VelocityData v;
  PositionData pos;
  MagnetData mag;
  GPSData gps;
  RTKData rtk;
  RadioData rc;
  FlightStatus status; //! @todo define enum
  BatteryData battery;
  CtrlInfoData ctrlInfo;
  Angle yaw;
  Angle roll;
  Angle pitch;
  PositionData curPosition;
  PositionData originPosition;
 	DJI::Vector3dData curLocalOffset; 
  DJI::EulerAngle curEuler;
usleep(500000);
  curPosition = flight -> getPosition();

	originPosition = curPosition;
 localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
	myfile.open ("test.csv");
	myfile << "Quaternion q0,Quaternion q1,quaternion q2,quaternion q3,velocity x,velocity y, velocity z,latitude,longitude,altitude,height, acceleration x,acceleration y, acceleration z, mag x, mag y, mag z,Yaw,Roll,Pitch,\n";
pos=flight -> getPosition();
    while (i<100){
        q=flight   -> getQuaternion();
        v=flight   -> getVelocity();
        pos=flight -> getPosition();
	acc=flight -> getAcceleration();
	ya=flight  -> getYawRate();
	mag=flight -> getMagnet();
        yaw=flight ->getYaw();
	roll=flight ->getRoll();
        pitch=flight ->getPitch();
        curPosition =flight -> getPosition();
curEuler = Flight::toEulerAngle(q);
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);

	//gps=flight -> 
	// rtk
	//rc	

	myfile <<q.q0  <<","<< q.q1 <<","<< q.q2 <<","<< q.q3 <<","<< v.x <<","<< v.y <<","<< v.z <<","<< pos.latitude <<","<< pos.longitude <<","<< pos.altitude <<","<< pos.height <<","<< acc.x <<","<< acc.y <<","<< acc.z <<","<< mag.x <<","<< mag.y <<","<< mag.z <<","<<yaw <<","<<roll<<","<<pitch<< ","<< curLocalOffset.x << ","<< curLocalOffset.y << ","<<curLocalOffset.z <<",\n";

	i=i+1;
	usleep(500000);
	
}	

}

void *LinuxThread::key_call (void *param) {
while(1){
c=getchar();

}
}

*/

