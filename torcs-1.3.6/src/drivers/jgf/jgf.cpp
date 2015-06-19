/***************************************************************************

    file                 : jgf.cpp
    created              : Wed Jan 8 18:31:16 CET 2003
    copyright            : (C) 2002-2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: jgf.cpp,v 1.5.2.2 2008/11/09 17:50:19 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "driver.h"

#define NBBOTS 10

struct CpsControlOutput
{
public:
	double accelerator;
	double steeringAngle;
}; 

static const char* botname[NBBOTS] = {
	"jgf 1", "jgf 2", "jgf 3", "jgf 4", "jgf 5",
	"jgf 6", "jgf 7", "jgf 8", "jgf 9", "jgf 10"
};

static const char* botdesc[NBBOTS] = {
	"jgf 1", "jgf 2", "jgf 3", "jgf 4", "jgf 5",
	"jgf 6", "jgf 7", "jgf 8", "jgf 9", "jgf 10"
};

static Driver *driver[NBBOTS];

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void recordData(tCarElt* car, double simTime);
static int pitcmd(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);
static void endRace(int index, tCarElt *car, tSituation *s);
static CpsControlOutput runCarControl();
static int getGear(tCarElt* car);

// Module entry point.
extern "C" int jgf(tModInfo *modInfo)
{
	int i;
	
	// Clear all structures.
	memset(modInfo, 0, 10*sizeof(tModInfo));

	for (i = 0; i < NBBOTS; i++) {
		modInfo[i].name    = strdup(botname[i]);	// name of the module (short).
		modInfo[i].desc    = strdup(botdesc[i]);	// Description of the module (can be long).
		modInfo[i].fctInit = InitFuncPt;			// Init function.
		modInfo[i].gfId    = ROB_IDENT;				// Supported framework version.
		modInfo[i].index   = i;						// Indices from 0 to 9.
	}
	return 0;
}


// Module interface initialization.
static int InitFuncPt(int index, void *pt)
{
	tRobotItf *itf = (tRobotItf *)pt;

	// Create robot instance for index.
	driver[index] = new Driver(index);
	itf->rbNewTrack = initTrack;	// Give the robot the track view called.
	itf->rbNewRace  = newRace;		// Start a new race.
	itf->rbDrive    = drive;		// Drive during race.
	itf->rbPitCmd   = pitcmd;		// Pit commands.
	itf->rbEndRace  = endRace;		// End of the current race.
	itf->rbShutdown = shutdown;		// Called before the module is unloaded.
	itf->index      = index;		// Index used if multiple interfaces.
	return 0;
}


// Called for every track change or new race.
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
	driver[index]->initTrack(track, carHandle, carParmHandle, s);
}

// Start a new race.
static void newRace(int index, tCarElt* car, tSituation *s)
{
	driver[index]->newRace(car, s);
}




// Drive during race.
static void drive(int index, tCarElt* car, tSituation *s)
{
	//driver[index]->drive(s);
	memset(&car->ctrl, 0, sizeof(tCarCtrl));

	if (index == 0)
	{
		CpsControlOutput cco = runCarControl();

		//move me forward
		car->ctrl.accelCmd = cco.accelerator;
		car->ctrl.gear = getGear(car);

		//steer me
		NORM_PI_PI(cco.steeringAngle);
		float angle;
		const float SC = 0.1f;

		angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
		NORM_PI_PI(angle); // put the angle back in the range from -PI to PI
		angle -= SC*car->_trkPos.toMiddle / car->_trkPos.seg->width;
		car->ctrl.steer = angle / car->_steerLock;
		recordData(car, s->currentTime);
	}
	else
	{
		float angle;
		const float SC = 0.1f;

		angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw - PI;
		NORM_PI_PI(angle); // put the angle back in the range from -PI to PI
		angle += SC*car->_trkPos.toMiddle / car->_trkPos.seg->width;
		car->ctrl.accelCmd = 0.9f;
		car->ctrl.gear = getGear(car);
		car->ctrl.steer = angle / car->_steerLock;
	}
}

const float SHIFT = 0.9f;							// [-] (% of rpmredline) When do we like to shift gears.
const float SHIFT_MARGIN = 4.0f;					// [m/s] Avoid oscillating gear changes.

// Compute gear.
static int getGear(tCarElt* car)
{
	if (car->_gear <= 0) {
		return 1;
	}
	float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
	float omega = car->_enginerpmRedLine / gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	}
	else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = car->_enginerpmRedLine / gr_down;
		if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
			return car->_gear - 1;
		}
	}
	return car->_gear;
}

static const int flushThreshold = 100;
static char* records[flushThreshold];
static bool csvHeaderWritten = false;
static int flushCounter = 0;

static void recordData(tCarElt* car, double simTime)
{
	if (!csvHeaderWritten)
	{
		//Write CSV Header
		records[flushCounter++] = "drive_step,theta_c,x_dot_c,y_dot_c,x_c,y_c,x_o\n";
		csvHeaderWritten = true;
	}
	//Write CSV Data Line
	char * logLine = (char *)(malloc(255));
	sprintf(logLine, "%f,%f,%f,%f,%f,%f,%f\n", simTime, car->_yaw, car->pub.DynGCg.vel.x, car->pub.DynGCg.vel.y, car->pub.DynGCg.pos.x, car->pub.DynGCg.pos.y, 0.0);
	records[flushCounter] = logLine;

	if (++flushCounter == flushThreshold)
	{
		FILE * logFile = fopen(".\\Log.csv", "a+");
		int i;
		for (i = 0; i < flushThreshold; i++)
		{
			fprintf(logFile, records[i]);
		}
		fclose(logFile);
		flushCounter = 0;
	}
}

static CpsControlOutput runCarControl()
{
	//TODO: Replace function body with actual control
	CpsControlOutput cco;
	cco.accelerator = 0.3;
	cco.steeringAngle = 0.0;
	return cco;
}

// Pitstop callback.
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
	return driver[index]->pitCommand(s);
}


// End of the current race.
static void endRace(int index, tCarElt *car, tSituation *s)
{
	driver[index]->endRace(s);
}


// Called before the module is unloaded.
static void shutdown(int index)
{
	delete driver[index];
}



