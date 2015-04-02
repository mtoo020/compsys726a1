/*
 * pioneer.h
 *
 *  Created on: 2 Apr 2015
 *      Author: osboxes
 */

#ifndef PIONEER_H_
#define PIONEER_H_

#include <libplayerc++/playerc++.h>
#include <iostream>
#include <stdlib.h>
#include "args.h"
#include <GL/glut.h>
#include <thread>
#include <math.h>

using namespace PlayerCc;
using namespace std;

class Pioneer {
private:
	static Pioneer* pioneer;
	PlayerClient* robot;
	Position2dProxy* position;
	SonarProxy* sonar;
	LaserProxy* laser;
//	SpeechProxy* speech;

	thread* t;

	int laserCount;
	double LASER_LEFT;
	double LASER_FRONT;
	double LASER_RIGHT;

	int sonarCount;
	double SONAR_LEFT_FRONT;
	double SONAR_LEFT_BACK;
	double SONAR_FRONT_LEFT;
	double SONAR_FRONT_RIGHT;
	double SONAR_RIGHT_FRONT;
	double SONAR_RIGHT_BACK;
	double SONAR_BACK_LEFT;
	double SONAR_BACK_RIGHT;

	void printSonar();
	void printLaser();

public:
	Pioneer(int argc, char **argv);
	void run();
};

#endif /* PIONEER_H_ */
