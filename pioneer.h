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
	PlayerClient* robot;
	Position2dProxy* position;
	SonarProxy* sonar;
	LaserProxy* laser;
//	SpeechProxy* speech;

	int DIRECTION_LEFT = 1;
	int DIRECTION_RIGHT = -1;

	double ANGLE_LEFT = M_PI;
	double ANGLE_UP = M_PI_2;
	double ANGLE_RIGHT = 0;
	double ANGLE_DOWN = -M_PI_2;

	double ROOM_THRESHOLD = 0.8;
	double OBJECT_THRESHOLD = ROOM_THRESHOLD / 2;
	double BIG_ANGLE_GAP = M_PI/9;
	double ANGLE_GAP = M_PI/60;
	double BIG_GAP = 0.7;
	double GAP = 0.4;
	double SLOW = 0.2;
	double FAST = 0.5;

	int FRONT_LASER_THRESHOLD = 10;

	int laserCount;
	int LASER_LEFT;
	int LASER_FRONT_LEFT;
	int LASER_FRONT_RIGHT;
	int LASER_RIGHT;

	int sonarCount;
	int SONAR_LEFT_FRONT;
	int SONAR_LEFT_BACK;
	int SONAR_FRONT_LEFT;
	int SONAR_FRONT_RIGHT;
	int SONAR_RIGHT_FRONT;
	int SONAR_RIGHT_BACK;
	int SONAR_BACK_LEFT;
	int SONAR_BACK_RIGHT;

	void printLaser();
	void printSonar();

	double absDiff(double a, double b);
	double getFrontLaserRange();
	double getClosestLaserAngle();
	double getClosestSonarAngle();

	void drive(bool checkForCavity);
	void turn(double angle, bool checkFrontLasers);

public:
	Pioneer(int argc, char **argv);
	void run();
};

#endif /* PIONEER_H_ */
