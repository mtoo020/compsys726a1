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

	int cornersCompleted = -2;

	int DIRECTION_LEFT = 1;
	int DIRECTION_RIGHT = -1;

	double ANGLE_LEFT = M_PI;
	double ANGLE_UP = M_PI_2;
	double ANGLE_RIGHT = 0;
	double ANGLE_DOWN = -M_PI_2;

	double BIG_ANGLE = M_PI/9;
	double ANGLE = M_PI/60;
	double BIG_GAP = 0.7;
	double GAP = 0.4;
	double SLOW = 0.2;
	double FAST = 1;

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

	void printLaser();
	void printSonar();

	int getClosestLaser();
	int getClosestSonar();
	double getLaserAngleError(int threshold);
	double getTrueSonarAngle(int index);
	int getTurningDirection();
	double absDiff(double a, double b);

	void turnToNearestWall(int direction);
	void driveToWall();
	void turn90(int direction);

public:
	Pioneer(int argc, char **argv);
	void run();
};

#endif /* PIONEER_H_ */
