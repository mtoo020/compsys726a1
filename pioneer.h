/*
 * pioneer.h
 *
 *  Created on: 2 Apr 2015
 *      Author: osboxes
 */

#ifndef PIONEER_H_
#define PIONEER_H_

#define degrees(r) (r/(2*M_PI) * 360.0)
#define radians(d) (d/360.0 * 2*M_PI)

#include <libplayerc++/playerc++.h>
#include <iostream>
#include <cstdlib>
#include "args.h"
#include <GL/glut.h>
#include <thread>
#include <cmath>
#include <ctime>

using namespace PlayerCc;
using namespace std;

class Pioneer {
private:
	PlayerClient* robot;
	Position2dProxy* position;
	SonarProxy* sonar;
	LaserProxy* laser;
	SpeechProxy* speech;

	int DIRECTION_LEFT = 1;
	int DIRECTION_RIGHT = -1;

	double ANGLE_LEFT = M_PI;
	double ANGLE_UP = M_PI_2;
	double ANGLE_RIGHT = 0;
	double ANGLE_DOWN = -M_PI_2;

	double BIG_ANGLE_GAP = radians(15);
	double ANGLE_GAP = radians(3); //radians(1) robot
	double BIG_GAP = 0.8; // 0.6 robot
	double GAP = 0.5; //0.3 robot;
	double SLOW = 0.2; //0.2 0.15 minimum robot
	double FAST = 0.3;
	double TURNING_ERROR = radians(2); //0.05 robot;

	double ROOM_THRESHOLD = 0.5;
	double OBJECT_THRESHOLD = ROOM_THRESHOLD / 3;
	double HAND_THRESHOLD = 4;

	int FRONT_LASER_THRESHOLD = 10;

	int laserCount;
	int LASER_LEFT;
	int LASER_FRONT_LEFT;
	int LASER_FRONT_RIGHT;
	int LASER_RIGHT;

	int sonarCount; //only front end
	int SONAR_LEFT_FRONT;
	int SONAR_LEFT_BACK;
	int SONAR_FRONT_LEFT;
	int SONAR_FRONT_RIGHT;
	int SONAR_RIGHT_FRONT;
	int SONAR_RIGHT_BACK;
	int SONAR_BACK_LEFT;
	int SONAR_BACK_RIGHT;

	double angleDiff(double a, double b);
	double getFrontLaserRange();
	double getClosestFrontLaserAngle();
	double getClosestLaserAngle();
	double getClosestSonarAngle();
	void askIfOk();
	void drive(bool checkForCavity);
	void turn(double angle);
	void output(string text);

public:
	Pioneer(int argc, char **argv);
	void run();
};

#endif /* PIONEER_H_ */
