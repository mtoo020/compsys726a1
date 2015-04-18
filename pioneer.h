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
#include <queue>

using namespace PlayerCc;
using namespace std;

class Pioneer {
private:
	PlayerClient* robot;
	Position2dProxy* position;
	SonarProxy* sonar;
	LaserProxy* laser;
	SpeechProxy* speech;

	int DIRECTION_FORWARD = 1;
	int DIRECTION_BACKWARD = -1;
	int DIRECTION_LEFT = 1;
	int DIRECTION_RIGHT = -1;

	double ANGLE_LEFT = M_PI;
	double ANGLE_UP = M_PI_2;
	double ANGLE_RIGHT = 0;
	double ANGLE_DOWN = -M_PI_2;

	double BIG_ANGLE_GAP = radians(15);
	double ANGLE_GAP = radians(2);
	double BIG_GAP = 0.9;
	double GAP = 0.35;
	double SLOW = 0.2;
	double FAST = 0.3;

	double ROOM_THRESHOLD = 0.3;
	double OBJECT_THRESHOLD = ROOM_THRESHOLD / 4;
	double HAND_THRESHOLD = 0.25;

	int FRONT_LASER_THRESHOLD = 10;

	int laserCount;
	int LASER_LEFT;
	int LASER_NW;
	int LASER_NNW;
	int LASER_FRONT_LEFT;
	int LASER_FRONT_RIGHT;
	int LASER_NNE;
	int LASER_NE;
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

	queue<string> dialogue;
	bool exitThread = false;

	double absDiff(double a, double b, bool absolute);
	bool angleIsBetween(double start, double angle, double target, int direction);
	int signOf(double n);
	double angleDiff(double a, double b);

	double getFrontLaserRange();
	int getClosestLaser();
	void runSpeechGenerator();
	void moveToStartingCorner();
	void turn(double angle, bool useLasers);
	void drive(bool checkForRooms);
	void drive(double distance);
	void analyseRoom();
	void askIfOk();
	void output(string text);
	double getRightDifference(bool absolute);
	double getFrontDifference(bool absolute);
	void printLaserPoints();

public:
	Pioneer(int argc, char **argv);
	void run();
};

#endif /* PIONEER_H_ */
