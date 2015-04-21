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
	LaserProxy* laser;
	SonarProxy* sonar;
	SpeechProxy* speech;

	int DIRECTION_FORWARD = 1;
	int DIRECTION_BACKWARD = -1;
	int DIRECTION_LEFT = 1;
	int DIRECTION_RIGHT = -1;

	double BIG_ANGLE_GAP = radians(5);
	double BIG_GAP = 0.6;
	double FRONT_GAP = 0.4;
	double RIGHT_GAP = 0.4;
	double SLOW = 0.2;
	double FAST = 0.3;

	double ROOM_THRESHOLD = 0.75;
	double OBJECT_THRESHOLD = 0.1;
	double HAND_THRESHOLD = 0.25;

	int laserCount;
	int LASER_LEFT;
	int LASER_NW;
	int LASER_FRONT_LEFT;
	int LASER_FRONT_RIGHT;
	int LASER_NE;
	int LASER_ENE;
	int LASER_RIGHT;

	int sonarCount;

	queue<string> dialogue;
	bool exitThread = false;

	void runSpeechGenerator();
	void moveToStartingCorner();
	void turn(double angle, bool useLasers = false);
	void drive(bool checkForRooms = false);
	void drive(double distance);
	void analyseRoom();
	void askIfOk();
	void output(string text);

	double getFrontLaserRange();
	double getClosestLaserBearing(int first, int last);
	player_point_2d getLaserPoint(int i);
	player_point_2d rotate90(player_point_2d point);
	double distanceBetween(player_point_2d a, player_point_2d b);
	double angleDiff(double a, double b);

public:
	Pioneer(int argc, char **argv);
	void run();
};

#endif /* PIONEER_H_ */
