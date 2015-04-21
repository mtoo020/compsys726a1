/*
 * pioneer.h
 *
 *  Created on: 2 Apr 2015
 *      Author: osboxes
 */

#ifndef PIONEER_H_
#define PIONEER_H_

#define degrees(r) (r/(2*M_PI) * 360.0) // converts radians to degrees
#define radians(d) (d/360.0 * 2*M_PI) // converts degrees to radians

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
	// player and proxies
	PlayerClient* robot;
	Position2dProxy* position;
	LaserProxy* laser;
	SonarProxy* sonar;
	SpeechProxy* speech;

	// directions used when driving and turning
	int DIRECTION_FORWARD = 1;
	int DIRECTION_BACKWARD = -1;
	int DIRECTION_LEFT = 1;
	int DIRECTION_RIGHT = -1;

	double BIG_ANGLE_GAP = radians(5); // final angle amount to turn slowly
	double BIG_GAP = 0.6; // final distance to drive slowly when approaching a wall
	double FRONT_GAP = 0.4; // distance between the front of the robot and the wall once finished driving
	double RIGHT_GAP = 0.4; // distance between the right of the robot and the wall while driving

	// speeds used for driving and turning
	double SLOW = 0.2;
	double FAST = 0.3;

	double GAP_IN_WALL = 2; // distances equal or greater to this will be ignored when checking for rooms.
	double ROOM_THRESHOLD = 0.75; // distance used for detecting rooms
	double OBJECT_THRESHOLD = 0.1; // distance used for detecting an object's edge
	double HAND_THRESHOLD = 0.25; // distance used for detecting a hand

	int laserCount;
	int sonarCount;

	// useful laser indexes
	int LASER_LEFT;
	int LASER_NW;
	int LASER_FRONT_LEFT;
	int LASER_FRONT_RIGHT;
	int LASER_NE;
	int LASER_ENE;
	int LASER_RIGHT;

	queue<string> dialogue; // queue used by the speech thread
	bool exitThread = false; // signal the speech thread to finish

	// see comments in cc file
	void runSpeechGenerator();
	void moveToStartingCorner();
	void turn(double angle, bool useLasers = false);
	void drive(bool checkForRooms = false);
	void drive(double distance);
	void analyseRoom();
	void askIfOk();
	void output(string text);

	double getFrontLaserRange();
	double getClosestLaserBearing();
	player_point_2d getLaserPoint(int i);
	player_point_2d rotate90(player_point_2d point);
	double distanceBetween(player_point_2d a, player_point_2d b);
	double angleDiff(double a, double b);

public:
	Pioneer(int argc, char **argv);
	void run();
};

#endif /* PIONEER_H_ */
