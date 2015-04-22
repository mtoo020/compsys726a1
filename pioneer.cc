#include "pioneer.h"

// Represents the robot.
Pioneer::Pioneer(int argc, char **argv) {
	// parse the command line arguments given
	parse_args(argc, argv);

	// connect to player on the robot and initialise the proxies to be used
	robot = new PlayerClient(gHostname, gPort);
	position = new Position2dProxy(robot, gIndex);
	laser = new LaserProxy(robot, gIndex);
	sonar = new SonarProxy(robot, gIndex);
	speech = new SpeechProxy(robot, gIndex);

	laser->RequestGeom(); // allow GetPoint() to work - probably not needed
	position->SetMotorEnable(true); // enable movement

	// initialise the laser and sonar counts
	robot->Read();
	robot->Read();

	laserCount = laser->GetCount();
	sonarCount = sonar->GetCount();

	// useful laser indexes
	LASER_LEFT = laserCount - 1;
	LASER_NW = 3 * laserCount / 4;
	LASER_FRONT_LEFT = laserCount / 2;
	LASER_FRONT_RIGHT = laserCount / 2 - 1;
	LASER_NE = laserCount / 4;
	LASER_RIGHT = 0;
	LASER_ENE = LASER_RIGHT + 10;
}

// Turns the robot [angle] radians, using odometry and optionally lasers to improve accuracy.
void Pioneer::turn(double angle, bool useLasers) {
	int direction = angle > 0 ? DIRECTION_LEFT : DIRECTION_RIGHT;
	double startYaw = position->GetYaw();
	double targetYaw = fmod(startYaw + angle, radians(360));
	if (targetYaw < 0) {
		targetYaw += radians(360);
	}

	cout << "Turning " << degrees(angle) << " degrees" << endl;

	// quickly turn the robot most of the way
	while (angleDiff(targetYaw, position->GetYaw()) > BIG_ANGLE_GAP) {
		robot->Read();
		position->SetSpeed(0, FAST * direction);
	}

	// slow down near the end to improve accuracy
	if (useLasers) {
		// line the robot parallel with the right wall using lasers
		while (getLaserPoint(LASER_ENE).px - getLaserPoint(LASER_RIGHT).px > 0.002) {
			robot->Read();
			position->SetSpeed(0, SLOW * direction);
		}
	} else {
		// complete the turn using odometry
		while (angleDiff(startYaw, position->GetYaw()) < abs(angle)) {
			robot->Read();
			position->SetSpeed(0, SLOW * direction);
		}
	}
	// prevent the robot from continuing to move - helpful in stage.
	// not sure if it'll improve accuracy
	position->SetSpeed(0, 0);
}

// Drives the robot forward until it reaches a wall, optionally checking for rooms along the way.
void Pioneer::drive(bool checkForRooms) {
	double speed = FAST; // start off driving fast
	double yaw = 0;

	cout << "Driving - " << (checkForRooms ? "checking for" : "ignoring") << " rooms" << endl;

	// drive until the robot until the front of the robot is FRONT_GAP metres away from a wall.
	while (getFrontLaserRange() > FRONT_GAP) {
		robot->Read();

		if (checkForRooms) {
			// ensure the robot remains parallel with the right wall and about RIGHT_GAP metres away.
			double distance = laser->GetRange(LASER_RIGHT);
			double difference = getLaserPoint(LASER_ENE).px - getLaserPoint(LASER_RIGHT).px;
			if (difference > 0.001 && distance > RIGHT_GAP && distance < ROOM_DEPTH) { // don't turn into rooms through
				yaw = -0.12;
			} else if (difference < -0.001 && distance < RIGHT_GAP) {
				yaw = 0.12;
			} else {
				yaw = 0;
			}

			// room detection - try to ignore gaps in the wall
			if (ROOM_THRESHOLD < distance && distance < GAP_IN_WALL) {
				double startYaw = position->GetYaw();

				output("Room found");
				drive(0.25); // drive to the middle of the room
				turn(-radians(90)); // turn right
				analyseRoom(); // analyse the room
				turn(angleDiff(position->GetYaw(), startYaw)); // turn left to the original yaw - odometry is inaccurate so the turn may not be 90 degrees.
				drive(0.3); // drive to the end of the room
			}
		}

		// slow down as the robot gets closer to the wall
		if (getFrontLaserRange() < BIG_GAP) {
			speed = SLOW;
		}
		position->SetSpeed(speed, yaw);
	}
	// same reasoning as in the turn method
	position->SetSpeed(0, 0);
}

// Drives the robot [distance] metres.
void Pioneer::drive(double distance) {
	int direction = distance > 0 ? DIRECTION_FORWARD : DIRECTION_BACKWARD;
	player_point_2d startPosition = rotate90( { position->GetXPos(), position->GetYPos() });
	player_point_2d currentPosition = startPosition;

	cout << "Driving " << distance << "m" << endl;

	// drive slowly (to improve accuracy) until the distance between where the robot started and is currently is [distance] metres,
	// or until the robot reaches a wall.
	while (distanceBetween(startPosition, currentPosition) < distance && getFrontLaserRange() > FRONT_GAP) {
		currentPosition = rotate90( { position->GetXPos(), position->GetYPos() });
		robot->Read();
		position->SetSpeed(SLOW * direction, 0);
	}
	// same reasoning as in the turn method
	position->SetSpeed(0, 0);
}

// Analyses the contents of a side room.
void Pioneer::analyseRoom() {
	int objectsFound = 0;
	int lastObjectDetectionIndex = -100;

	cout << "Analysing room" << endl;

	// Using the front 90 degrees of lasers, examine the room looking for +/- jumps greater than OBJECT_THRESHOLD.
	// Ignore jumps from lasers with indexes of +/- 5 from the last index that detected a big jump,
	// as these will be from the same edge of the object.
	robot->Read();
	for (int i = LASER_NE; i < LASER_NW; i++) {
		if (abs(distanceBetween(getLaserPoint(i), getLaserPoint(i - 1))) > OBJECT_THRESHOLD) {
			if (abs(lastObjectDetectionIndex - i) > 5) {
				objectsFound++;
				lastObjectDetectionIndex = i;
			}
		}
	}

	// each object has two edges, and assume an object exists if only one edge is detected.
	objectsFound = ceil(objectsFound / 2.0);

	switch (objectsFound) {
	case 0:
		output("Nothing");
		break;
	case 1:
		output("An object");
		break;
	case 2: // a pair of legs
		output("Are you ok?");
		askIfOk();
		break;
	default:
		output("Not sure");
		break;
	}
}

// Asks if the person is ok and waits for their response.
void Pioneer::askIfOk() {
	time_t startTime = time(0);
	bool handDetected = false;

	cout << "Asking if they're ok" << endl;

	// give the person 10 seconds to put their hand in front of a sonar sensor to say that they're ok.
	while (!handDetected && (time(0) - startTime) < 10) {
		robot->Read();
		for (int i = 0; i < sonarCount; i++) {
			// the sonar sensor sometimes gives very small numbers - ignore these
			if (0.1 < sonar->GetScan(i) && sonar->GetScan(i) < HAND_THRESHOLD) {
				handDetected = true;
			}
		}
	}

	if (handDetected) {
		output("Thanks! I'm glad you're ok!");
	} else {
		output("Help!");
	}
}

// Moves the robot to a corner to start from.
void Pioneer::moveToStartingCorner() {
	output("Moving to a starting corner");
	double frontAngle = getClosestLaserBearing();
	turn(frontAngle); // turn towards the nearest wall
	drive(); // drive towards it, ignoring rooms
	turn(radians(90), true); // turn left using lasers as the right wall will be nearby
	drive(); // drive to the corner to start from, ignoring rooms
}

// Explores the building to find survivors.
void Pioneer::run() {
	// start the speech thread
	thread speechThread([this] {this->runSpeechGenerator(); return;});

	moveToStartingCorner();

	// examine four walls then conclude search
	output("Starting search");
	for (int i = 0; i < 4; i++) {
		turn(radians(90), true); // turn left using lasers as the right wall will be nearby
		drive(true); //drive along the wall checking for rooms
	}
	output("Search complete");

	exitThread = true; // signal the speech thread to finish.
	position->SetSpeed(0, 0); // immediately ensure any motion is stopped
	speechThread.join(); // wait until the speech thread has finished
	position->SetSpeed(0, 0); // finish gracefully
}

// The speech thread runs this method to say dialogue.
void Pioneer::runSpeechGenerator() {
	// continuously check a queue for any dialogue and say it.
	while (true) {
		if (!dialogue.empty()) {
			speech->Say(dialogue.front());
			dialogue.pop();
			sleep(2);
		} else if (exitThread) {
			// if the robot has completed its search, say any remaining dialogue on the queue then return.
			return;
		}
	}
}

// Outputs important messages as speech and to the terminal.
void Pioneer::output(string text) {
	dialogue.push(text); // adds the dialogue to the queue that the speech thread checks
	cout << text << endl;
}

// Returns the average distance in the front two lasers.
double Pioneer::getFrontLaserRange() {
	return (getLaserPoint(LASER_FRONT_LEFT).py + getLaserPoint(LASER_FRONT_RIGHT).py) / 2;
}

// Returns the smallest angle (in radians) between two bearings (in radians).
// For example, angleDiff(0, 2*M_PI) returns 0.
double Pioneer::angleDiff(double a, double b) {
	double d = abs(a - b);
	return (d < M_PI) ? d : 2 * M_PI - d;
}

// Returns the distance between two points, i.e. point [b] - point [a].
double Pioneer::distanceBetween(player_point_2d a, player_point_2d b) {
	double dx = b.px - a.px;
	double dy = b.py - a.py;
	return sqrt(dx * dx + dy * dy);
}

// Returns bearing of the laser with the shortest range reading.
double Pioneer::getClosestLaserBearing() {
	int closestLaser = 0;
	double minDistance = laser->GetRange(0);

	robot->Read();
	for (int i = 0; i < laserCount; i++) {
		if (laser->GetRange(i) < minDistance) {
			closestLaser = i;
			minDistance = laser->GetRange(i);
		}
	}
	return laser->GetBearing(closestLaser);
}

// Returns the point read by laser [i] rotated by 90 degrees, so that the Y axis is parallel to the heading of the robot.
// Useful because the robot initially considers the X axis to be parallel to its heading.
player_point_2d Pioneer::getLaserPoint(int i) {
	return rotate90(laser->GetPoint(i));
}

// Rotates a point 90 degrees.
player_point_2d Pioneer::rotate90(player_point_2d point) {
	double x = -point.py;
	double y = point.px;
	point.px = x;
	point.py = y;
	return point;
}

// Starts the robot.
int main(int argc, char **argv) {
	try {
		Pioneer pioneer(argc, argv);
		pioneer.run();
	} catch (PlayerError *e) {
		cout << e->GetErrorCode() << " " << e->GetErrorStr() << " " << e->GetErrorFun() << endl;
		return -1;
	}
}
