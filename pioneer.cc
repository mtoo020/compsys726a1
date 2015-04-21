#include "pioneer.h"

Pioneer::Pioneer(int argc, char **argv) {
	parse_args(argc, argv);

	robot = new PlayerClient(gHostname, gPort);
	position = new Position2dProxy(robot, gIndex);
	laser = new LaserProxy(robot, gIndex);
	sonar = new SonarProxy(robot, gIndex);
	speech = new SpeechProxy(robot, gIndex);

	laser->RequestGeom();
	position->SetMotorEnable(true);
	robot->Read();
	robot->Read();

	laserCount = laser->GetCount();
	LASER_LEFT = laserCount - 1;
	LASER_NW = 3 * laserCount / 4;
	LASER_FRONT_LEFT = laserCount / 2;
	LASER_FRONT_RIGHT = laserCount / 2 - 1;
	LASER_NE = laserCount / 4;
	LASER_RIGHT = 0;
	LASER_ENE = LASER_RIGHT + 10;

	sonarCount = sonar->GetCount();
}

void Pioneer::turn(double angle, bool useLasers) {
	int direction = angle > 0 ? DIRECTION_LEFT : DIRECTION_RIGHT;
	double startYaw = position->GetYaw();
	double targetYaw = fmod(startYaw + angle, radians(360));
	if (targetYaw < 0) {
		targetYaw += radians(360);
	}

	cout << "Turning " << degrees(angle) << " degrees" << endl;

	while (angleDiff(targetYaw, position->GetYaw()) > BIG_ANGLE_GAP) {
		robot->Read();
		position->SetSpeed(0, FAST * direction);
	}
	if (useLasers) {
		while (getLaserPoint(LASER_ENE).px - getLaserPoint(LASER_RIGHT).px > 0.002) {
			robot->Read();
			position->SetSpeed(0, SLOW * direction);
		}
	} else {
		while (angleDiff(startYaw, position->GetYaw()) < abs(angle)) {
			robot->Read();
			position->SetSpeed(0, SLOW * direction);
		}
	}
	position->SetSpeed(0, 0);
}

void Pioneer::drive(bool checkForRooms) {
	double speed = FAST;
	double yaw = 0;

	cout << "Driving - " << (checkForRooms ? "checking for" : "ignoring") << " rooms" << endl;

	while (getFrontLaserRange() > FRONT_GAP) {
		robot->Read();

		if (checkForRooms) {
			double distance = laser->GetRange(LASER_RIGHT);
			double difference = getLaserPoint(LASER_ENE).px - getLaserPoint(LASER_RIGHT).px;
			if (difference > 0.001 && distance > RIGHT_GAP && distance < ROOM_THRESHOLD) {
				yaw = -0.12;
			} else if (difference < -0.001 && distance < RIGHT_GAP) {
				yaw = 0.12;
			} else {
				yaw = 0;
			}
			if (distance > ROOM_THRESHOLD) {
				double startYaw = position->GetYaw();

				output("Room found");
				drive(0.25);
				turn(-radians(90));
				analyseRoom();
				turn(angleDiff(position->GetYaw(), startYaw));
				drive(0.3);
			}
		}
		if (getFrontLaserRange() < BIG_GAP) {
			speed = SLOW;
		}
		position->SetSpeed(speed, yaw);
	}
	position->SetSpeed(0, 0);
}

void Pioneer::drive(double distance) {
	int direction = distance > 0 ? DIRECTION_FORWARD : DIRECTION_BACKWARD;
	player_point_2d startPosition = rotate90( { position->GetXPos(), position->GetYPos() });
	player_point_2d currentPosition = startPosition;

	cout << "Driving " << distance << "m" << endl;

	while (distanceBetween(startPosition, currentPosition) < distance && getFrontLaserRange() > FRONT_GAP) {
		currentPosition = rotate90( { position->GetXPos(), position->GetYPos() });
		robot->Read();
		position->SetSpeed(SLOW * direction, 0);
	}
	position->SetSpeed(0, 0);
}

void Pioneer::analyseRoom() {
	int objectsFound = 0;
	int lastObjectDetection = -100;

	cout << "Analysing room" << endl;

	robot->Read();
	for (int i = LASER_NE; i < LASER_NW; i++) {
		if (abs(distanceBetween(getLaserPoint(i), getLaserPoint(i - 1))) > OBJECT_THRESHOLD) {
			if (abs(lastObjectDetection - i) > 5) {
				objectsFound++;
				lastObjectDetection = i;
			}
		}
	}
	objectsFound = ceil(objectsFound / 2.0);

	switch (objectsFound) {
	case 0:
		output("Nothing");
		break;
	case 1:
		output("An object");
		break;
	case 2:
		output("Are you ok?");
		askIfOk();
		break;
	default:
		output("Not sure");
		break;
	}
}

void Pioneer::askIfOk() {
	time_t startTime = time(0);
	bool handDetected = false;

	cout << "Asking if they're ok" << endl;

	while (!handDetected && (time(0) - startTime) < 10) {
		robot->Read();
		for (int i = 0; i < sonarCount; i++) {
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

void Pioneer::moveToStartingCorner() {
	output("Moving to a starting corner");
	double frontAngle = getClosestLaserBearing(0, laserCount);
	turn(frontAngle, true);
	drive();
	turn(radians(90), true);
	drive();
}

void Pioneer::run() {
	thread t([this] {this->runSpeechGenerator(); return;});

	moveToStartingCorner();
	output("Starting search");
	for (int i = 0; i < 4; i++) {
		turn(radians(90), true);
		drive(true);
	}
	output("Search complete");

	exitThread = true;
	position->SetSpeed(0, 0);
	t.join();
	position->SetSpeed(0, 0);
}

void Pioneer::runSpeechGenerator() {
	while (true) {
		if (!dialogue.empty()) {
			speech->Say(dialogue.front());
			dialogue.pop();
			sleep(2);
		} else if (exitThread) {
			return;
		}
	}
}

void Pioneer::output(string text) {
	dialogue.push(text);
	cout << text << endl;
}

double Pioneer::getFrontLaserRange() {
	return (getLaserPoint(LASER_FRONT_LEFT).py + getLaserPoint(LASER_FRONT_RIGHT).py) / 2;
}

double Pioneer::angleDiff(double a, double b) {
	double d = abs(a - b);
	return (d < M_PI) ? d : 2 * M_PI - d;
}

double Pioneer::distanceBetween(player_point_2d a, player_point_2d b) {
	double dx = b.px - a.px;
	double dy = b.py - a.py;
	return sqrt(dx * dx + dy * dy);
}

double Pioneer::getClosestLaserBearing(int first, int last) {
	int closestLaser = 0;
	double minDistance = laser->GetRange(0);

	robot->Read();
	for (int i = first; i < last; i++) {
		if (laser->GetRange(i) < minDistance) {
			closestLaser = i;
			minDistance = laser->GetRange(i);
		}
	}
	return laser->GetBearing(closestLaser);
}

player_point_2d Pioneer::getLaserPoint(int i) {
	return rotate90(laser->GetPoint(i));
}

player_point_2d Pioneer::rotate90(player_point_2d point) {
	double x = -point.py;
	double y = point.px;
	point.px = x;
	point.py = y;
	return point;
}

int main(int argc, char **argv) {
	try {
		Pioneer pioneer(argc, argv);
		pioneer.run();
	} catch (PlayerError *e) {
		cout << e->GetErrorCode() << " " << e->GetErrorStr() << " " << e->GetErrorFun() << endl;
		return -1;
	}
}
