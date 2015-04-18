#include "pioneer.h"

LaserProxy* g_Laser;
SonarProxy* g_Sonar;

//next steps
//refer to assignment sheet
//yaw based on wheel turn
//try analysing the wall content
//up to 0.035 or 0.122 radians error per stop when turning 90 degrees
//up to 0.0873 or 0.175 when 180 degrees
//10th at 0.2
//0.05 @ 0.5
//RCL has 360 sonar, UG1 has 180 degree
//will report max distance if too close

void showLaserAndSonar() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glBegin(GL_LINES);

	if (g_Laser) {
		glColor3f(0, 1, 0);
		int n = g_Laser->GetCount();
		for (int i = 100; i < 3 * n / 8; i++) {
			glVertex3d(0, 0, -5);
			glVertex3d(-g_Laser->GetPoint(i).py, g_Laser->GetPoint(i).px, -5);
		}
	}

//	if (g_Sonar) {
//		glColor3f(1, 1, 1);
//		int n = g_Sonar->GetCount();
//		for (int i = 0; i < n; i++) {
//			player_pose3d_t pose = g_Sonar->GetPose(i);
//			glVertex3d(-pose.py, pose.px, -5);
//			glVertex3d(-pose.py - g_Sonar->GetScan(i) * sin(pose.pyaw), pose.px + g_Sonar->GetScan(i) * cos(pose.pyaw),
//					-5);
//		}
//	}

	glEnd();
	glutSwapBuffers();
}

void handleResize(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double) w / (double) h, 1.0, 200.0);
}

Pioneer::Pioneer(int argc, char **argv) {
	parse_args(argc, argv);

	robot = new PlayerClient(gHostname, gPort);
	position = new Position2dProxy(robot, gIndex);
	laser = new LaserProxy(robot, gIndex);
	sonar = new SonarProxy(robot, gIndex);
	speech = new SpeechProxy(robot, gIndex);

	laser->RequestGeom();
//	sonar->RequestGeom();
	g_Laser = laser;
//	g_Sonar = sonar;

	position->SetMotorEnable(true);
	robot->Read();
	robot->Read();
	robot->Read();

	laserCount = laser->GetCount();
	LASER_LEFT = laserCount - 1;
	LASER_NW = 3 * laserCount / 4;
	LASER_FRONT_LEFT = laserCount / 2;
	LASER_FRONT_RIGHT = laserCount / 2 + 1;
	LASER_NNE = LASER_FRONT_LEFT + 20;
	LASER_NNW = LASER_FRONT_RIGHT - 20;
	LASER_NE = laserCount / 4;
	LASER_RIGHT = 0;

	sonarCount = sonar->GetCount();
	SONAR_LEFT_FRONT = 0;
	SONAR_LEFT_BACK = sonarCount - 1;
	SONAR_FRONT_LEFT = sonarCount / 4 - 1;
	SONAR_FRONT_RIGHT = sonarCount / 4;
	SONAR_RIGHT_FRONT = sonarCount / 2 - 1;
	SONAR_RIGHT_BACK = sonarCount / 2;
	SONAR_BACK_LEFT = 3 * sonarCount / 4;
	SONAR_BACK_RIGHT = 3 * sonarCount / 4 - 1;
}

double Pioneer::absDiff(double a, double b, bool absolute = true) {
	double d = abs(a) - abs(b);
	return absolute ? abs(d) : d;
}

void Pioneer::turn(double angle, bool useLasers) {
	int direction = angle > 0 ? DIRECTION_LEFT : DIRECTION_RIGHT; //had problems with turning where the yaw would continue in previous direction causing angleIsBetween to fail (angle outside of start and target)
	double startYaw = position->GetYaw();
	double targetYaw = fmod(startYaw + angle, radians(360)); //expectedYaw
	if (targetYaw < 0) {
		targetYaw += radians(360);
	}

	while (angleDiff(targetYaw, position->GetYaw()) > BIG_ANGLE_GAP) { //use only with position->GetYaw() not expectedYaw
		robot->Read();
		position->SetSpeed(0, FAST * direction);
	}
	if (useLasers) {
		while (getRightDifference(true) > 0.005) {
			robot->Read();
			position->SetSpeed(0, SLOW * direction);
			//		cout << "right x: " << absDiff(laser->GetPoint(LASER_RIGHT).px, laser->GetPoint(LASER_RIGHT + 20).px) << "y: "
			//				<< absDiff(laser->GetPoint(LASER_RIGHT).py, laser->GetPoint(LASER_RIGHT + 20).py) << endl;
			//		cout << "front x: " << absDiff(laser->GetPoint(LASER_NNE).px, laser->GetPoint(LASER_FRONT_RIGHT).px) << "y: "
			//				<< absDiff(laser->GetPoint(LASER_NNE).py, laser->GetPoint(LASER_FRONT_RIGHT).py) << endl;
		}
	} else {
		while (angleDiff(targetYaw, position->GetYaw()) > ANGLE_GAP) { //use only with position->GetYaw() not expectedYaw
			robot->Read();
			position->SetSpeed(0, SLOW * direction);
		}
	}
}

double Pioneer::getFrontDifference(bool absolute) {
	return absDiff(laser->GetPoint(LASER_FRONT_RIGHT).px, laser->GetPoint(LASER_NNE).px, absolute);
}

double Pioneer::getRightDifference(bool absolute) {
	return absDiff(laser->GetPoint(LASER_RIGHT + 20).py, laser->GetPoint(LASER_RIGHT).py, absolute);
}

void Pioneer::printLaserPoints() {
	robot->Read();
	for (int i = 140; i < 180; i++) {
		player_point_2d p = laser->GetPoint(i);
		cout << p.px << "\t" << p.py << endl;
	}
}

void Pioneer::drive(bool checkForRooms = false) {
	double speed = FAST;
	double previousRange = laser->GetRange(LASER_RIGHT); //or use a range of lasers NE and E
	double previousDifference = 0;
	bool roomFound = false;
	player_point_2d previousNE = laser->GetPoint(LASER_NE);
	double yaw = 0;
	double distanceToCorner = 0;

	while (getFrontLaserRange() > GAP) {
		robot->Read();
//		cout << getFrontLaserRange() << endl;
		if (checkForRooms) {
//			double rangeDifference = laser->GetRange(LASER_RIGHT) - previousRange;
//			if (signOf(previousDifference) == signOf(rangeDifference) && abs(rangeDifference) > 0.001) {
//				rangeDifference += previousDifference;
//			}

//			cout << "NE previous NEx: "
//					<< absDiff(laser->GetPoint(LASER_RIGHT + 100).px, laser->GetPoint(LASER_RIGHT + 20).px, true)
//					<< endl;
//			cout << "NE previous NEy: "
//					<< absDiff(laser->GetPoint(LASER_RIGHT + 100).py, laser->GetPoint(LASER_RIGHT + 20).py, true)
//					<< endl;

			if (!roomFound) {
				if (ROOM_THRESHOLD < getRightDifference(false)) {
					speed = SLOW;
					roomFound = true;
					for (int i = 150; i < 200; i++) {
						if (absDiff(laser->GetPoint(i).px, laser->GetPoint(i - 1).px) > 0.01) {
							distanceToCorner = abs(laser->GetPoint(i).py);
							cout << "distance: " << distanceToCorner << endl;
							drive(distanceToCorner / 2);
							break;
						}
					}
				}
			} else {
				output("Room found - analysing content");
				turn(-radians(90), false);
				analyseRoom();
				turn(radians(90), true);
				cout << "distance: " << distanceToCorner << endl;
				drive(distanceToCorner / 2);
				roomFound = false;
////				cout << "NE previous NEx: " << absDiff(laser->GetPoint(LASER_NE).px, previousNE.px) << endl;
////				cout << "NE previous NEy: " << absDiff(laser->GetPoint(LASER_NE).py, previousNE.px) << endl;
////				if (absDiff(laser->GetPoint(LASER_NE).px, previousNE.px) > 0.001) {
////					cout << "NE previous NE: " << absDiff(laser->GetPoint(LASER_NE).px, previousNE) << endl;
////				}
//				//drive until px is half of it
////				if (absDiff(laser->GetPoint(LASER_RIGHT + 100).px, laser->GetPoint(LASER_RIGHT + 20).px, true) < 0.1) {
////				if (absDiff(laser->GetPoint(LASER_RIGHT + 100).px, laser->GetPoint(LASER_RIGHT + 20).px, true) < 0.1) {
//				output("Room found - analysing content");
//				turn(-radians(90), false);
//				analyseRoom();
//				turn(radians(90), false);
//				roomFound = false;
			}

			previousRange = laser->GetRange(LASER_RIGHT);
//			previousDifference = rangeDifference;
			previousNE = laser->GetPoint(LASER_NE);
		}

		if (!roomFound) {
			double difference = getRightDifference(false);
			cout << difference << endl;
			if (difference > 0.001) {
				yaw = -0.1;
			} else if (difference < -0.001) {
				yaw = 0.1;
			} else {
				yaw = 0;
			}
		}

		if (getFrontLaserRange() < BIG_GAP) {
			speed = SLOW;
		}
		position->SetSpeed(speed, yaw);
	}
}

//drives too far after room
void Pioneer::drive(double distance) {
	int direction = distance > 0 ? DIRECTION_FORWARD : DIRECTION_BACKWARD;
	double startPosition = position->GetXPos();

	while (absDiff(startPosition, position->GetXPos()) < abs(distance) && getFrontLaserRange() < GAP) {
		cout << "start: " << startPosition << " XPos" << position->GetXPos() << "distance: " << distance << endl;
		robot->Read();
		position->SetSpeed(FAST * direction, 0);
	}
}

void Pioneer::analyseRoom() {
	int objectsFound = 0;
	robot->Read();
	for (int i = LASER_NE; i < LASER_NW; i++) {
		if (laser->GetPoint(i - 1).px - laser->GetPoint(i).px > OBJECT_THRESHOLD) {
			objectsFound++;
		}
	}

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

void Pioneer::askIfOk() { // done
	time_t startTime = time(0);
	bool handDetected = false;

	while (!handDetected && (time(0) - startTime) < 10) {
		robot->Read();
		for (int i = 0; i < sonarCount; i++) {
			if (0.1 < sonar->GetScan(i) && sonar->GetScan(i) < HAND_THRESHOLD) {
				handDetected = true;
			}
		}
	}

	if (handDetected) {
		output("I'm glad you're ok!");
	} else {
		output("Help! Help!");
	}

}

int Pioneer::getClosestLaser() {
	int closestLaser = 0;
	double minDistance = laser->GetRange(0);

	robot->Read();
	for (int i = 0; i < laserCount; i++) {
		cout << i << ": " << laser->GetPoint(i) << endl;
		if (laser->GetRange(i) < minDistance) {
			closestLaser = i;
			minDistance = laser->GetRange(i);
		}
	}
	return closestLaser;
}

void Pioneer::moveToStartingCorner() {
	double frontAngle = laser->GetBearing(getClosestLaser());
	turn(frontAngle, true);
	drive();
	turn(radians(90), true);
	drive();
}

void Pioneer::run() {
	thread t([this] {this->runSpeechGenerator(); return;});
	output("Moving to a corner to start from");
	moveToStartingCorner();
	output("Starting search");
	for (int i = 0; i < 4; i++) {
		turn(radians(90), true);
		drive(true);
	}
	output("Search complete");
	exitThread = true;
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
	cout << text << endl;
	dialogue.push(text);
}

double Pioneer::getFrontLaserRange() {
	return (laser->GetRange(LASER_FRONT_LEFT) + laser->GetRange(LASER_FRONT_RIGHT)) / 2;
}

bool Pioneer::angleIsBetween(double start, double angle, double target, int direction) {
	cout << start << " " << angle << " " << target << " " << direction << endl;
	if (direction == DIRECTION_LEFT) {
		//robot does not need to go past 0/360 mark
		if (start < target) {
			return angle < target;
		}
		//before 0/360 mark, start <= angle, afterwards angle < target
		return start <= angle || angle < target;
	}
	if (target == 0) {
		return start >= angle;
	}
//robot does not need to go past 0/360 mark
	if (start > target) {
		return angle > target;
	}
//before 0/360 mark, angle <= start, afterwards target < angle
	return start >= angle || angle > target;
}

int Pioneer::signOf(double n) {
	return n < 0 ? -1 : 1;
}

double Pioneer::angleDiff(double a, double b) {
	double d = abs(a - b);
	return (d < M_PI) ? d : 2 * M_PI - d;
}

void test(int argc, char **argv) {
	glutInit(&argc, argv);
	glutInitWindowSize(500, 500);
	glutCreateWindow("Radar and Sonar Visualisation");
	glutDisplayFunc(showLaserAndSonar);
	glutReshapeFunc(handleResize);
	glutIdleFunc(glutPostRedisplay);
	glutMainLoop();
}

int main(int argc, char **argv) {
//	thread t(test, argc, argv);
	try {
		Pioneer pioneer(argc, argv);
		pioneer.run();
	} catch (PlayerError *e) {
		cout << e->GetErrorCode() << " " << e->GetErrorStr() << " " << e->GetErrorFun() << endl;
		return -1;
	}
//	t.join();
}
