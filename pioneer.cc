#include "pioneer.h"

LaserProxy* g_Laser;
//SonarProxy* g_Sonar;

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

	glRotatef(90, 0, 0, 1);
	glBegin(GL_LINES);

	if (g_Laser) {
		glColor3f(0, 1, 0);
		int n = g_Laser->GetCount();
		for (int i = 0; i < n; i++) {
			glVertex3d(0, 0, -5);
			glVertex3d(g_Laser->GetPoint(i).px, g_Laser->GetPoint(i).py, -5);
		}
	}

//	if (g_Sonar) {
//		glColor3f(1, 1, 1);
//		int n = g_Sonar->GetCount();
//		for (int i = 0; i < n; i++) {
//			player_pose3d_t pose = g_Sonar->GetPose(i);
//			glVertex3d(pose.px, pose.py, -5);
//			glVertex3d(pose.px + g_Sonar->GetScan(i) * cos(pose.pyaw), pose.py + g_Sonar->GetScan(i) * sin(pose.pyaw));
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

void test(int argc, char **argv) {
	glutInit(&argc, argv);
	glutInitWindowSize(500, 500);
	glutCreateWindow("Radar and Sonar Visualisation");
	glutDisplayFunc(showLaserAndSonar);
	glutReshapeFunc(handleResize);
	glutIdleFunc(glutPostRedisplay);
	glutMainLoop();
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

	laserCount = laser->GetCount();
	LASER_LEFT = laserCount - 1;
	LASER_NW = 3 * laserCount / 4;
	LASER_FRONT_LEFT = laserCount / 2;
	LASER_FRONT_RIGHT = laserCount / 2 - 1;
	LASER_NE = laserCount / 4;
	LASER_RIGHT = 0;
	LASER_NEE = LASER_RIGHT + 10;

	sonarCount = sonar->GetCount();
//	SONAR_LEFT_FRONT = 0;
//	SONAR_LEFT_BACK = sonarCount - 1;
//	SONAR_FRONT_LEFT = sonarCount / 4 - 1;
//	SONAR_FRONT_RIGHT = sonarCount / 4;
//	SONAR_RIGHT_FRONT = sonarCount / 2 - 1;
//	SONAR_RIGHT_BACK = sonarCount / 2;
//	SONAR_BACK_LEFT = 3 * sonarCount / 4;
//	SONAR_BACK_RIGHT = 3 * sonarCount / 4 - 1;
}

void Pioneer::turn(double angle, bool useLasers) {
	int direction = angle > 0 ? DIRECTION_LEFT : DIRECTION_RIGHT;
	double startYaw = position->GetYaw();
	double targetYaw = fmod(startYaw + angle, radians(360));
	if (targetYaw < 0) {
		targetYaw += radians(360);
	}

	while (angleDiff(targetYaw, position->GetYaw()) > BIG_ANGLE_GAP) { //verify
		robot->Read();
		position->SetSpeed(0, FAST * direction);
	}
	if (useLasers) {
		while (getLaserPoint(LASER_NEE).px - getLaserPoint(LASER_RIGHT).px > 0.002) { //verified - more accurate than getFrontLaser()
			robot->Read();
			position->SetSpeed(0, SLOW * direction);
		}
	} else {
		while (angleDiff(startYaw, position->GetYaw()) < abs(angle)) { //verify
			robot->Read();
			position->SetSpeed(0, SLOW * direction);
		}
	}
	position->SetSpeed(0, 0);
}

void Pioneer::drive(bool checkForRooms = false) { //driving away needs improvement
	double speed = FAST;
	double yaw = 0;
//	double distanceToCorner = 0;

	while (getFrontLaserRange() > FRONT_GAP) {
		robot->Read();

		if (checkForRooms) {
			double distance = laser->GetRange(LASER_RIGHT);
			double difference = getLaserPoint(LASER_NEE).px - getLaserPoint(LASER_RIGHT).px;
			if (difference > 0.001 && distance > RIGHT_GAP && distance < ROOM_THRESHOLD) {
				yaw = -0.12;
			} else if (difference < -0.001 && distance < RIGHT_GAP) {
				yaw = 0.12;
			} else {
				yaw = 0;
			}
			if (distance > ROOM_THRESHOLD) { //verify
				double startYaw = position->GetYaw();
				drive(0.25);
//				printLaserPoints();
//				for (int i = 120; i < 180; i++) { //verify with object in room
//					if (getLaserPoint(i).py - getLaserPoint(i - 1).py > 0.01) { //verify
//						distanceToCorner = abs(getLaserPoint(i).py);
//						drive(distanceToCorner / 2);
//						break;
//					}
//				}
				output("Room found, analysing content");
				turn(-radians(90), false); //compare sides of room
//				correctYaw();
				analyseRoom();
				turn(angleDiff(position->GetYaw(), startYaw), false);
//				drive(distanceToCorner / 2);
				drive(0.3); //or drive until out of room
				cout << "out" << endl;
			}
		}
		if (getFrontLaserRange() < BIG_GAP) {
			speed = SLOW;
		}
		position->SetSpeed(speed, yaw);
	}
	position->SetSpeed(0, 0);
}

void Pioneer::drive(double distance) { //verified
	int direction = distance > 0 ? DIRECTION_FORWARD : DIRECTION_BACKWARD;
	player_point_2d startPosition = rotate90( { position->GetXPos(), position->GetYPos() });
	player_point_2d currentPosition = startPosition;

	while (distanceBetween(startPosition, currentPosition) < distance && getFrontLaserRange() > FRONT_GAP) {
		currentPosition = rotate90( { position->GetXPos(), position->GetYPos() });
		robot->Read();
		position->SetSpeed(SLOW * direction, 0);
	}
	position->SetSpeed(0, 0);
}

//void Pioneer::correctYaw() {
//	double dx = 0;
//	double dy = 0;
//	int n = 10;
//
//	for (int i = LASER_NE + 1; i < LASER_NE + n; i++) {
//		dx += getLaserPoint(i).px - getLaserPoint(i - 1).px;
//		dy += getLaserPoint(i).py - getLaserPoint(i - 1).py;
//	}
//	double averageDX = getLaserPoint(LASER_NE + n).px - getLaserPoint(LASER_NE).px;
//	double averageDY = getLaserPoint(LASER_NE + n).py - getLaserPoint(LASER_NE).py;
//	double adjacent = dy;
//	double opposite = dx;
//
//	cout << averageDX << " " << dx << endl;
//	cout << averageDY << " " << dy << endl;
//	if (abs(averageDX - dx) < 0.01 && abs(averageDY - dy) < 0.01) {
//		cout << degrees(atan(opposite / adjacent)) << endl;
//	}
//	printLaserPoints();
//}

void Pioneer::analyseRoom() { //verify - not working at angles
	int objectsFound = 0;
	int lastObjectDetection = -100;

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

	printLaserPoints();

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

void Pioneer::askIfOk() { // done - verified
	time_t startTime = time(0);
	bool handDetected = false;

	while (!handDetected && (time(0) - startTime) < 10) {
		robot->Read();
		for (int i = 0; i < sonarCount; i++) {
			if (0.1 < sonar->GetScan(i) && sonar->GetScan(i) < HAND_THRESHOLD) {
				handDetected = true;
			}
			if (0.1 < sonar->GetScan(i) && sonar->GetScan(i) < 0.5) {
				cout << "Sonar " << i << ": " << sonar->GetScan(i) << endl;
			}
		}
	}

	if (handDetected) {
		output("I'm glad you're ok!");
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
	position->SetSpeed(0, 0); //otherwise it will continue while it waits
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
	dialogue.push(text); //first
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

void Pioneer::printLaserPoints() {
	robot->Read();
	for (int i = 0; i < laserCount; i++) {
		player_point_2d p = getLaserPoint(i);
		cout << p.px << "\t" << p.py << endl;
	}
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
