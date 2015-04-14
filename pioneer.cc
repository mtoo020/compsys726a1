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
//			glVertex3d(-pose.py, pose.px, -5);
//			glVertex3d(-pose.py - g_Sonar->GetScan(i) * sin(pose.pyaw),
//					pose.px + g_Sonar->GetScan(i) * cos(pose.pyaw), -5);
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

//	laser->RequestGeom();
//	sonar->RequestGeom();
//	g_Laser = laser;
//	g_Sonar = sonar;

	position->SetMotorEnable(true);
	robot->Read();
	robot->Read();
	expectedYaw = position->GetYaw();
	cout << "expected yaw: " << expectedYaw << endl;

	laserCount = laser->GetCount();
	LASER_LEFT = laserCount - 1;
	LASER_NW = 3 * laserCount / 4;
	LASER_FRONT_LEFT = laserCount / 2;
	LASER_FRONT_RIGHT = laserCount / 2 + 1;
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

void Pioneer::turn(double angle) {
	int direction = angle > 0 ? DIRECTION_LEFT : DIRECTION_RIGHT; //had problems with turning where the yaw would continue in previous direction causing angleIsBetween to fail (angle outside of start and target)
	double speed = FAST; //SLOW didn't work on sneezy in RCL
	double startYaw = position->GetYaw();
	double targetYaw = fmod(expectedYaw + angle, radians(360));

	//while (angleDiff(startYaw, position->GetYaw()) < abs(angle)); use only with position->GetYaw() not expectedYaw
	while (angleIsBetween(startYaw, position->GetYaw(), targetYaw, direction)) {
		robot->Read();
		if (angleDiff(targetYaw, position->GetYaw()) < BIG_ANGLE_GAP) {
			speed = SLOW;
		}
		position->SetSpeed(0, speed * direction);
	}
	expectedYaw = targetYaw;
	cout << "overturn: " << angleDiff(position->GetYaw(), expectedYaw) << endl;
}

void Pioneer::drive(bool checkForRooms = false) {
	double speed = FAST;
	double previousRange = laser->GetRange(LASER_RIGHT);
	double roomStartPosition = 0;

	while (getFrontLaserRange() > GAP) {
		robot->Read();

		if (checkForRooms) {
			double increase = laser->GetRange(LASER_RIGHT) - previousRange;
			double decrease = -increase;

			if (abs(increase) > 0.1) {
				cout << increase << endl;
			}

			if (ROOM_THRESHOLD < increase) {
				roomStartPosition = position->GetXPos();
				speed = SLOW;

				cout << "room start " << increase << endl;
				output("Room found - analysing content");
			} else if (ROOM_THRESHOLD < decrease) {
				double roomEndPosition = position->GetXPos();

				drive((roomStartPosition - roomEndPosition) / 2);
				turn(radians(-90));
				analyseRoom();
				turn(radians(90));
				drive((roomEndPosition - roomStartPosition) / 2);
				cout << "room end " << decrease << endl;
			}
			previousRange = laser->GetRange(LASER_RIGHT);
		}

		if (getFrontLaserRange() < BIG_GAP) {
			speed = SLOW;
		}
		position->SetSpeed(speed, 0);
	}

	robot->Read();
}

void Pioneer::drive(double distance) {
	int direction = distance > 0 ? DIRECTION_FORWARD : DIRECTION_BACKWARD;
	double speed = FAST;
	double startPosition = position->GetXPos();

	while (abs(position->GetXPos() - startPosition) < abs(distance)) {
		robot->Read();
		if (getFrontLaserRange() < BIG_GAP) {
			speed = SLOW;
		}
		position->SetSpeed(speed * direction, 0);
	}
}

void Pioneer::analyseRoom() {
	int objectsFound = 0;

	robot->Read();
	for (int i = LASER_NW; i < LASER_NE; i++) {
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
		output("A pair of legs");
		askIfOk();
		break;
	default:
		output("Not sure");
		break;
	}
}

void Pioneer::askIfOk() {
	time_t startTime = time(0);
	double referenceScans[sonarCount];
	bool handDetected = false;

	robot->Read();
	for (int i = 0; i < sonarCount; i++) {
		referenceScans[i] = sonar->GetScan(i);
	}

	while (!handDetected && (time(0) - startTime) < 10) {
		robot->Read();
		for (int i = 0; i < sonarCount; i++) {
			if (abs(sonar->GetScan(i) - referenceScans[i]) > HAND_THRESHOLD) {
				handDetected = true;
			}
		}
	}

	if (handDetected) {
		output("Got a response. Thanks human!");
	} else {
		output("No response! Help!");
	}

}

int Pioneer::getClosestLaser() {
	int closestLaser = 0;
	double minDistance = laser->GetRange(0);

	robot->Read();
	for (int i = 0; i < laserCount; i++) {
		if (laser->GetRange(i) < minDistance) {
			closestLaser = i;
			minDistance = laser->GetRange(i);
		}
	}
	return closestLaser;
}

void Pioneer::moveToStartingCorner() {
	int closestFrontLaser = getClosestLaser();
	double frontDistance = laser->GetRange(closestFrontLaser);
	double frontAngle = laser->GetBearing(closestFrontLaser);

	turn(radians(180));

	int closestBackLaser = getClosestLaser();
	double backDistance = laser->GetRange(closestBackLaser);
	double backAngle = laser->GetBearing(closestBackLaser);

	if (frontDistance < backDistance) {
		turn(radians(180) + frontAngle);
	} else {
		turn(backAngle);
	}
	drive();
	turn(radians(90));
	drive();
}

void Pioneer::run() {
	output("Moving to a corner to start from");
	moveToStartingCorner();

	output("Starting search");
	for (int i = 0; i < 4; i++) {
		turn(radians(90));
		drive(true);
	}
	output("Search complete");
	position->SetSpeed(0, 0);
}

void Pioneer::output(string text) {
	cout << text << endl;
//	speech->Say(text);
//	thread t([this, text]{this->speech->Say(text);});
////	t.join();
}

double Pioneer::getFrontLaserRange() {
	return (laser->GetRange(LASER_FRONT_LEFT) + laser->GetRange(LASER_FRONT_RIGHT)) / 2;
}

bool Pioneer::angleIsBetween(double start, double angle, double target, int direction) {
//	cout << start << " " << angle << " " << target << " " << direction << endl;
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
