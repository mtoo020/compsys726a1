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
//	speech = new SpeechProxy(robot, gIndex);

//	laser->RequestGeom();
//	sonar->RequestGeom();
//	g_Laser = laser;
//	g_Sonar = sonar;

	position->SetMotorEnable(true);
	robot->Read();
	robot->Read();

	//correct - at least on the virtual pioneer
	laserCount = laser->GetCount();
	LASER_LEFT = laserCount - 1;
	LASER_FRONT_LEFT = laserCount / 2;
	LASER_FRONT_RIGHT = laserCount / 2 + 1;
	LASER_RIGHT = 0;

	//correct - at least on the virtual pioneer
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

void Pioneer::turn(double targetYaw) {
	double turningSpeed = FAST;
	int direction = targetYaw > 0 ? DIRECTION_LEFT : DIRECTION_RIGHT;

	while (abs(angleDiff(targetYaw, position->GetYaw())) > ANGLE_GAP) {
		robot->Read();
		if (abs(angleDiff(targetYaw, position->GetYaw())) < BIG_ANGLE_GAP) {
			turningSpeed = SLOW;
		}
		position->SetSpeed(0, turningSpeed * direction);
	}
}

void Pioneer::drive(bool checkForRooms = false) {
	double speed = FAST;
	double previousRange = laser->GetRange(LASER_RIGHT);
	bool roomFound = false;
	bool objectFound = false;
	int objectsFound = 0;
	int n = 0;

	while (getFrontLaserRange() > GAP) {
		robot->Read();

		if (checkForRooms) {
			double increase = laser->GetRange(LASER_RIGHT) - previousRange;
			double decrease = -increase;

			if (abs(increase) > 0.1) {

				cout << increase << endl;
			}

			if (!roomFound && ROOM_THRESHOLD < increase) {
				roomFound = true;
				speed = SLOW;
				objectsFound = 0;

				cout << "room start " << increase << endl;
				output("Room found - analysing content");
			} else if (roomFound) {

				if (!objectFound && OBJECT_THRESHOLD < decrease && decrease < ROOM_THRESHOLD) {
					objectFound = true;
					cout << "object start " << decrease << endl;

				} else if (objectFound && OBJECT_THRESHOLD < increase && increase < ROOM_THRESHOLD) {
					objectsFound++;
					objectFound = false;
					cout << "object end " << increase << endl;

				} else if (ROOM_THRESHOLD < decrease) {
					roomFound = false;
					speed = FAST;
					cout << "room end " << decrease << endl;

					switch (objectsFound) {
					case 0:
						output("Nothing");
						//speak
						break;
					case 1:
						output("An object");
						break;
					case 2:
						output("A pair of legs");
						position->SetSpeed(0, 0);
						askIfOk();
						break;
					default:
						output("Not sure");
						break;
					}
					cout << endl;

				}
			}
			if (n % 1 == 0) {
				previousRange = laser->GetRange(LASER_RIGHT);
			}
			n++;
		}

		if (getFrontLaserRange() < BIG_GAP) {
			speed = SLOW;
		}
		position->SetSpeed(speed, 0);
	}

	robot->Read();
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
			if (abs(sonar->GetScan(i)-referenceScans[i]) > HAND_THRESHOLD) {
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

double Pioneer::getClosestFrontLaserAngle() {
	int closestLaser = 0;
	double minDistance = 100;
	for (int i = LASER_FRONT_LEFT - FRONT_LASER_THRESHOLD; i < LASER_FRONT_RIGHT + FRONT_LASER_THRESHOLD; i++) {
		if (laser->GetRange(i) < minDistance) {
			closestLaser = i;
			minDistance = laser->GetRange(i);
		}
	}
	return laser->GetBearing(closestLaser);
}

double Pioneer::getClosestLaserAngle() {
	int closestLaser = 0;
	double minDistance = 100;
	bool needToTurnBack = true;

	for (int i = 0; i < laserCount; i++) {
		if (laser->GetRange(i) < minDistance) {
			closestLaser = i;
			minDistance = laser->GetRange(i);
		}
	}

	turn(radians(180));
	robot->Read();

	for (int i = 0; i < laserCount; i++) {
		if (laser->GetRange(i) < minDistance) {
			needToTurnBack = false;
			closestLaser = i;
			minDistance = laser->GetRange(i);
		}
	}

	if (needToTurnBack) {
		turn(radians(180));
	}

	cout << "Closest laser: " << laser->GetBearing(closestLaser) << endl;
	return laser->GetBearing(closestLaser);
}

double Pioneer::getClosestSonarAngle() {
	int closestSonar = 0;
	double minDistance = 100;
	for (int i = 0; i < sonarCount; i++) {
		if (sonar->GetScan(i) < minDistance) {
			closestSonar = i;
			minDistance = sonar->GetScan(i);
		}
	}
	return sonar->GetPose(closestSonar).pyaw;
}

void Pioneer::run() {
	output("Moving to a corner to start from");
//	turn(getClosestLaserAngle());
//	drive();
//	turn(M_PI_2);
//	drive();

	output("Starting search");
	while (true) {
		for (int i = 1; i <= 4; i++) {
			drive(true);
			turn(i * radians(90) - TURNING_ERROR);
		}
	}
	position->SetSpeed(0, 0);
	output("Search complete");
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

double Pioneer::angleDiff(double a, double b) {
	//needs to be checked in stage
	double d = a - b;
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
