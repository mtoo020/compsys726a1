#include "pioneer.h"

LaserProxy* g_Laser;
SonarProxy* g_Sonar;

void showLaserAndSonar() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glBegin(GL_LINES);

	if (g_Laser) {
		glColor3f(0, 1, 0);
		int n = g_Laser->GetCount();
		for (int i = 0; i < n; i++) {
//			player_pose3d_t pose = g_Laser->GetPose();
			glVertex3d(0, 0, -5);
			glVertex3d(0.2 * g_Laser->GetRange(i) * cos(i * M_PI / n), 0.2 * g_Laser->GetRange(i) * sin(i * M_PI / n),
					-5);
		}
	}

	if (g_Sonar) {
		glColor3f(1, 1, 1);
		int n = g_Sonar->GetCount();
		for (int i = 0; i < n; i++) {
			player_pose3d_t pose = g_Sonar->GetPose(i);
			glVertex3d(-pose.py, pose.px, -5);
			glVertex3d(-pose.py - 0.2 * g_Sonar->GetScan(i) * sin(pose.pyaw),
					pose.px + 0.2 * g_Sonar->GetScan(i) * cos(pose.pyaw), -5);
		}
	}

	glEnd();
	glutSwapBuffers();
}

Pioneer::Pioneer(int argc, char **argv) {
	parse_args(argc, argv);

	robot = new PlayerClient(gHostname, gPort);
	position = new Position2dProxy(robot, gIndex);
	laser = new LaserProxy(robot, gIndex);
	sonar = new SonarProxy(robot, gIndex);
	//	speech = new SpeechProxy(&robot, gIndex); not available

//	laser->RequestGeom();
	sonar->RequestGeom();
	g_Laser = laser;
	g_Sonar = sonar;

	position->SetMotorEnable(true);
	robot->Read();
	robot->Read();

	//correct - at least on the virtual pioneer
	laserCount = laser->GetCount();
	LASER_LEFT = laserCount - 1;
	LASER_FRONT = laserCount / 2;
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

void Pioneer::printLaser() {
	cout << "Laser left: " << laser->GetRange(LASER_LEFT) << "    front: " << laser->GetRange(LASER_FRONT)
			<< "    right: " << laser->GetRange(LASER_RIGHT) << endl << endl;
}

void Pioneer::printSonar() {
	cout << "Sonar left: " << sonar->GetScan(SONAR_LEFT_FRONT) << " " << sonar->GetScan(SONAR_LEFT_BACK)
			<< "    front: " << sonar->GetScan(SONAR_FRONT_LEFT) << " " << sonar->GetScan(SONAR_FRONT_RIGHT)
			<< "    right: " << sonar->GetScan(SONAR_RIGHT_FRONT) << " " << sonar->GetScan(SONAR_RIGHT_BACK)
			<< "    back: " << sonar->GetScan(SONAR_BACK_LEFT) << " " << sonar->GetScan(SONAR_BACK_RIGHT) << endl
			<< endl;
}

double Pioneer::getTrueSonarAngle(int index) {
	return M_PI_2 + sonar->GetPose(index).pyaw;
}

int Pioneer::getClosestLaser() {
	int closestLaser = 0;
	double minDistance = 100;
	for (int i = 0; i < laserCount; i++) {
		if (laser->GetRange(i) < minDistance) {
			closestLaser = i;
			minDistance = laser->GetRange(i);
		}
	}
	return closestLaser;
}
double Pioneer::getLaserAngleError(int threshold) {
	int closestLaser = 0;
	double minDistance = 100;
	for (int i = LASER_FRONT - threshold; i < LASER_FRONT + threshold; i++) {
		if (laser->GetRange(i) < minDistance) {
			closestLaser = i;
			minDistance = laser->GetRange(i);
		}
	}
	double pn = laser->GetBearing(closestLaser) - laser->GetBearing(LASER_FRONT);
	cout << pn << endl;
	return laser->GetBearing(closestLaser) - laser->GetBearing(LASER_FRONT);
}

int Pioneer::getClosestSonar() {
	int closestSonar = 0;
	double minDistance = 100;
	for (int i = 0; i < sonarCount; i++) {
		if (sonar->GetScan(i) < minDistance) {
			closestSonar = i;
			minDistance = sonar->GetScan(i);
		}
	}
	return closestSonar;
}

int Pioneer::getTurningDirection() {
	int closestSonar = getClosestSonar();
	return SONAR_FRONT_RIGHT < closestSonar && closestSonar < SONAR_BACK_RIGHT ? DIRECTION_RIGHT : DIRECTION_LEFT;
}

double Pioneer::absDiff(double a, double b) {
	return abs(abs(a) - abs(b));
}

void Pioneer::turnToNearestWall(int direction) {
	double closestSonarAngle = getTrueSonarAngle(getClosestSonar());
	double turningSpeed = FAST;

	cout << "turnToNearestWall" << endl;
	while (absDiff(position->GetYaw(), closestSonarAngle) > ANGLE) {
		robot->Read();

		if (absDiff(position->GetYaw(), closestSonarAngle) < BIG_ANGLE) {
			turningSpeed = SLOW;
		}
		position->SetSpeed(0, turningSpeed * direction);
	}
}

void Pioneer::turn90(int direction) {
	double targetAngle = position->GetYaw() + direction * M_PI_2 + getLaserAngleError(10);
	double turningSpeed = FAST;

	if (targetAngle > M_PI) {
		targetAngle = fmod(targetAngle, M_PI) - M_PI;
	}

	while (absDiff(targetAngle, position->GetYaw()) > ANGLE) {
		robot->Read();

		if (absDiff(targetAngle, position->GetYaw()) < BIG_ANGLE) {
			turningSpeed = SLOW;
		}
		position->SetSpeed(0, turningSpeed * direction);
	}
}

void Pioneer::driveToWall() {
	double speed = FAST;

	cout << "driveToWall" << endl;
	while (laser->GetRange(LASER_FRONT) > GAP) {
		robot->Read();

		if (laser->GetRange(LASER_FRONT) < BIG_GAP) {
			speed = SLOW;
		}
		position->SetSpeed(speed, 0);
	}

	cornersCompleted++;
}

void Pioneer::run() {
	int turningDirection = getTurningDirection();

	turnToNearestWall(turningDirection);
	driveToWall();

	while (cornersCompleted < 4) {
		turn90(-turningDirection);
		driveToWall();
	}
}

////		while (true) {
////			driveToNearestWall();
////			driveAlongWall();
////			if (obstacleInFront()) {
////				if (voidOnLeft()) {
////					turnLeft();
////				} else if (voidOnRight()) {
////					turnRight();
////				}
////			} else if (voidOnLeft()) {
////				turnLeft();
////			} else if (voidOnRight()) {
////				turnRight();
////			}
////			pp.SetSpeed(1, 0);
////		}
////	while (true) {
////		robot.Read();
////		pp.SetSpeed(0, 1);
////		//			cout << lp[0] << " " << lp[128] << " " << lp[256] << " " << lp[384] << endl;
////		//			if (0 < lp[255] && lp[255] < 0.1) {
////		//				pp.SetSpeed(0, 1);
////		//				cout << lp[0] << endl;
////		//				robot.Stop();
////		//				return 0;
////		//			}
////	}
//
//	//		while (true) {
//	//			robot.Read();
//	//			pp.SetSpeed(0.1, 0);
//	//			cout << sp[0] << " " << sp[4] << " " << sp[8] << " " << sp[12]
//	//					<< endl;
//	//			for (int i=0;i<16;i++) {
//	//				if (0 < sp[i] && sp[i] < 0.3) {
//	//					cout << i << endl;
//	//				}
//	//			}
//	////			if (0 < sp[0] && sp[0] < 0.3) {
//	////				cout << "l" << endl;
//	////				robot.Stop();
//	////				return 0;
//	////			}
//	////			if (0 < sp[4] && sp[4] < 0.3) {
//	////				cout << "f" << endl;
//	////				robot.Stop();
//	////				return 0;
//	////			}
//	////			if (0 < sp[8] && sp[8] < 0.3) {
//	////				cout << "r" << endl;
//	////				robot.Stop();
//	////				return 0;
//	////			}
//	////			if (0 < sp[12] && sp[12] < 0.3) {
//	////				cout << "b" << endl;
//	////				robot.Stop();
//	////				return 0;
//	////			}
//	//		}
//}

//Called when the window is resized
void handleResize(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double) w / (double) h, 1.0, 200.0);
}

void showWindow(int argc, char **argv) {
	glutInit(&argc, argv);
	glutInitWindowSize(400, 400);
	glutCreateWindow("Radar and Sonar Visualisation");
	glutDisplayFunc(showLaserAndSonar);
	glutReshapeFunc(handleResize);
	glutIdleFunc(glutPostRedisplay);
	glutMainLoop();
}

int main(int argc, char **argv) {
//	thread t(showWindow, argc, argv);

	try {
		Pioneer pioneer(argc, argv);
		pioneer.run();
	} catch (PlayerError *e) {
		cout << e->GetErrorCode() << " " << e->GetErrorStr() << " " << e->GetErrorFun() << endl;
		return -1;
	}
//	t.join();
}
