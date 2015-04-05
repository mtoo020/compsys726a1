#include "pioneer.h"

LaserProxy* g_Laser;
SonarProxy* g_Sonar;

//next steps
//refer to assignment sheet

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

//Called when the window is resized
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

void Pioneer::printLaser() {
	cout << "Laser left: " << laser->GetRange(LASER_LEFT) << "    front: " << laser->GetRange(LASER_FRONT_LEFT) << " "
			<< laser->GetRange(LASER_FRONT_RIGHT) << "    right: " << laser->GetRange(LASER_RIGHT) << endl << endl;
}

void Pioneer::printSonar() {
	cout << "Sonar left: " << sonar->GetScan(SONAR_LEFT_FRONT) << " " << sonar->GetScan(SONAR_LEFT_BACK)
			<< "    front: " << sonar->GetScan(SONAR_FRONT_LEFT) << " " << sonar->GetScan(SONAR_FRONT_RIGHT)
			<< "    right: " << sonar->GetScan(SONAR_RIGHT_FRONT) << " " << sonar->GetScan(SONAR_RIGHT_BACK)
			<< "    back: " << sonar->GetScan(SONAR_BACK_LEFT) << " " << sonar->GetScan(SONAR_BACK_RIGHT) << endl
			<< endl;
}

double Pioneer::absDiff(double a, double b) {
	return abs(abs(a) - abs(b));
}

double Pioneer::getFrontLaserRange() {
	return (laser->GetRange(LASER_FRONT_LEFT) + laser->GetRange(LASER_FRONT_RIGHT)) / 2;
}

double Pioneer::getLaserAngleError(int threshold) {
	int closestLaser = 0;
	double minDistance = 100;
	for (int i = LASER_FRONT_LEFT - threshold; i <= LASER_FRONT_RIGHT + threshold; i++) {
		if (laser->GetRange(i) < minDistance) {
			closestLaser = i;
			minDistance = laser->GetRange(i);
		}
	}
	return laser->GetBearing(closestLaser);
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

void Pioneer::turn(double angle, bool checkFrontLasers = true) {
	int direction = angle > 0 ? DIRECTION_LEFT : DIRECTION_RIGHT;
	double turningSpeed = FAST;
	double targetAngle = position->GetYaw() + angle;

	if (checkFrontLasers) {
		targetAngle += getLaserAngleError(FRONT_LASER_THRESHOLD);
	}

	if (targetAngle < -M_PI) {
		targetAngle += 2 * M_PI;
	} else if (M_PI < targetAngle) {
		targetAngle -= 2 * M_PI;
	}

	while (absDiff(targetAngle, position->GetYaw()) > ANGLE_GAP) {
		robot->Read();

		if (absDiff(targetAngle, position->GetYaw()) < BIG_ANGLE_GAP) {
			turningSpeed = SLOW;
		}
		position->SetSpeed(0, turningSpeed * direction);
	}
}

void Pioneer::drive() {
	double speed = FAST;

	while (getFrontLaserRange() > GAP) {
		robot->Read();

		if (getFrontLaserRange() < BIG_GAP) {
			speed = SLOW;
		}
		position->SetSpeed(speed, 0);
	}
}

void Pioneer::run() {
	//get to starting corner
	turn(sonar->GetPose(getClosestSonar()).pyaw, false);
	drive();
	turn(M_PI_2);
	drive();

	for (int wallsCompleted = 0; wallsCompleted < 100; wallsCompleted++) {
		turn(M_PI_2);
		drive();
	}
	position->SetSpeed(0, 0);
}

int main(int argc, char **argv) {
	try {
		Pioneer pioneer(argc, argv);
		pioneer.run();
	} catch (PlayerError *e) {
		cout << e->GetErrorCode() << " " << e->GetErrorStr() << " " << e->GetErrorFun() << endl;
		return -1;
	}
//	glutInit(&argc, argv);
//	glutInitWindowSize(400, 400);
//	glutCreateWindow("Radar and Sonar Visualisation");
//	glutDisplayFunc(showLaserAndSonar);
//	glutReshapeFunc(handleResize);
//	glutIdleFunc(glutPostRedisplay);
//	glutMainLoop();
}
