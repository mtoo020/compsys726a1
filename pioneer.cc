#include "pioneer.h"

SonarProxy* g_Sonar;
LaserProxy* g_Laser;

void displaySonar() {
	//Clear information from last draw
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW); //Switch to the drawing perspective
	glLoadIdentity(); //Reset the drawing perspective

	glBegin(GL_LINES);

	if (g_Sonar) {
		int n = g_Sonar->GetCount();
		for (int i = 0; i < n; i++) {
			glVertex3f(0, 0, -5);
			glVertex3f(-0.2 * g_Sonar->GetScan(i) * cos(i * 2 * M_PI / n), 0.2 * g_Sonar->GetScan(i) * sin(i * 2 * M_PI / n), -5);
		}
	}

	glEnd(); //End triangle coordinates

	glutSwapBuffers(); //Send the 3D scene to the screen
}

Pioneer::Pioneer(int argc, char **argv) {
	parse_args(argc, argv);

	robot = new PlayerClient(gHostname, gPort);
	position = new Position2dProxy(robot, gIndex);
	sonar = new SonarProxy(robot, gIndex);
	laser = new LaserProxy(robot, gIndex);

	g_Sonar = sonar;
	g_Laser = laser;

//	speech = new SpeechProxy(&robot, gIndex); not available

	position->SetMotorEnable(true);
	robot->Read();
	robot->Read();

	laserCount = laser->GetCount();
	LASER_LEFT = laserCount - 1;
	LASER_FRONT = laserCount / 2;
	LASER_RIGHT = 0;

	sonarCount = sonar->GetCount();
	SONAR_LEFT = 2 * sonarCount / 3;
	SONAR_FRONT = sonarCount / 3;
	SONAR_RIGHT = 0;
	SONAR_BACK = sonarCount - 1;
}

void Pioneer::printLaser() {
	cout << "Laser left: " << laser->GetRange(LASER_LEFT) << "    front: "
			<< laser->GetRange(LASER_FRONT) << "    right: "
			<< laser->GetRange(LASER_RIGHT) << endl << endl;
}

void Pioneer::printSonar() {
	cout << "Sonar left: " << sonar->GetScan(SONAR_LEFT) << "    front: "
			<< sonar->GetScan(SONAR_FRONT) << "    right: "
			<< sonar->GetScan(SONAR_RIGHT) << "    back: "
			<< sonar->GetScan(SONAR_BACK) << endl << endl;
}

void Pioneer::run() {
	while (true) {
		robot->Read();
		printSonar();
		printLaser();
		position->SetSpeed(-0.1, 0);
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
	glutCreateWindow("Sonar visualisation");
	glutDisplayFunc(displaySonar);
	glutReshapeFunc(handleResize);
	glutIdleFunc(glutPostRedisplay);
	glutMainLoop();
}

int main(int argc, char **argv) {
	thread t(showWindow, argc, argv);

	try {
		Pioneer pioneer(argc, argv);
		pioneer.run();
	} catch (PlayerError *e) {
		cout << e->GetErrorCode() << " " << e->GetErrorStr() << " "
				<< e->GetErrorFun() << endl;
		return -1;
	}
	t.join();
}
