#include <libplayerc++/playerc++.h>
#include <iostream>
#include <stdlib.h>
#include "args.h"

using namespace PlayerCc;
using namespace std;

template <typename T>
void print(T arg) {
  cout << arg << endl;
}

int main(int argc, char **argv) {
	parse_args(argc, argv);
	try {
		PlayerClient robot(gHostname, gPort);
		Position2dProxy pp(&robot, gIndex);
//		SonarProxy sp(&robot, gIndex);
		LaserProxy lp(&robot, gIndex);
//		SpeechProxy speech(&robot, gIndex);

		pp.SetMotorEnable(true);
		robot.Read();
		print(lp.GetCount());

//		while (true) {
//			driveToNearestWall();
//			driveAlongWall();
//			if (obstacleInFront()) {
//				if (voidOnLeft()) {
//					turnLeft();
//				} else if (voidOnRight()) {
//					turnRight();
//				}
//			} else if (voidOnLeft()) {
//				turnLeft();
//			} else if (voidOnRight()) {
//				turnRight();
//			}
//			pp.SetSpeed(1, 0);
//		}
		robot.Read();
		robot.Read();
		robot.Read();
		while (true) {
			robot.Read();
			pp.SetSpeed(0,1);
//			cout << lp[0] << " " << lp[128] << " " << lp[256] << " " << lp[384] << endl;
//			if (0 < lp[255] && lp[255] < 0.1) {
//				pp.SetSpeed(0, 1);
//				cout << lp[0] << endl;
//				robot.Stop();
//				return 0;
//			}
		}

//		while (true) {
//			robot.Read();
//			pp.SetSpeed(0.1, 0);
//			cout << sp[0] << " " << sp[4] << " " << sp[8] << " " << sp[12]
//					<< endl;
//			for (int i=0;i<16;i++) {
//				if (0 < sp[i] && sp[i] < 0.3) {
//					cout << i << endl;
//				}
//			}
////			if (0 < sp[0] && sp[0] < 0.3) {
////				cout << "l" << endl;
////				robot.Stop();
////				return 0;
////			}
////			if (0 < sp[4] && sp[4] < 0.3) {
////				cout << "f" << endl;
////				robot.Stop();
////				return 0;
////			}
////			if (0 < sp[8] && sp[8] < 0.3) {
////				cout << "r" << endl;
////				robot.Stop();
////				return 0;
////			}
////			if (0 < sp[12] && sp[12] < 0.3) {
////				cout << "b" << endl;
////				robot.Stop();
////				return 0;
////			}
//		}

	} catch (PlayerError *e) {
		cout << e->GetErrorCode() << " " << e->GetErrorStr() << " " << e->GetErrorFun() << endl;
		return -1;
	}
}
