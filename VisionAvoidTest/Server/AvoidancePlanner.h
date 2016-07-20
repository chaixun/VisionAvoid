#ifndef AVOIDANCEPLANNER_H
#define AVOIDANCEPLANNER_H

#include <math.h>
#include <iostream>
#include "RobObsPose.h"
#include "Eigen/Dense"

using namespace std;

using namespace Eigen;

using namespace RobObsPose;

class AvoidancePlanner
{
public:
    struct Point3D
    {
        double x;
        double y;
		double z;
    };

    enum PLANNER_STATE
    {
        GENBODTANDFEETPOS = 1,
        NOTSTART = 1,
        GENBODYANDFEETFINISHED = 2,
        GAITSTART = 3,
		GAITEND = 4,
    };

	AvoidancePlanner();
    ~AvoidancePlanner();

    void ChangeGenPoseState();
	void GenBodyandFeetPose();
	void StartGait(int timeNow);
	int OutBodyandFeetTraj(double bodyPose[6], double feetPosi[18], int timeNow);
	PLANNER_STATE GetPlannerState() const { return plannerState; }
	void ClearValues();

private:
    PLANNER_STATE plannerState = GAITEND;
	double oneStepLength = 0.3;
    double halfStep = 0.18;
    int halfStepT = 2500;
    double deltaT = 0.001;
	vector<double> timeSeg;
    vector<RobPose> bodyPoses;
    vector<FootHold> feetPoses;

    int timeStart = 0;
    bool leftSwing = false;

	double detectRange = 0.65 + 1;
	double radiusVirtObs = 0.3;
	double safeMargin = 0.2;
	double dualDist = safeMargin + 0.45 + 0.45 + safeMargin;
	double safeDist = safeMargin + 0.45;
	double lRobot = 0.275;
	double rRobot = 0.5274;
	double dSafe = 0.05;

	Point3D P, Q, R, S;
	ObsPose nowLeftObsPose, nextLeftObsPose, nowRightObsPose, nextRightObsPose;

	void GenAvoidPath();
	void FindSafePose();
	double CalCostFunction(Point3D, Point3D, Point3D, Point3D);

	void GenBodyPoses();
	void GenFeetHolds();
	void GenFootTraj(double feetTrajPosi[18], int timeCount, int numCycle);
};

#endif // AVOIDANCEPLANNER_H
