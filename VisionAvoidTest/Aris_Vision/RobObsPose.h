#ifndef ROBOBSPOSE_H
#define ROBOBSPOSE_H

#include <vector>

using namespace std;

namespace RobObsPose
{
	struct RobPose
	{
		double x;
		double y;
		double z;
		double alpha;
		double beta;
		double gama;
	};

	struct FootHold
	{
		double feetHold[18];
	};

	struct ObsPose
	{
		double x;
		double y;
		double r;
		double h;
	};

	extern vector<ObsPose> obsPoses;
	extern RobPose nowRobPose;
	extern RobPose nextRobPose;
}
#endif // ROBOBSPOSE_H
