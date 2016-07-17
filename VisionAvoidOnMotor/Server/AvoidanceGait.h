#ifndef AVOIDANCEGAIT_H
#define AVOIDANCEGAIT_H

#include <iostream>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>
#include "AvoidancePlanner.h"

using namespace std;

namespace Avoidance
{
	struct AvoidanceGaitParam final: public aris::server::GaitParamBase
	{
	};

	class AvoidanceGaitWrapper
	{
	public:
		static double bodyPose[6];
		static double feetPosi[18];
		static AvoidancePlanner avoidancePlanner;

        static auto AvoidanceParse(const string &cmd, const map<string, string> &param, aris::core::Msg &msg) -> void;
        static auto AvoidanceGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in) -> int;
	};

	static AvoidanceGaitWrapper wrapper;
}

#endif // AVOIDANCEGAIT_H
