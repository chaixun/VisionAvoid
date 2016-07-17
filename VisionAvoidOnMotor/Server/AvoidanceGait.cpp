#include "AvoidanceGait.h"

namespace Avoidance
{
double AvoidanceGaitWrapper::bodyPose[6] = { 0, 0, 0, 0, 0, 0 };

double AvoidanceGaitWrapper::feetPosi[18] =
{ -0.3,  -0.85, -0.65,
  -0.45, -0.85,  0,
  -0.3,  -0.85,  0.65,
  0.3,  -0.85, -0.65,
  0.45, -0.85,  0,
  0.3,   -0.85,  0.65 };

AvoidancePlanner AvoidanceGaitWrapper::avoidancePlanner;

void AvoidanceGaitWrapper::AvoidanceParse(const string &cmd, const map<string, string> &params, aris::core::Msg &msg)
{
    AvoidanceGaitParam param;
    msg.copyStruct(param);
}

int AvoidanceGaitWrapper::AvoidanceGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const AvoidanceGaitParam &>(param_in);

    int timeNow = param.count;
    static aris::dynamic::FloatMarker beginMark{ robot.ground() };

    if (nowRobPose.x < 9 && avoidancePlanner.GetPlannerState() == AvoidancePlanner::GAITEND)
    {
        avoidancePlanner.GenBodyandFeetPose();
        avoidancePlanner.StartGait(timeNow);
    }

    if (avoidancePlanner.GetPlannerState() == AvoidancePlanner::GAITSTART)
    {
        avoidancePlanner.OutBodyandFeetTraj(bodyPose, feetPosi, timeNow);
    }

    robot.SetPeb(bodyPose, beginMark);
    robot.SetPee(feetPosi, beginMark);


    if (nowRobPose.x >= 9 && avoidancePlanner.GetPlannerState() == AvoidancePlanner::GAITEND)
    {
        avoidancePlanner.ClearValues();
        return 0;
    }
}
}
