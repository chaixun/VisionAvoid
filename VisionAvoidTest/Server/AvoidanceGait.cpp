#include "AvoidanceGait.h"

namespace Avoidance
{

aris::sensor::KINECT AvoidanceGaitWrapper::kinect1;
aris::control::Pipe<int> AvoidanceGaitWrapper::visionPipe(true);
std::thread AvoidanceGaitWrapper::visionThread;

TerrainAnalysis AvoidanceGaitWrapper::terrainAnalysisResult;
ObstacleDetection AvoidanceGaitWrapper::obstacleDetectionResult;

atomic_bool AvoidanceGaitWrapper::isMapAnalysisFinished(false);
atomic_bool AvoidanceGaitWrapper::isSending(false);
atomic_bool AvoidanceGaitWrapper::isStop(false);

double AvoidanceGaitWrapper::bodyPose[6] = { 0, 0, 0, 0, 0, 0 };

double AvoidanceGaitWrapper::feetPosi[18] =
{ -0.3,  -0.85, -0.65,
  -0.45, -0.85,  0,
  -0.3,  -0.85,  0.65,
  0.3,  -0.85, -0.65,
  0.45, -0.85,  0,
  0.3,   -0.85,  0.65 };

AvoidancePlanner AvoidanceGaitWrapper::avoidancePlanner;

void AvoidanceGaitWrapper::KinectStart()
{
    kinect1.start();

    visionThread = std::thread([]()
    {
        while(true)
        {
            int a;
            visionPipe.recvInNrt(a);
            auto visiondata = kinect1.getSensorData();

            obstacleDetectionResult.ObstacleDetecting(visiondata.get().obstacleMap, nowRobPose);

            if(obstacleDetectionResult.tempobsPoses.size() > 0)
            {
                if(obsPoses.size() == 0)
                {
                    obsPoses.push_back(obstacleDetectionResult.tempobsPoses[0]);
                }
                else if(fabs(obsPoses.back().x - obstacleDetectionResult.tempobsPoses[0].x) > obsPoses.back().r
                        ||fabs(obsPoses.back().y - obstacleDetectionResult.tempobsPoses[0].y) > obsPoses.back().r)
                {
                    obsPoses.push_back(obstacleDetectionResult.tempobsPoses[0]);
                }
            }

            for(int i = 0; i < obsPoses.size(); i++)
            {
                cout<<"Obs "<< i <<" Pos: x:"<<obsPoses[i].x<<" y:"<<obsPoses[i].y<<" radius:"<<obsPoses[i].r<<endl;
            }

            isMapAnalysisFinished = true;

            cout<<"avoidAnalysisFinished"<<endl;
        }
    });
}

void AvoidanceGaitWrapper::AvoidanceParse(const string &cmd, const map<string, string> &params, aris::core::Msg &msg)
{
    AvoidanceGaitParam param;
    msg.copyStruct(param);
}

void AvoidanceGaitWrapper::StopAvoidanceParse(const string &cmd, const map<string, string> &params, aris::core::Msg &msg)
{
    isStop = true;
}

int AvoidanceGaitWrapper::AvoidanceGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const AvoidanceGaitParam &>(param_in);

    int timeNow = param.count;
    static aris::dynamic::FloatMarker beginMark{ robot.ground() };

    if(isMapAnalysisFinished)
    {
        if (avoidancePlanner.GetPlannerState() == AvoidancePlanner::GENBODTANDFEETPOS)
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

        if(avoidancePlanner.GetPlannerState() == AvoidancePlanner::GAITEND)
        {
            if(nowRobPose.x >= 6 || isStop == true)
            {
                avoidancePlanner.ClearValues();
                isMapAnalysisFinished = false;
                return 0;
            }
            else
            {
                isMapAnalysisFinished = false;
                avoidancePlanner.ChangeGenPoseState();
            }
        }
    }
    else
    {
        if(isSending)
        {
            return -1;
        }
        else
        {
            visionPipe.sendToNrt(6);
            isSending = true;
            return -1;
        }
    }
}
}
