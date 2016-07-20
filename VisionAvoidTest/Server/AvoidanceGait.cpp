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

std::string fileName1 = "RobPose.txt";
std::ofstream robPoseFile(fileName1);
std::string fileName2 = "obsPose.txt";
std::ofstream obsPoseFile(fileName2);

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
            terrainAnalysisResult.TerrainAnalyze(visiondata.get().gridMap, visiondata.get().pointCloud);

            cout<<"nowRobPose: "<<nowRobPose.x<<" "<<nowRobPose.y<<" "<<nowRobPose.gama<<endl;

            if(robPoseFile.is_open())
            {
                robPoseFile << nowRobPose.x <<" "<< nowRobPose.y <<" "<< nowRobPose.gama <<std::endl;
            }

            obstacleDetectionResult.ObstacleDetecting(visiondata.get().obstacleMap, nowRobPose);

            if(obstacleDetectionResult.tempobsPoses.size() > 0)
            {
                if(obsPoses.size() == 0)
                {
                    obsPoses.push_back(obstacleDetectionResult.tempobsPoses[0]);

                    if(obsPoseFile.is_open())
                    {
                        obsPoseFile << obsPoses.back().x <<" "<< obsPoses.back().y <<" "<< obsPoses.back().r <<std::endl;
                    }

                }
                else if(fabs(obsPoses.back().x - obstacleDetectionResult.tempobsPoses[0].x) > obsPoses.back().r
                        ||fabs(obsPoses.back().y - obstacleDetectionResult.tempobsPoses[0].y) > obsPoses.back().r)
                {
                    obsPoses.push_back(obstacleDetectionResult.tempobsPoses[0]);

                    if(obsPoseFile.is_open())
                    {
                        obsPoseFile << obsPoses.back().x <<" "<< obsPoses.back().y <<" "<< obsPoses.back().r <<std::endl;
                    }

                }
            }

            for(int i = 0; i < obsPoses.size(); i++)
            {
                cout<<"Obs "<< i <<" Pos: x:"<<obsPoses[i].x<<" y:"<<obsPoses[i].y<<" radius:"<<obsPoses[i].r<<endl;
            }

            isMapAnalysisFinished = true;

            //   cout<<"avoidAnalysisFinished"<<endl;
        }
    });
}

auto AvoidanceGaitWrapper::AvoidanceParse(const string &cmd, const map<string, string> &params, aris::core::Msg &msg)->void
{
    AvoidanceGaitParam param;
    msg.copyStruct(param);
}

auto AvoidanceGaitWrapper::StopAvoidanceParse(const string &cmd, const map<string, string> &params, aris::core::Msg &msg)->void
{
    isStop = true;
}

auto AvoidanceGaitWrapper::AvoidanceGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    if(isMapAnalysisFinished)
    {
        auto &robot = static_cast<Robots::RobotBase &>(model);
        auto &param = static_cast<const AvoidanceGaitParam &>(param_in);

        int timeNow = param.count;
        static aris::dynamic::FloatMarker beginMark{ robot.ground() };

        if (avoidancePlanner.GetPlannerState() == AvoidancePlanner::GENBODTANDFEETPOS)
        {
            avoidancePlanner.GenBodyandFeetPose();
            avoidancePlanner.StartGait(timeNow);
            rt_printf("nextRobPose: ");
            rt_printf("%f %f %f \n", nextRobPose.x, nextRobPose.y, nextRobPose.gama);
        }

        if (avoidancePlanner.GetPlannerState() == AvoidancePlanner::GAITSTART)
        {
            avoidancePlanner.OutBodyandFeetTraj(bodyPose, feetPosi, timeNow);
        }

        robot.SetPeb(bodyPose, beginMark);
        robot.SetPee(feetPosi, beginMark);

        if(avoidancePlanner.GetPlannerState() == AvoidancePlanner::GAITEND)
        {
            rt_printf("nowRobPose: ");
            rt_printf("%f %f %f \n",nowRobPose.x, nowRobPose.y, nowRobPose.gama);

            // if(isStop || nowRobPose.x >= 9)
            if(nowRobPose.x >= 9)
            {
                rt_printf("nowRobPose1: ");
                rt_printf("%f \n",nowRobPose.x);
                //rt_printf("%d \n",isStop);
                avoidancePlanner.ClearValues();
                avoidancePlanner.ChangeGenPoseState();
                isSending = false;
                isMapAnalysisFinished = false;
                return 0;
            }
            else
            {
                // rt_printf("now Finished!!!\n");
                isSending = false;
                isMapAnalysisFinished = false;
                avoidancePlanner.ChangeGenPoseState();
            }
        }
    }
    else
    {
        if(isSending)
        {
            //            rt_printf("Sending !!!\n");
            return -1;
        }
        else
        {
            //            rt_printf("Begin Sending \n");
            visionPipe.sendToNrt(6);
            isSending = true;
            return -1;
        }
    }
}
}
