#ifndef AVOIDANCEGAIT_H
#define AVOIDANCEGAIT_H

#include <iostream>

#include <thread>
#include "aris.h"
#include <Robot_Gait.h>
#include <Robot_Base.h>
#include "Aris_Vision.h"
#include "Vision_Terrain0.h"
#include "Vision_ObstacleDetection.h"
#include "AvoidancePlanner.h"

using namespace aris::core;
using namespace std;

namespace Avoidance
{
struct AvoidanceGaitParam final: public aris::server::GaitParamBase
{
};

class AvoidanceGaitWrapper
{
public:

    static aris::sensor::KINECT kinect1;
    static aris::control::Pipe<int> visionPipe;
    static std::thread visionThread;

    static TerrainAnalysis terrainAnalysisResult;
    static ObstacleDetection obstacleDetectionResult;

    static double bodyPose[6];
    static double feetPosi[18];
    static AvoidancePlanner avoidancePlanner;

    static void KinectStart();

    static auto AvoidanceParse(const string &cmd, const map<string, string> &param, aris::core::Msg &msg) -> void;
    static auto StopAvoidanceParse(const string &cmd, const map<string, string> &param, aris::core::Msg &msg) -> void;
    static auto AvoidanceGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in) -> int;

private:
    static atomic_bool isMapAnalysisFinished;
    static atomic_bool isSending;
    static atomic_bool isStop;

};

static AvoidanceGaitWrapper wrapper;
}

#endif // AVOIDANCEGAIT_H
