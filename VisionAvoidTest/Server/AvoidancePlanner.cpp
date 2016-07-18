#include "AvoidancePlanner.h"

inline auto acc_up(int n, int i)noexcept->double
{
    return (-1.0 / 2 / n / n / n * i*i*i + 3.0 / 2 / n / n * i*i);
}
inline auto acc_down(int n, int i)noexcept->double
{
    return (-1.0*i*i*i / 2.0 / n / n / n + 3.0 * i*i / 2 / n / n);
}
inline auto dec_up(int n, int i)noexcept->double
{
    return 1 - (-1.0 / 2 / n / n / n * (n - i)*(n - i)*(n - i) + 3.0 / 2 / n / n * (n - i)*(n - i));
}
inline auto dec_down(int n, int i)noexcept->double
{
    return 1 - (-1.0*(n - i)*(n - i)*(n - i) / 2.0 / n / n / n + 3.0 * (n - i)*(n - i) / 2 / n / n);
}
inline auto acc_even(int n, int i)noexcept->double
{
    return 1.0 / n / n  * i * i;
}
inline auto dec_even(int n, int i)noexcept->double
{
    return 1.0 - 1.0 / n / n * (n - i)*(n - i);
}
inline auto even(int n, int i)noexcept->double
{
    return 1.0 / n*i;
}

AvoidancePlanner::AvoidancePlanner()
{
    plannerState = GENBODTANDFEETPOS;
    nowLeftObsPose = { nowRobPose.x, nowRobPose.y + safeDist + radiusVirtObs, radiusVirtObs, 0 };
    nextLeftObsPose = nowLeftObsPose;
    nowRightObsPose = { nowRobPose.x, nowRobPose.y - safeDist - radiusVirtObs, radiusVirtObs, 0 };
    nextRightObsPose = nowRightObsPose;
}

AvoidancePlanner::~AvoidancePlanner()
{

}

void AvoidancePlanner::ChangeGenPoseState()
{
    plannerState = GENBODTANDFEETPOS;
}

void AvoidancePlanner::StartGait(int timeNow)
{
    timeStart = timeNow;
    plannerState = GAITSTART;
}

void AvoidancePlanner::ClearValues()
{
    nowRobPose = { 0, 0, 0, 0, 0, 0 };
    nextRobPose = { 0, 0, 0, 0, 0, 0 };
    nowLeftObsPose = { nowRobPose.x, nowRobPose.y + safeDist + radiusVirtObs, radiusVirtObs, 0 };
    nextLeftObsPose = nowLeftObsPose;
    nowRightObsPose = { nowRobPose.x, nowRobPose.y - safeDist - radiusVirtObs, radiusVirtObs, 0 };
    nextRightObsPose = nowRightObsPose;
}

void AvoidancePlanner::GenBodyandFeetPose()
{
    GenAvoidPath();
    GenBodyPoses();
    GenFeetHolds();
}

void AvoidancePlanner::FindSafePose()
{
    nowLeftObsPose = nextLeftObsPose;
    nowRightObsPose = nextRightObsPose;

    ObsPose nextObsPose = { 0, 0, 0, 0 };
    ObsPose leftObsPose = { 0, 0, 0, 0 };
    ObsPose rightObsPose = { 0, 0, 0, 0 };

    if (obsPoses.size() > 0)
    {
        for (unsigned int i = 0; i < obsPoses.size(); i++)
        {
            if (obsPoses[i].x - nowRobPose.x > 0 && obsPoses[i].x - nowRobPose.x < detectRange)
            {
                nextObsPose = obsPoses[i];
                break;
            }
        }
    }

    if (nextObsPose.r == 0)
    {
        nextRobPose = nowRobPose;
        nextRobPose.x += oneStepLength;

        leftObsPose.r = radiusVirtObs;
        leftObsPose.x = nextRobPose.x;
        leftObsPose.y = nextRobPose.y + safeDist + radiusVirtObs;

        rightObsPose.r = radiusVirtObs;
        rightObsPose.x = nextRobPose.x;
        rightObsPose.y = nextRobPose.y - safeDist - radiusVirtObs;
    }
    else if (nextObsPose.y < nowRobPose.y)
    {
        rightObsPose = nextObsPose;
        leftObsPose.r = radiusVirtObs;
        leftObsPose.x = nextObsPose.x;
        leftObsPose.y = nextObsPose.y + nextObsPose.r + dualDist + radiusVirtObs;

        nextRobPose = nowRobPose;
        nextRobPose.x = nextObsPose.x;
        nextRobPose.y = nextObsPose.y + nextObsPose.r + safeDist;
    }
    else
    {
        leftObsPose = nextObsPose;
        rightObsPose.r = radiusVirtObs;
        rightObsPose.x = nextObsPose.x;
        rightObsPose.y = nextObsPose.y - nextObsPose.r - dualDist - radiusVirtObs;

        nextRobPose = nowRobPose;
        nextRobPose.x = nextObsPose.x;
        nextRobPose.y = nextObsPose.y - nextObsPose.r - safeDist;
    }

    nextLeftObsPose = leftObsPose;
    nextRightObsPose = rightObsPose;
}



double AvoidancePlanner::CalCostFunction(Point3D mP, Point3D mQ, Point3D mR, Point3D mS)
{
    double costValue = 0;

    for (double k = 0; k <= 1; k = k + 0.1)
    {
        double X = mP.x * pow((1 - k), 3) + 3 * mQ.x * k * pow((1 - k), 2) + 3 * mR.x * pow(k, 2) * (1 - k) + mS.x * pow(k, 3);
        double Y = mP.y * pow((1 - k), 3) + 3 * mQ.y * k * pow((1 - k), 2) + 3 * mR.y * pow(k, 2) * (1 - k) + mS.y * pow(k, 3);

        double VX = (9 * mQ.x - 3 * mP.x - 9 * mR.x + 3 * mS.x) * pow(k, 2) + (6 * mP.x - 12 * mQ.x + 6 * mR.x) * k - (3 * mP.x - 3 * mQ.x);
        double VY = (9 * mQ.y - 3 * mP.y - 9 * mR.y + 3 * mS.y) * pow(k, 2) + (6 * mP.y - 12 * mQ.y + 6 * mR.y) * k - (3 * mP.y - 3 * mQ.y);

        double theta = atan2(VY, VX);

        double FX1 = X + lRobot * cos(theta);
        double FY1 = Y + lRobot * sin(theta);

        double BX1 = X - lRobot * cos(theta);
        double BY1 = Y - lRobot * sin(theta);

        double dist1 = sqrt(pow((FX1 - nowLeftObsPose.x), 2) + pow((FY1 - nowLeftObsPose.y), 2)) - rRobot - nowLeftObsPose.r;
        double dist2 = sqrt(pow((FX1 - nowRightObsPose.x), 2) + pow((FY1 - nowRightObsPose.y), 2)) - rRobot - nowRightObsPose.r;

        double dist3 = sqrt(pow((BX1 - nowLeftObsPose.x), 2) + pow((BY1 - nowLeftObsPose.y), 2)) - rRobot - nowLeftObsPose.r;
        double dist4 = sqrt(pow((BX1 - nowRightObsPose.x), 2) + pow((BY1 - nowRightObsPose.y), 2)) - rRobot - nowRightObsPose.r;

        double dist5 = sqrt(pow((FX1 - nextLeftObsPose.x), 2) + pow((FY1 - nextLeftObsPose.y), 2)) - rRobot - nextLeftObsPose.r;
        double dist6 = sqrt(pow((FX1 - nextRightObsPose.x), 2) + pow((FY1 - nextRightObsPose.y), 2)) - rRobot - nextRightObsPose.r;

        double dist7 = sqrt(pow((BX1 - nextLeftObsPose.x), 2) + pow((BY1 - nextLeftObsPose.y), 2)) - rRobot - nextLeftObsPose.r;
        double dist8 = sqrt(pow((BX1 - nextRightObsPose.x), 2) + pow((BY1 - nextRightObsPose.y), 2)) - rRobot - nextRightObsPose.r;


        dist1 = ((1 / dist1 - 1 / dSafe) < 0) ? 0 : 0.5 * pow((1 / dist1 - 1 / dSafe), 2);
        dist2 = ((1 / dist2 - 1 / dSafe) < 0) ? 0 : 0.5 * pow((1 / dist2 - 1 / dSafe), 2);

        dist3 = ((1 / dist3 - 1 / dSafe) < 0) ? 0 : 0.5 * pow((1 / dist3 - 1 / dSafe), 2);
        dist4 = ((1 / dist4 - 1 / dSafe) < 0) ? 0 : 0.5 * pow((1 / dist4 - 1 / dSafe), 2);

        dist5 = ((1 / dist5 - 1 / dSafe) < 0) ? 0 : 0.5 * pow((1 / dist5 - 1 / dSafe), 2);
        dist6 = ((1 / dist6 - 1 / dSafe) < 0) ? 0 : 0.5 * pow((1 / dist6 - 1 / dSafe), 2);

        dist7 = ((1 / dist7 - 1 / dSafe) < 0) ? 0 : 0.5 * pow((1 / dist7 - 1 / dSafe), 2);
        dist8 = ((1 / dist8 - 1 / dSafe) < 0) ? 0 : 0.5 * pow((1 / dist8 - 1 / dSafe), 2);

        costValue += (dist1 + dist2 + dist3 + dist4 + dist5 + dist6 + dist7 + dist8);
    }

    return costValue;
}

void AvoidancePlanner::GenAvoidPath()
{
    FindSafePose();

    P = { nowRobPose.x, nowRobPose.y, nowRobPose.z };
    double theta1 = 0;
    Q = { P.x + 0 * cos(theta1), P.y + 0 * sin(theta1), P.z };

    S = { nextRobPose.x, nextRobPose.y, nextRobPose.z };
    double theta2 = 0;
    R = { S.x + 0 * cos(M_PI + theta2), S.y + 0 * sin(M_PI + theta2), S.z };

    double searchLen = (nextRobPose.x - nowRobPose.x) / 2;
    double costValue = 10000;

    for (double d1Var = 0; d1Var <= searchLen + 0.00001; d1Var += 0.05)
    {
        for (double d2Var = 0; d2Var <= searchLen + 0.0001; d2Var += 0.05)
        {
            Point3D QVar = { P.x + d1Var * cos(theta1), P.y + d1Var * sin(theta1), P.z };
            Point3D RVar = { S.x + d2Var * cos(M_PI + theta2), S.y + d2Var * sin(M_PI + theta2), S.z };

            double costValueVar = CalCostFunction(P, QVar, RVar, S);

            if (costValue >= costValueVar)
            {
                costValue = costValueVar;
                Q = QVar;
                R = RVar;
            }
        }
    }
}

void AvoidancePlanner::GenBodyPoses()
{
    double curveLength = 0;
    bodyPoses.clear();
    timeSeg.clear();

    RobPose startPose = { P.x, P.y, P.z, 0, 0, 0 };
    RobPose targetPose = { S.x, S.y, S.z, 0, 0, 0 };

    bodyPoses.push_back(startPose);
    timeSeg.push_back(0);

    for (double t0 = 0; t0 <= 1 - deltaT + 0.000001; t0 += deltaT)
    {
        double t1 = t0 + deltaT;

        double x0 = P.x * pow((1 - t0), 3) + 3 * Q.x * t0 * pow((1 - t0), 2) + 3 * R.x * pow(t0, 2) * (1 - t0) + S.x * pow(t0, 3);
        double y0 = P.y * pow((1 - t0), 3) + 3 * Q.y * t0 * pow((1 - t0), 2) + 3 * R.y * pow(t0, 2) * (1 - t0) + S.y * pow(t0, 3);

        double x1 = P.x * pow((1 - t1), 3) + 3 * Q.x * t1 * pow((1 - t1), 2) + 3 * R.x * pow(t1, 2) * (1 - t1) + S.x * pow(t1, 3);
        double y1 = P.y * pow((1 - t1), 3) + 3 * Q.y * t1 * pow((1 - t1), 2) + 3 * R.y * pow(t1, 2) * (1 - t1) + S.y * pow(t1, 3);

        double vx0 = (9 * Q.x - 3 * P.x - 9 * R.x + 3 * S.x) * pow(t0, 2) + (6 * P.x - 12 * Q.x + 6 * R.x) * t0 - (3 * P.x - 3 * Q.x);
        double vy0 = (9 * Q.y - 3 * P.y - 9 * R.y + 3 * S.y) * pow(t0, 2) + (6 * P.y - 12 * Q.y + 6 * R.y) * t0 - (3 * P.y - 3 * Q.y);

        double theta = atan2(vy0, vx0);

        RobPose tempRobPose = { x0, y0, 0, 0, 0, theta };

        curveLength += pow(pow((x1 - x0), 2) + pow((y1 - y0), 2), 0.5);

        if (bodyPoses.size() == 1 && curveLength > 0.5 * halfStep)
        {
            curveLength = 0;
            bodyPoses.push_back(tempRobPose);
            timeSeg.push_back(t0);
        }

        if (curveLength > halfStep)
        {
            curveLength = 0;
            bodyPoses.push_back(tempRobPose);
            timeSeg.push_back(t0);
        }
    }
    timeSeg.push_back(1);
    bodyPoses.push_back(targetPose);
}

void AvoidancePlanner::GenFeetHolds()
{
    leftSwing = true;
    feetPoses.clear();

    MatrixXd LFO(3, 3), RFO(3, 3);
    LFO << 0.65, -0.65, 0,
            0.3, 0.3, -0.45,
            1, 1, 1;
    RFO << 0, -0.65, 0.65,
            0.45, -0.3, -0.3,
            1, 1, 1;

    Matrix3d initT, endT;
    initT << cos(bodyPoses.front().gama), -sin(bodyPoses.front().gama), bodyPoses.front().x,
            sin(bodyPoses.front().gama), cos(bodyPoses.front().gama), bodyPoses.front().y,
            0, 0, 1;
    endT << cos(bodyPoses.back().gama), -sin(bodyPoses.back().gama), bodyPoses.back().x,
            sin(bodyPoses.back().gama), cos(bodyPoses.back().gama), bodyPoses.back().y,
            0, 0, 1;

    MatrixXd initLF(3, 3), endLF(3, 3), initRF(3, 3), endRF(3, 3);
    initLF = initT * LFO;
    endLF = endT * LFO;
    initRF = initT * RFO;
    endRF = endT * RFO;

    FootHold initFootHold, endFootHold;
    initFootHold =
    { initLF(0, 0), initLF(1, 0), 0,
      initRF(0, 0), initRF(1, 0), 0,
      initLF(0, 1), initLF(1, 1), 0,
      initRF(0, 2), initRF(1, 2), 0,
      initLF(0, 2), initLF(1, 2), 0,
      initRF(0, 1), initRF(1, 1), 0 };
    endFootHold =
    { endLF(0, 0), endLF(1, 0), 0,
      endRF(0, 0), endRF(1, 0), 0,
      endLF(0, 1), endLF(1, 1), 0,
      endRF(0, 2), endRF(1, 2), 0,
      endLF(0, 2), endLF(1, 2), 0,
      endRF(0, 1), endRF(1, 1), 0 };

    FootHold tempFootHold = initFootHold;

    feetPoses.push_back(tempFootHold);

    for (unsigned int i = 1; i < bodyPoses.size() - 1; i++)
    {
        MatrixXd LF, RF;
        LF = LFO;
        RF = RFO;

        Matrix3d T;
        T << cos(bodyPoses[i].gama), -sin(bodyPoses[i].gama), bodyPoses[i].x,
                sin(bodyPoses[i].gama), cos(bodyPoses[i].gama), bodyPoses[i].y,
                0, 0, 1;

        if (leftSwing)
        {
            MatrixXd nextLF(3, 3);
            LF.row(0) += MatrixXd::Ones(1, 3) * halfStep / 2;
            nextLF = T * LF;

            tempFootHold.feetHold[0] = nextLF(0, 0);
            tempFootHold.feetHold[1] = nextLF(1, 0);
            tempFootHold.feetHold[6] = nextLF(0, 1);
            tempFootHold.feetHold[7] = nextLF(1, 1);
            tempFootHold.feetHold[12] = nextLF(0, 2);
            tempFootHold.feetHold[13] = nextLF(1, 2);

            feetPoses.push_back(tempFootHold);

            leftSwing = !leftSwing;
        }
        else
        {
            MatrixXd nextRF(3, 3);
            RF.row(0) += MatrixXd::Ones(1, 3) * halfStep / 2;
            nextRF = T * RF;

            tempFootHold.feetHold[3] = nextRF(0, 0);
            tempFootHold.feetHold[4] = nextRF(1, 0);
            tempFootHold.feetHold[9] = nextRF(0, 2);
            tempFootHold.feetHold[10] = nextRF(1, 2);
            tempFootHold.feetHold[15] = nextRF(0, 1);
            tempFootHold.feetHold[16] = nextRF(1, 1);

            feetPoses.push_back(tempFootHold);

            leftSwing = !leftSwing;
        }
    }

    if (leftSwing)
    {
        tempFootHold.feetHold[0] = endLF(0, 0);
        tempFootHold.feetHold[1] = endLF(1, 0);
        tempFootHold.feetHold[6] = endLF(0, 1);
        tempFootHold.feetHold[7] = endLF(1, 1);
        tempFootHold.feetHold[12] = endLF(0, 2);
        tempFootHold.feetHold[13] = endLF(1, 2);

        feetPoses.push_back(tempFootHold);
    }
    else
    {
        tempFootHold.feetHold[3] = endRF(0, 0);
        tempFootHold.feetHold[4] = endRF(1, 0);
        tempFootHold.feetHold[9] = endRF(0, 2);
        tempFootHold.feetHold[10] = endRF(1, 2);
        tempFootHold.feetHold[15] = endRF(0, 1);
        tempFootHold.feetHold[16] = endRF(1, 1);

        feetPoses.push_back(tempFootHold);
    }
    feetPoses.push_back(endFootHold);
}

void AvoidancePlanner::GenFootTraj(double feetTrajPosi[18], int timeCount, int numCycle)
{

    const double s = -(M_PI / 2)*cos(M_PI * (timeCount + 1) / halfStepT) + M_PI / 2;

    for (int i = 0; i < 6; i++)
    {
        Point3D foot1 = { feetPoses[numCycle].feetHold[i * 3], feetPoses[numCycle].feetHold[i * 3 + 1], feetPoses[numCycle].feetHold[i * 3 + 2] };
        Point3D foot2 = { feetPoses[numCycle + 1].feetHold[i * 3], feetPoses[numCycle + 1].feetHold[i * 3 + 1], feetPoses[numCycle + 1].feetHold[i * 3 + 2] };

        if (foot1.x != foot2.x)
        {
            Point3D difFoot = { foot2.x - foot1.x, foot2.y - foot1.y, foot2.z - foot1.z };
            double ellipL = sqrt(difFoot.x * difFoot.x + difFoot.y * difFoot.y);
            double ellipH = 0.04;
            double theta = atan2(difFoot.y, difFoot.x);

            double x = foot1.x + (ellipL / 2 - ellipL / 2 * cos(s)) * cos(theta);
            double y = foot1.y + (ellipL / 2 - ellipL / 2 * cos(s)) * sin(theta);
            double z = ellipH * sin(s);
            feetTrajPosi[i * 3 + 0] = x;
            feetTrajPosi[i * 3 + 1] = y;
            feetTrajPosi[i * 3 + 2] = z;
        }
        else
        {
            double x = foot1.x;
            double y = foot1.y;
            double z = 0;
            feetTrajPosi[i * 3 + 0] = x;
            feetTrajPosi[i * 3 + 1] = y;
            feetTrajPosi[i * 3 + 2] = z;
        }
    }
}

int AvoidancePlanner::OutBodyandFeetTraj(double bodyPose[6], double feetPosi[18], int timeNow)
{
    double cBodyPose[6] = { 0 };
    double cFeetPosi[18] = { 0 };

    int iInCycle = (timeNow - timeStart) % halfStepT;
    int numCycle = (timeNow - timeStart) / halfStepT;

    if (timeNow - timeStart < halfStepT)
    {
        double t = timeSeg[numCycle] + acc_even(halfStepT, iInCycle + 1) * timeSeg[numCycle + 1];

        double x = P.x * pow((1 - t), 3) + 3 * Q.x * t * pow((1 - t), 2) + 3 * R.x * pow(t, 2) * (1 - t) + S.x * pow(t, 3);
        double y = P.y * pow((1 - t), 3) + 3 * Q.y * t * pow((1 - t), 2) + 3 * R.y * pow(t, 2) * (1 - t) + S.y * pow(t, 3);

        double vx = (9 * Q.x - 3 * P.x - 9 * R.x + 3 * S.x) * pow(t, 2) + (6 * P.x - 12 * Q.x + 6 * R.x) * t - (3 * P.x - 3 * Q.x);
        double vy = (9 * Q.y - 3 * P.y - 9 * R.y + 3 * S.y) * pow(t, 2) + (6 * P.y - 12 * Q.y + 6 * R.y) * t - (3 * P.y - 3 * Q.y);

        double theta = atan2(vy, vx);

        cBodyPose[0] = x;
        cBodyPose[1] = y;
        cBodyPose[5] = theta;

        GenFootTraj(cFeetPosi, iInCycle, numCycle);
    }
    else if (timeNow - timeStart < (bodyPoses.size() - 2) * halfStepT)
    {
        double t = timeSeg[numCycle] + even(halfStepT, iInCycle + 1) * (timeSeg[numCycle + 1] - timeSeg[numCycle]);

        double x = P.x * pow((1 - t), 3) + 3 * Q.x * t * pow((1 - t), 2) + 3 * R.x * pow(t, 2) * (1 - t) + S.x * pow(t, 3);
        double y = P.y * pow((1 - t), 3) + 3 * Q.y * t * pow((1 - t), 2) + 3 * R.y * pow(t, 2) * (1 - t) + S.y * pow(t, 3);

        double vx = (9 * Q.x - 3 * P.x - 9 * R.x + 3 * S.x) * pow(t, 2) + (6 * P.x - 12 * Q.x + 6 * R.x) * t - (3 * P.x - 3 * Q.x);
        double vy = (9 * Q.y - 3 * P.y - 9 * R.y + 3 * S.y) * pow(t, 2) + (6 * P.y - 12 * Q.y + 6 * R.y) * t - (3 * P.y - 3 * Q.y);

        double theta = atan2(vy, vx);

        cBodyPose[0] = x;
        cBodyPose[1] = y;
        cBodyPose[5] = theta;

        GenFootTraj(cFeetPosi, iInCycle, numCycle);
    }
    else if (timeNow - timeStart < (bodyPoses.size() - 1) * halfStepT)
    {
        double t = timeSeg[numCycle] + dec_even(halfStepT, iInCycle + 1) * (timeSeg[numCycle + 1] - timeSeg[numCycle]);

        double x = P.x * pow((1 - t), 3) + 3 * Q.x * t * pow((1 - t), 2) + 3 * R.x * pow(t, 2) * (1 - t) + S.x * pow(t, 3);
        double y = P.y * pow((1 - t), 3) + 3 * Q.y * t * pow((1 - t), 2) + 3 * R.y * pow(t, 2) * (1 - t) + S.y * pow(t, 3);

        double vx = (9 * Q.x - 3 * P.x - 9 * R.x + 3 * S.x) * pow(t, 2) + (6 * P.x - 12 * Q.x + 6 * R.x) * t - (3 * P.x - 3 * Q.x);
        double vy = (9 * Q.y - 3 * P.y - 9 * R.y + 3 * S.y) * pow(t, 2) + (6 * P.y - 12 * Q.y + 6 * R.y) * t - (3 * P.y - 3 * Q.y);

        double theta = atan2(vy, vx);

        cBodyPose[0] = x;
        cBodyPose[1] = y;
        cBodyPose[5] = theta;

        GenFootTraj(cFeetPosi, iInCycle, numCycle);
    }
    else if (timeNow - timeStart < bodyPoses.size() * halfStepT)
    {
        double x = S.x;
        double y = S.y;
        double theta = 0;

        cBodyPose[0] = x;
        cBodyPose[1] = y;
        cBodyPose[5] = theta;

        GenFootTraj(cFeetPosi, iInCycle, numCycle);
    }

    for (int i = 0; i < 6; i++)
    {
        feetPosi[i * 3] = cFeetPosi[(5 -i) * 3 + 1];
        feetPosi[i * 3 + 1] = cFeetPosi[(5 - i) * 3 + 2] - 0.85;
        feetPosi[i * 3 + 2] = cFeetPosi[(5 - i) * 3];
    }

    bodyPose[0] = cBodyPose[1];
    bodyPose[1] = cBodyPose[2];
    bodyPose[2] = cBodyPose[0];
    bodyPose[3] = M_PI / 2;
    bodyPose[4] = cBodyPose[5];
    bodyPose[5] = -M_PI / 2;

    int n = bodyPoses.size() * halfStepT - timeNow + timeStart - 1;

    if (n == 0)
    {
        plannerState = GAITEND;
        nowRobPose = nextRobPose;
    }

    return n;
}
