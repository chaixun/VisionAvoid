#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>
#include <vector>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>
#include "AvoidanceGait.h"

#include "rtdk.h"
#include "unistd.h"

using namespace aris::core;
using namespace RobObsPose;


int main(int argc, char *argv[])
{   
    ObsPose simObsPoses[6] =
    { { 1.35, -0.5, 0.353774, 0 },
      { 2.80, 0.9, 0.290743, 0 },
      { 4.30, -0.5, 0.257694, 0 },
      { 5.80, 0.9, 0.251558, 0 },
      { 7.30, -0.5, 0.285318, 0 },
      { 8.80, 0.9, 0.313249, 0 }
    };

    for (int i = 0; i < 6; i++)
    {
        obsPoses.push_back(simObsPoses[i]);
    }

    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/VisionAvoid/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/VisionAvoid/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
        xml_address = "/home/hex/VisionAvoid/Robot_VIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);

    rs.addCmd("vwk", Avoidance::AvoidanceGaitWrapper::AvoidanceParse, Avoidance::AvoidanceGaitWrapper::AvoidanceGait);

    rs.open();

    rs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)
            throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });

    aris::core::runMsgLoop();

    return 0;
}
