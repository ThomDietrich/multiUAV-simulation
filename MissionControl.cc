//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "MissionControl.h"

Define_Module (MissionControl);

void MissionControl::initialize() {
    missionQueue[0] = loadCommandsFromWaypointsFile("BostonParkCircle.waypoints");
    missionQueue[1] = loadCommandsFromWaypointsFile("BostonParkLine.waypoints");
    missionQueue[2] = loadCommandsFromWaypointsFile("BostonParkZigZag.waypoints");
    
    cMessage *start = new cMessage("startScheduling");
    scheduleAt(par("startTime"), start);
}

void MissionControl::handleMessage(cMessage *msg) {
    if (msg->isName("startScheduling")) {
        EV_INFO << "Mission Control Scheduling started." << endl;
        
        MissionMsg *uavStart;
        UAVNode *selected;
        
        selected = selectUAVNode();
        if (selected) {
            selected->loadCommands(missionQueue[0]); //TODO dirty hack
            uavStart = new MissionMsg("startMission");
            uavStart->setMissionId(0);
            uavStart->setMission(missionQueue[0]);
            uavStart->setMissionRepeat(true);
            send(uavStart, "gate$o", selected->getIndex());
        }
        selected = selectUAVNode();
        if (selected) {
            selected->loadCommands(missionQueue[0]); //TODO dirty hack
            uavStart = new MissionMsg("startMission");
            uavStart->setMissionId(0);
            uavStart->setMission(missionQueue[0]);
            uavStart->setMissionRepeat(true);
            send(uavStart, "gate$o", selected->getIndex());
        }
        selected = selectUAVNode();
        if (selected) {
            selected->loadCommands(missionQueue[1]); //TODO dirty hack
            uavStart = new MissionMsg("startMission");
            uavStart->setMissionId(1);
            uavStart->setMission(missionQueue[1]);
            uavStart->setMissionRepeat(true);
            send(uavStart, "gate$o", selected->getIndex());
        }
        selected = selectUAVNode();
        if (selected) {
            selected->loadCommands(missionQueue[1]); //TODO dirty hack
            uavStart = new MissionMsg("startMission");
            uavStart->setMissionId(1);
            uavStart->setMission(missionQueue[1]);
            uavStart->setMissionRepeat(true);
            send(uavStart, "gate$o", selected->getIndex());
        }
        selected = selectUAVNode();
        if (selected) {
            selected->loadCommands(missionQueue[2]); //TODO dirty hack
            uavStart = new MissionMsg("startMission");
            uavStart->setMissionId(2);
            uavStart->setMission(missionQueue[2]);
            uavStart->setMissionRepeat(true);
            send(uavStart, "gate$o", selected->getIndex());
        }
        selected = selectUAVNode();
        if (selected) {
            selected->loadCommands(missionQueue[2]); //TODO dirty hack
            uavStart = new MissionMsg("startMission");
            uavStart->setMissionId(2);
            uavStart->setMission(missionQueue[2]);
            uavStart->setMissionRepeat(true);
            send(uavStart, "gate$o", selected->getIndex());
        }
    }
    else if (msg->isName("commandCompleted")) {
        CmdCompletedMsg *ccmsg = check_and_cast<CmdCompletedMsg *>(msg);
        EV_INFO << "commandCompleted message received" << endl;
        delete ccmsg;
    }
    else {
        std::string message = "Unknown message name encountered: ";
        message += msg->getFullName();
        throw cRuntimeError(message.c_str());
    }
    delete msg;
}

/**
 * Load commands from a Mission Planner *.waypoints text file.
 * See: http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format
 *
 * @param fileName relative path to *.waypoints file
 */
CommandQueue MissionControl::loadCommandsFromWaypointsFile(const char* fileName) {
    CommandQueue commands;
    std::ifstream inputFile(fileName);
    int lineCnt = 1;
    int cmdId, unknown1, unknown2, commandType;
    std::string commandName;
    double p1, p2, p3, p4;
    double lat, lon, alt;
    int unknown3;
    
    // Skip first line (header)
    std::string str;
    std::getline(inputFile, str);
    EV_INFO << "Line " << lineCnt << " skipped (1)" << endl;
    // Skip second line (home)
    lineCnt++;
    std::getline(inputFile, str);
    EV_INFO << "Line " << lineCnt << " skipped (2)" << endl;
    
    while (true) {
        lineCnt++;
        inputFile >> cmdId >> unknown1 >> unknown2 >> commandType >> p1 >> p2 >> p3 >> p4 >> lat >> lon >> alt >> unknown3;
        
        if (inputFile.fail()) { //TODO differentiate between EOF and failure
            EV_INFO << "Line " << lineCnt << " failed (EOF)" << endl;
            break;
        }
        //EV_INFO << "Line " << lineCnt << " okay" << endl;
        
        switch (commandType) {
            case 16: { // WAYPOINT
                commands.push_back(new WaypointCommand(OsgEarthScene::getInstance()->toX(lon), OsgEarthScene::getInstance()->toY(lat), alt));
                break;
            }
            case 17: { // LOITER_UNLIM
                throw cRuntimeError("readWaypointsFromFile(): Command not implemented yet: LOITER_UNLIM");
                break;
            }
            case 19: { // LOITER_TIME
                commands.push_back(new HoldPositionCommand(p1));
                break;
            }
            case 20: { // RETURN_TO_LAUNCH
                throw cRuntimeError("readWaypointsFromFile(): Command not implemented yet: RETURN_TO_LAUNCH");
                break;
            }
            case 21: { // LAND
                throw cRuntimeError("readWaypointsFromFile(): Command not implemented yet: LAND");
                break;
            }
            case 22: { // TAKEOFF
                commands.push_back(new TakeoffCommand(alt));
                break;
            }
            default: {
                throw cRuntimeError("readWaypointsFromFile(): Unexpected file content.");
                break;
            }
        }
    }
    return commands;
}

UAVNode* MissionControl::selectUAVNode() {
    cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
    UAVNode *uav;
    for (SubmoduleIterator it(network); !it.end(); ++it) {
        cModule *mod = *it;
        if (mod->isName("uav")) {
            uav = check_and_cast<UAVNode *>(mod);
            if (uav->hasCommandsInQueue()) continue;
            EV_INFO << "MissionControl::selectUAVNode(): " << uav->getFullPath() << endl;
            return uav;
        }
    }
    EV_WARN << "MissionControl::selectUAVNode(): No available Nodes found." << endl;
    return nullptr;
}
