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

#ifdef WITH_OSG
#include "MissionControl.h"

Define_Module(MissionControl);

void MissionControl::initialize()
{
    missionQueue.push_back(loadCommandsFromWaypointsFile("BostonParkCircle.waypoints"));
    missionQueue.push_back(loadCommandsFromWaypointsFile("BostonParkLine.waypoints"));
    missionQueue.push_back(loadCommandsFromWaypointsFile("BostonParkZigZag.waypoints"));

    // Add all GenericNodes to managedNodes list (map)
    cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
    for (SubmoduleIterator it(network); !it.end(); ++it) {
        cModule *module = *it;
        if (not module->isName("uav")) {
            continue;
        }
        NodeData *nodedata = new NodeData();
        nodedata->node = check_and_cast<GenericNode *>(module);
        nodedata->status = NodeStatus::IDLE;
        nodedata->replacementData = nullptr;
        EV_DEBUG << __func__ << "(): Adding " << module->getFullName() << " to managedNodes" << endl;
        std::pair<int, NodeData*> nodePair(nodedata->node->getIndex(), nodedata);
        managedNodes.insert(nodePair);
    }

    cMessage *start = new cMessage("startScheduling");
    scheduleAt(par("startTime"), start);
}

void MissionControl::handleMessage(cMessage *msg)
{
    if (msg->isName("startScheduling")) {
        GenericNode *idleNode;

        for (auto it = missionQueue.begin(); it != missionQueue.end(); it++) {
            idleNode = selectIdleNode();
            if (not idleNode) throw cRuntimeError("startScheduling: No nodes left to schedule.");

            CommandQueue mission = *it;
            int missionId = it - missionQueue.begin();

            // Generate and send out start mission message
            MissionMsg *nodeStartMission = new MissionMsg("startMission");
            nodeStartMission->setMissionId(missionId);
            nodeStartMission->setMission(mission);
            nodeStartMission->setMissionRepeat(true);
            send(nodeStartMission, "gate$o", idleNode->getIndex());

            // Mark node as with mission
            managedNodes.at(idleNode->getIndex())->status = NodeStatus::PROVISIONING;

            EV_INFO << __func__ << "(): Mission " << missionId << " assigned to node " << idleNode->getFullName() << ", node in state PROVISION." << endl;
        }
    }
    else if (msg->isName("commandCompleted")) {
        CmdCompletedMsg *ccmsg = check_and_cast<CmdCompletedMsg *>(msg);
        EV_INFO << "commandCompleted message received" << endl;
        NodeData* nodeData = managedNodes.at(ccmsg->getSourceNode());
        if (nodeData->status == NodeStatus::PROVISIONING) {
            EV_INFO << "Node switching over to nodeStatus MISSION" << endl;
            nodeData->status = NodeStatus::MISSION;
        }
        handleReplacementMessage(ccmsg->getReplacementData());
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
CommandQueue MissionControl::loadCommandsFromWaypointsFile(const char* fileName)
{
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

[[deprecated]]
UAVNode* MissionControl::selectUAVNode()
{
    EV_WARN << __func__ << "(): Deprecated. Maybe reuse later" << endl;
    return nullptr;
}

/**
 * Choose a free node from the managedNodes map.
 * Selection happens by lowest module index and amongst the IDLE nodes.
 */
GenericNode* MissionControl::selectIdleNode()
{
    for (auto it = managedNodes.begin(); it != managedNodes.end(); ++it) {
        if (it->second->status == NodeStatus::IDLE) {
            GenericNode* node = check_and_cast<MobileNode *>(it->second->node);
            //EV_DEBUG << __func__ << "(): " << node->getFullPath() << endl;
            return node;
        }
    }
    throw cRuntimeError("MissionControl::selectIdleNode(): No available Nodes found. This case is not handled yet.");
    return nullptr;
}

/**
 * Update the managedNodes map with the replacement request by a node.
 * After each update, reschedule the replacement process, i.e. the 'provisionReplacement' self-message.
 *
 * @param replData Incoming ReplacementData
 */
void MissionControl::handleReplacementMessage(ReplacementData replData)
{
    NodeData* nodeData = managedNodes.at(replData.nodeToReplace->getIndex());
    nodeData->replacementData = &replData;

    // Delete and reschedule if old msg available
    if (nodeData->replacementMsg != nullptr) {
        cancelEvent(nodeData->replacementMsg);
    }
    // If not available reserve an idle node as the replacing node
    if (nodeData->replacementData->replacingNode == nullptr) {
        nodeData->replacementData->replacingNode = selectIdleNode();
        nodeData->status = NodeStatus::RESERVED;
    }
    //TODO unclear: nodes in NodeData are of GenericNode but only MobileNode knows movement
    UAVNode* replacingUavNode = check_and_cast<UAVNode *>(nodeData->replacementData->replacingNode);

    cMessage *replacementMsg = new cMessage("provisionReplacement");
    nodeData->replacementMsg = replacementMsg;
    scheduleAt(nodeData->replacementData->timeOfReplacement, replacementMsg);
}

#endif // WITH_OSG
