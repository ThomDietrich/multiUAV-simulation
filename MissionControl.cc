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
    //missionQueue.push_back(loadCommandsFromWaypointsFile("BostonParkLine.waypoints"));
    //missionQueue.push_back(loadCommandsFromWaypointsFile("BostonParkZigZag.waypoints"));

    // Add all GenericNodes to managedNodes list (map)
    cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
    for (SubmoduleIterator it(network); !it.end(); ++it) {
        cModule *module = *it;
        if (not module->isName("uav")) {
            continue;
        }
        NodeShadow *nodeShadow = new NodeShadow(check_and_cast<GenericNode *>(module));

        EV_DEBUG << __func__ << "(): Adding " << module->getFullName() << " to managedNodes" << endl;
        std::pair<int, NodeShadow*> nodePair(nodeShadow->getNodeIndex(), nodeShadow);
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
            if (not idleNode) throw cRuntimeError("startScheduling: No nodes left to schedule for mission.");

            CommandQueue mission = *it;
            int missionId = it - missionQueue.begin();

            // Generate and send out start mission message
            MissionMsg *nodeStartMission = new MissionMsg("startMission");
            nodeStartMission->setMissionId(missionId);
            nodeStartMission->setMission(mission);
            nodeStartMission->setMissionRepeat(true);
            send(nodeStartMission, "gate$o", idleNode->getIndex());

            // Mark node as with mission
            managedNodes.at(idleNode->getIndex())->setStatus(NodeStatus::PROVISIONING);

            EV_INFO << __func__ << "(): Mission " << missionId << " assigned to node " << idleNode->getFullName() << ", node in state PROVISION." << endl;
        }
    }
    else if (msg->isName("commandCompleted")) {
        CmdCompletedMsg *ccmsg = check_and_cast<CmdCompletedMsg *>(msg);
        EV_INFO << "commandCompleted message received" << endl;
        NodeShadow* nodeShadow = managedNodes.at(ccmsg->getSourceNode());
        if (nodeShadow->isStatusProvisioning()) {
            EV_INFO << "Node switching over to nodeStatus MISSION" << endl;
            nodeShadow->setStatus(NodeStatus::MISSION);
        }
        if (ccmsg->getReplacementDataAvailable()) handleReplacementMessage(ccmsg->getReplacementData());
    }
    else if (msg->isName("provisionReplacement")) {
        // Identify node requesting replacement
        NodeShadow* nodeShadow = nullptr;
        for (auto it = managedNodes.begin(); it != managedNodes.end(); ++it) {
            if (it->second->compareReplacementMsg(msg)) {
                nodeShadow = it->second;
                break;
            }
        }
        if (nodeShadow == nullptr) {
            throw cRuntimeError("provisionReplacement message not found amongst the managed nodes.");
        }
        GenericNode *replacingNode = nodeShadow->getReplacingNode();
        ReplacementData *replData = nodeShadow->getReplacementData();
        EV_INFO << "provisionReplacement message received for node " << nodeShadow->getNodeIndex() << endl;

        // Send provision mission to node
        CommandQueue provMission;
        provMission.push_back(new WaypointCommand(replData->x, replData->y, replData->z));
        provMission.push_back(new ExchangeCommand());
        MissionMsg *nodeStartMission = new MissionMsg("startProvision");
        nodeStartMission->setMission(provMission);
        send(nodeStartMission, "gate$o", replacingNode->getIndex());

        managedNodes.at(replacingNode->getIndex())->setStatus(NodeStatus::PROVISIONING);

        EV_INFO << __func__ << "(): Mission PROVISION assigned to node " << replacingNode->getIndex() << " (replacing node " << nodeShadow->getNodeIndex()
                << ")" << endl;
    }
    else {
        std::string message = "Unknown message name encountered: ";
        message += msg->getFullName();
        throw cRuntimeError(message.c_str());
    }
    delete msg;
}

/**
 * Update the managedNodes map with the replacement request by a node.
 * After each update, reschedule the replacement process, i.e. the 'provisionReplacement' self-message.
 * Method will reserve a node as soon as the first replacement message arrives for one node executing a mission - this might change in future.
 *
 * @param replData Incoming ReplacementData
 */
void MissionControl::handleReplacementMessage(ReplacementData replData)
{
    NodeShadow* nodeShadow = managedNodes.at(replData.nodeToReplace->getIndex());

    // TODO: Test if new selection would differ...
    if (nodeShadow->hasReplacingNode()) {
        GenericNode* replNodeOld = nodeShadow->getReplacingNode();
        nodeShadow->setReplacementData(new ReplacementData(replData));
        nodeShadow->setReplacingNode(replNodeOld);
    }
    else {
        nodeShadow->setReplacementData(new ReplacementData(replData));
        nodeShadow->setReplacingNode(selectIdleNode());
        EV_INFO << __func__ << "(): No ReplacingNode for node " << nodeShadow->getNodeIndex() << ",";
        EV_INFO << " node " << nodeShadow->getReplacingNodeIndex() << " reserved for replacement" << endl;
        NodeShadow* replNodeShadow = managedNodes.at(nodeShadow->getReplacingNodeIndex());
        replNodeShadow->setStatus(NodeStatus::RESERVED);
    }

    simtime_t timeOfProvisioning;
    {
        //Retrieve provisioning time
        //TODO unclear: nodes in NodeShadow are of GenericNode but only MobileNode knows movement
        UAVNode* replacingUavNode = check_and_cast<UAVNode *>(nodeShadow->getReplacingNode());
        CommandQueue commands;
        commands.push_back(new WaypointCommand(nodeShadow->getReplacementData()->x, nodeShadow->getReplacementData()->y, nodeShadow->getReplacementData()->z));
        simtime_t timeOfReplacement = nodeShadow->getReplacementTime();
        replacingUavNode->clearCommands();
        replacingUavNode->loadCommands(commands);
        double timeForProvisioning = replacingUavNode->estimateCommandsDuration();
        timeOfProvisioning = timeOfReplacement - timeForProvisioning;
        replacingUavNode->clearCommands();
    }

    //throw cRuntimeError("test");
    cMessage *replacementMsg = new cMessage("provisionReplacement");

    if (simTime() < timeOfProvisioning) {
        // Delete and reschedule if old msg available
        bool reprovision = false;
        if (nodeShadow->hasReplacementMsg()) {
            cancelEvent(nodeShadow->getReplacementMsg());
            delete nodeShadow->getReplacementMsg();
            reprovision = true;
        }
        nodeShadow->setReplacementMsg(replacementMsg);
        scheduleAt(timeOfProvisioning, replacementMsg);
        EV_INFO << __func__ << "(): " << (reprovision ? "Updating provision time." : "Provisioning node.");
        EV_INFO << " Node " << nodeShadow->getNodeIndex() << " will be replaced by node " << nodeShadow->getReplacingNodeIndex() << ".";
        EV_INFO << " Provisioning in " << (timeOfProvisioning - simTime()) << " seconds";
        EV_INFO << endl;
    }
    else {
        EV_WARN << "Prediction time is in the past. This needs to be dealt with... somehow" << endl;
    }
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
        if (it->second->isStatusIdle()) {
            return it->second->getNode();
        }
    }
    throw cRuntimeError("MissionControl::selectIdleNode(): No available Nodes found. This case is not handled yet.");
    return nullptr;
}

#endif // WITH_OSG
