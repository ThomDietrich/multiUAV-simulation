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
#include <boost/algorithm/string.hpp>

Define_Module(MissionControl);

void MissionControl::initialize()
{
    std::vector<std::string> missionFiles;
    const char* missionFilesString = par("missionFiles").stringValue();
    boost::split(missionFiles, missionFilesString, boost::algorithm::is_any_of(","), boost::token_compress_on);
    for (auto it = missionFiles.begin(); it != missionFiles.end(); it++) {
        missionQueue.push_back(loadCommandsFromWaypointsFile(it->c_str()));
    }

    // Add all GenericNodes to managedNodes list (map)
    cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
    for (SubmoduleIterator it(network); !it.end(); ++it) {
        cModule *module = *it;
        if (not module->isName("uav")) {
            continue;
        }
        EV_DEBUG << __func__ << "(): Adding " << module->getFullName() << " to managedNodes" << endl;
        NodeShadow *nodeShadow = new NodeShadow(check_and_cast<GenericNode *>(module));
        managedNodeShadows.add(nodeShadow);
    }
    cMessage *start = new cMessage("startScheduling");
    scheduleAt(par("startTime"), start);
}

void MissionControl::handleMessage(cMessage *msg)
{
    if (msg->isName("startScheduling")) {
        for (auto it = missionQueue.begin(); it != missionQueue.end(); it++) {
            CommandQueue mission = *it;
            int missionId = it - missionQueue.begin();

            //Select free idle node
            NodeShadow *nodeShadow = managedNodeShadows.getClosest(NodeStatus::IDLE, mission.front()->getX(), mission.front()->getY(), mission.front()->getZ());

            // Generate and send out start mission message
            MissionMsg *nodeStartMission = new MissionMsg("startMission");
            nodeStartMission->setMissionId(missionId);
            nodeStartMission->setMission(mission);
            nodeStartMission->setMissionRepeat(true);
            send(nodeStartMission, "gate$o", nodeShadow->getNodeIndex());

            // Mark node accordingly
            nodeShadow->setStatus(NodeStatus::PROVISIONING);

            EV_INFO << __func__ << "(): Mission " << missionId << " assigned to node " << nodeShadow->getNode()->getFullName() << " (PROVISIONING)." << endl;
        }
    }
    else if (msg->isName("commandCompleted")) {
        CmdCompletedMsg *ccmsg = check_and_cast<CmdCompletedMsg *>(msg);
        NodeShadow* nodeShadow = managedNodeShadows.get(ccmsg->getSourceNodeIndex());
        //EV_INFO << __func__ << "(): commandCompleted message received for " << nodeShadow->getNode()->getFullName() << endl;

        //TODO: too simple. Should e.g. not happen for ReplacingNode
        if (nodeShadow->isStatusProvisioning()) {
            EV_INFO << "Node " << nodeShadow->getNodeIndex() << " switching over to nodeStatus MISSION" << endl;
            nodeShadow->setStatus(NodeStatus::MISSION);
        }
        if (ccmsg->getReplacementDataAvailable()) {
            handleReplacementMessage(ccmsg->getReplacementData());
        }
    }
    else if (msg->isName("provisionReplacement")) {
        // Identify node requesting replacement
        NodeShadow* nodeShadow = managedNodeShadows.getNodeRequestingReplacement(msg);
        GenericNode *replacingNode = nodeShadow->getReplacingNode();
        ReplacementData *replData = nodeShadow->getReplacementData();
        EV_INFO << "provisionReplacement message received for node " << nodeShadow->getNodeIndex() << endl;

        // When the replacing node ist charging currently send a message to stop the process
        if (replacingNode->getCommandExecEngine()) {
            if (replacingNode->getCommandExecEngine()->getCeeType() == CeeType::CHARGE) {
                cMessage *exitMessage = new cMessage("mobileNodeExit");
                send(exitMessage, "gate$o", replacingNode->getIndex());
            }
        }

        // Send provision mission to replacing node
        CommandQueue provMission;
        provMission.push_back(new WaypointCommand(replData->x, replData->y, replData->z));
        provMission.push_back(new ExchangeCommand(nodeShadow->getNode(), false, false));
        MissionMsg *nodeStartMission = new MissionMsg("startProvision");
        nodeStartMission->setMission(provMission);
        send(nodeStartMission, "gate$o", replacingNode->getIndex());

        // Set "otherNode" for exchangeCEE of replaced node
        // TODO: This is part of hack111...
        UAVNode *replacedNode = dynamic_cast<UAVNode *>(nodeShadow->getNode());
        replacedNode->replacingNode = replacingNode;
        replacedNode->replacementX = replData->x;
        replacedNode->replacementY = replData->y;
        replacedNode->replacementZ = replData->z;
        replacedNode->replacementTime = replData->timeOfReplacement;

        managedNodeShadows.setStatus(replacingNode, NodeStatus::PROVISIONING);

        EV_INFO << __func__ << "(): Mission PROVISION assigned to node " << replacingNode->getIndex();
        EV_INFO << " (replacing node " << nodeShadow->getNodeIndex() << ")" << endl;
    }
    else if (msg->isName("mobileNodeResponse")) {
        // write requested mobileNode information in corresponding nodeShadow's
        MobileNodeResponse *mnmsg = check_and_cast<MobileNodeResponse *>(msg);
        if (mnmsg->getNodeFound()) {
            NodeShadow* nodeShadow = managedNodeShadows.get(mnmsg->getMobileNodeIndex());
            nodeShadow->setKnownBattery(new Battery(mnmsg->getCapacity(), mnmsg->getRemaining()));
        }
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
    NodeShadow* nodeShadow = managedNodeShadows.get(replData.nodeToReplace);

    // TODO: Test if new selection would differ...

    if (nodeShadow->hasReplacingNode()) {
        GenericNode* replNode = nodeShadow->getReplacingNode();

        if (managedNodeShadows.get(replNode)->isStatusProvisioning()) {
            EV_WARN << __func__ << "(): Replacing Node is already on its way. No re-calculation needed. " << endl;
            return;
        }
        nodeShadow->setReplacementData(new ReplacementData(replData));
        nodeShadow->setReplacingNode(replNode);
    }
    else {
        // ToDo: Add highest capacity from config
        this->requestChargedNodesInformation(5400);
        // Get first free IDLE node
        NodeShadow* replacingNodeShadow = managedNodeShadows.getClosest(NodeStatus::IDLE, replData.x, replData.y, replData.z);
        if (!replacingNodeShadow) {
            replacingNodeShadow = managedNodeShadows.getHighestCharged();
        }
        // Assign as replacing node to this node
        replacingNodeShadow->setStatus(NodeStatus::RESERVED);
        nodeShadow->setReplacementData(new ReplacementData(replData));
        nodeShadow->setReplacingNode(replacingNodeShadow->getNode());

        EV_INFO << __func__ << "(): No ReplacingNode for node " << nodeShadow->getNodeIndex() << ",";
        EV_INFO << " node " << nodeShadow->getReplacingNodeIndex() << " reserved for replacement" << endl;
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

    cMessage *replacementMsg = new cMessage("provisionReplacement");

    if (simTime() < timeOfProvisioning) {
        // Delete and reschedule if old msg available
        bool reprovision = false;
        if (nodeShadow->hasReplacementMsg()) {
            if (nodeShadow->getReplacementMsg()->isSelfMessage()) {
                cancelEvent(nodeShadow->getReplacementMsg());
                delete nodeShadow->getReplacementMsg();
                reprovision = true;
            }
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
                commands.push_back(new HoldPositionCommand(OsgEarthScene::getInstance()->toX(lon), OsgEarthScene::getInstance()->toY(lat), alt, p1));
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

void MissionControl::requestChargedNodesInformation(double remainingBattery)
{
    // Send request to all ChargingStations
    cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
    for (SubmoduleIterator it(network); !it.end(); ++it) {
        cModule *module = *it;
        if (not module->isName("cs")) {
            continue;
        }
        MobileNodeRequest *mnRequest = new MobileNodeRequest("mobileNodeRequest");
        mnRequest->setRemaining(remainingBattery);
        send(mnRequest, getOutputGateTo(module));
    }
}

/**
 * Find and return the cGate pointing to another cModule.
 *
 * @param cMod
 * @return cGate*, 'nullptr' if no gate found
 */
cGate* MissionControl::getOutputGateTo(cModule *cMod)
{
    for (int i = 0; i < this->gateCount(); i++) {
        cGate *gate = this->gateByOrdinal(i);
        if (gate->getType() == cGate::Type::OUTPUT) {
            cModule *gateOwner = gate->getPathEndGate()->getOwnerModule();
            if (gateOwner == cMod) {
                return gate;
            }
            //EV_INFO << "Node is connected to " << gateOwner->getFullName() << " through gate at index " << i << ": " << gate->getFullName() << endl;
        }
    }
    return nullptr;
}

#endif // WITH_OSG
