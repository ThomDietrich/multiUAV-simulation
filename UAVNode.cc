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
#include <fstream>
#include <iostream>
#include <sstream>

#include "UAVNode.h"
#include "OsgEarthScene.h"
#include "ChannelController.h"

using namespace omnetpp;

Define_Module(UAVNode);

#define ERROR_MARGIN 0.0625

UAVNode::UAVNode()
{
}

UAVNode::~UAVNode()
{
}

/**
 * Simulation initialization with two stages, i.e. setup cycles
 *
 * @param stage
 */
void UAVNode::initialize(int stage)
{
    MobileNode::initialize(stage);
    switch (stage) {
        case 0: {
            // initial position
            x = par("startX");
            y = par("startY");
            z = par("startZ");
            break;
        }
        case 1: {
            break;
        }
    }
}

void UAVNode::handleMessage(cMessage *msg)
{
    double stepSize = 0;
    if (msg->isName("exchangeData")) {
        EV_INFO << __func__ << "(): exchangeData message received" << endl;

        if (commandExecEngine->getCeeType() != CeeType::EXCHANGE) {
            EV_WARN << __func__ << "(): Node not in ExchangeCEE. Ignoring exchangeData message." << endl;
            exchangeAfterCurrentCommand = true;
            delete msg;
            msg = nullptr;
            return;
        }

        ExchangeCEE *exchangeCEE = check_and_cast<ExchangeCEE *>(commandExecEngine);

        // Send out mission data
        if (not exchangeCEE->dataTransferPerformed) {
            UAVNode *node = check_and_cast<UAVNode *>(exchangeCEE->getOtherNode());
            transferMissionDataTo(node);
            exchangeCEE->dataTransferPerformed = true;
        }

        // Handle received mission data
        if (exchangeCEE->commandCompleted()) {
            EV_WARN << __func__ << "(): Mission exchange already completed." << endl;
        }
        else {
            MissionMsg *receivedMissionMsg = check_and_cast<MissionMsg *>(msg);
            missionId = receivedMissionMsg->getMissionId();
            commandsRepeat = receivedMissionMsg->getMissionRepeat();
            CommandQueue missionCommands = receivedMissionMsg->getMission();
            EV_WARN << __func__ << "(): Mission exchange, clearing " << cees.size() << " cees, loading " << missionCommands.size() << endl;
            clearCommands();
            loadCommands(missionCommands);

            // End ExchangeCEE, will trigger next command selection
            exchangeCEE->setExchangeCompleted();
        }

        delete msg;
        msg = nullptr;
    }
    else {
        MobileNode::handleMessage(msg);
        msg = nullptr;
    }

    if (msg != nullptr) {
        scheduleAt(simTime() + stepSize, msg);
    }
}

void UAVNode::transferMissionDataTo(UAVNode* node)
{
    CommandQueue missionCommands = *extractCommands();
    MissionMsg *exDataMsg = new MissionMsg("exchangeData");
    exDataMsg->setMission(missionCommands);
    exDataMsg->setMissionRepeat(commandsRepeat);
    exDataMsg->setMissionId(missionId);
    cGate* gateToNode = getOutputGateTo(node);
    send(exDataMsg, gateToNode);
    EV_INFO << __func__ << "(): " << missionCommands.size() << " commands extracted and sent to other node." << endl;
}

/**
 * Fetches the next command from the commands queue and creates a corresponding CEE.
 *
 * @throws cRuntimeError if no commands left in queue
 */
void UAVNode::selectNextCommand()
{
    if (cees.size() == 0) {
        throw cRuntimeError("selectNextCommand(): UAV has no commands in CEEs queue left.");
    }

    CommandExecEngine *scheduledCEE = cees.front();
    scheduledCEE->setFromCoordinates(getX(), getY(), getZ());
    scheduledCEE->initializeCEE();

    if (exchangeAfterCurrentCommand && not scheduledCEE->isCeeType(CeeType::EXCHANGE)) {
        throw cRuntimeError("selectNextCommand(): UAV should have switched into Exchange CEE now. Another UAV is waiting...");
    }
    else {
        exchangeAfterCurrentCommand = false;
    }

    bool choseClosestCNForExchange = par("choseClosestCNForExchange").boolValue();

    if (choseClosestCNForExchange) {
        ClosestThings closest = findClosest();

        /*
         *  calculate energy consumption
         */

        if (nullptr != replacingNode && cmpCoord(*replacingNode->getCommandExecEngine()->extractCommand(), getX(), getY(), getZ())) {
            // Generate and inject ExchangeCEE, only if not already done
            if (scheduledCEE->isPartOfMission()) {
                ExchangeCommand *exchangeCommand = new ExchangeCommand(replacingNode, true, true);
                CommandExecEngine *exchangeCEE = new ExchangeCEE(this, exchangeCommand);
                exchangeCEE->setPartOfMission(false);
                cees.push_front(exchangeCEE);
                EV_INFO << __func__ << "(): ExchangeCEE added to node." << endl;
            }
        }
        else {
            CommandExecEngine* nextCEE = nullptr;
            u_int nextCommandsFeasible = 0;

            // energy from closest command to closest charging station
            float energyCMDtoCN = estimateEnergy(                                  // estimate energy flying
                    closest.cmd->getX(), closest.cmd->getY(), closest.cmd->getZ(), // from closest command
                    closest.cn->getX(), closest.cn->getY(), closest.cn->getZ());   // to charging station

            // energy to closest command
            float energyToCMD = 0;
            if (not cmpCoord(*closest.cmd, this->getX(), this->getY(), this->getZ())) {
                do {
                    nextCEE = cees.at(nextCommandsFeasible % cees.size());
                    nextCommandsFeasible++;
                    energyToCMD += energyForCEE(nextCEE);
                } while (not cmpCoord(*(nextCEE->extractCommand()), *closest.cmd));
            }

            // estimate energy for one complete mission run
            u_int startingPosition = nextCommandsFeasible % cees.size();
            float energyMission = 0;
            do {
                nextCommandsFeasible++;
                nextCEE = cees.at(nextCommandsFeasible % cees.size());
                energyMission += energyForCEE(nextCEE);
            } while ((nextCommandsFeasible % cees.size()) != startingPosition);

            if (scheduledCEE->getCeeType() == CeeType::CHARGE) {
                EV_INFO << "Energy Management: Recharging now." << endl;
            }
            else if (battery.isEmpty()) {
                EV_ERROR << "Energy Management: One of our precious UAVs just died :-(" << endl;
                //throw cRuntimeError("Energy Management: One of our precious UAVs just died :-(");
            }
            else if (energyCMDtoCN + energyToCMD + energyMission < battery.getRemaining()) {
                // do nothing and continue at least one mission
                EV_INFO << "Energy Management: OK. UAV has enough energy to complete at least one mission";
                EV_INFO << " (" << std::setprecision(1) << std::fixed << this->battery.getRemainingPercentage() << "%)." << endl;
            }
            else {
                // Start Replacement Process now
                if (energyCMDtoCN + energyToCMD > battery.getRemaining()) {
                    EV_WARN << "Energy Management: Going to Charging Node. Attention! Energy insufficient";
                    EV_WARN << " (" << battery.getRemaining() << " < " << energyCMDtoCN + energyToCMD << " mAh)." << endl;
                }
                else {
                    if (cmpCoord(*closest.cmd, this->getX(), this->getY(), this->getZ())) {
                        EV_INFO << "Energy Management: Scheduling Replacement and Recharging Maintenance Process";
                        EV_INFO << " (" << std::setprecision(1) << std::fixed << this->battery.getRemainingPercentage() << "%)." << endl;

                        if (replacingNode == nullptr) throw cRuntimeError("selfScheduleExchange(): replacingNode should be known by now (part of hack111).");

                        // Generate and inject ExchangeCEE, only if not already done
                        if (scheduledCEE->isPartOfMission()) {
                            ExchangeCommand *exchangeCommand = new ExchangeCommand(replacingNode, true, true);
                            CommandExecEngine *exchangeCEE = new ExchangeCEE(this, exchangeCommand);
                            exchangeCEE->setPartOfMission(false);
                            cees.push_front(exchangeCEE);
                            EV_INFO << __func__ << "(): ExchangeCEE added to node." << endl;
                        }
                    }
                    else {
                        EV_INFO << "Energy Management: Resume flight until closest command for exchange.";
                        EV_INFO << " (" << std::setprecision(1) << std::fixed << this->battery.getRemainingPercentage() << "%)." << endl;
                    }
                }
            }
        }
    }
    else {
        float energyForSheduled = scheduledCEE->predictFullConsumptionQuantile();
        float energyToCNNow = energyToNearestCN(getX(), getY(), getZ());
        float energyToCNAfterScheduled = energyToNearestCN(scheduledCEE->getX1(), scheduledCEE->getY1(), scheduledCEE->getZ1());
        float energyRemaining = this->battery.getRemaining();
        bool atReplacementLocation = abs(replacementX - x) < 0.1 && abs(replacementY - y) < 0.1 && abs(replacementZ - z) < 0.1
                && abs((replacementTime - simTime()).dbl()) < 5;

        if (scheduledCEE->getCeeType() == CeeType::CHARGE) {
            EV_INFO << "Energy Management: Recharging now." << endl;
        }
        else if (battery.isEmpty()) {
            EV_ERROR << "Energy Management: One of our precious UAVs just died :-(" << endl;
            //throw cRuntimeError("Energy Management: One of our precious UAVs just died :-(");
        }
        else if (not atReplacementLocation && energyRemaining >= energyForSheduled + energyToCNAfterScheduled) {
            EV_INFO << "Energy Management: OK. UAV has enough energy to continue";
            EV_INFO << " (" << std::setprecision(1) << std::fixed << this->battery.getRemainingPercentage() << "%)." << endl;
        }
        else {
            // Start Replacement Process now
            if (atReplacementLocation) {
                EV_INFO << "Energy Management: replacementTime reached, starting replacement (part of hack111)" << endl;
            }
            if (energyRemaining < energyToCNNow) {
                EV_WARN << "Energy Management: Going to Charging Node. Attention! Energy insufficient";
                EV_WARN << " (" << energyRemaining << " < " << energyToCNNow << " mAh)." << endl;
            }
            else {
                EV_INFO << "Energy Management: Scheduling Replacement and Recharging Maintenance Process";
                EV_INFO << " (" << std::setprecision(1) << std::fixed << this->battery.getRemainingPercentage() << "%)." << endl;
            }

            if (replacingNode == nullptr) throw cRuntimeError("selfScheduleExchange(): replacingNode should be known by now (part of hack111).");

            // Generate and inject ExchangeCEE, only if not already done
            if (scheduledCEE->isPartOfMission()) {
                ExchangeCommand *exchangeCommand = new ExchangeCommand(replacingNode, true, true);
                CommandExecEngine *exchangeCEE = new ExchangeCEE(this, exchangeCommand);
                exchangeCEE->setPartOfMission(false);
                cees.push_front(exchangeCEE);
                EV_INFO << __func__ << "(): ExchangeCEE added to node." << endl;
            }
        }
    }

    // Activate next CEE
    commandExecEngine = cees.front();
    commandExecEngine->setFromCoordinates(getX(), getY(), getZ());
    commandExecEngine->initializeCEE();
    cees.pop_front();

    // Reinject command (if no non-mission command)
    if (commandsRepeat && (commandExecEngine->isPartOfMission()) && not (commandExecEngine->isCeeType(CeeType::TAKEOFF))) {
        cees.push_back(commandExecEngine);
    }
    EV_INFO << "New command loaded is " << commandExecEngine->getCeeTypeString() << " (ID " << commandExecEngine->getCommandId() << ")" << endl;
}

void UAVNode::collectStatistics()
{
    if (commandExecEngine == nullptr) throw cRuntimeError("collectStatistics(): Command Engine missing.");

    double thisCeeDuration = (commandExecEngine->hasDeterminedDuration()) ? commandExecEngine->getOverallDuration() : commandExecEngine->getDuration();
    double thisCeeEnergy = commandExecEngine->getConsumptionPerSecond() * thisCeeDuration;

    if (commandExecEngine->isPartOfMission()) {
        utilizationSecMission += thisCeeDuration;
        utilizationEnergyMission += thisCeeEnergy;
    }
    else {
        utilizationSecMaintenance += thisCeeDuration;
        utilizationEnergyMaintenance += thisCeeEnergy;
    }
}

/**
 * Initialize physical and logical state of the node based on the current CEE.
 * This method is normally called once at the beginning of the CEE execution life cycle.
 * Also update the visible label in the visualization to reflect the current command type of the UAV.
 */
void UAVNode::initializeState()
{
    if (commandExecEngine == nullptr) throw cRuntimeError("initializeState(): Command Engine missing.");

    commandExecEngine->initializeCEE();
    commandExecEngine->performEntryActions();
    commandExecEngine->setNodeParameters();

    std::string text(getFullName());
    switch (commandExecEngine->getCeeType()) {
        case CeeType::WAYPOINT:
            text += " WP";
            break;
        case CeeType::TAKEOFF:
            text += " TO";
            break;
        case CeeType::HOLDPOSITION:
            text += " HP";
            break;
        case CeeType::CHARGE:
            text += " CH";
            break;
        case CeeType::EXCHANGE:
            text += " EX";
            break;
        case CeeType::WAIT:
            text += " WA";
            break;
        default:
            throw cRuntimeError("initializeState(): CEE type not handled for label.");
            break;
    }
    labelNode->setText(text);
    std::string duration = (commandExecEngine->hasDeterminedDuration()) ? std::to_string(commandExecEngine->getOverallDuration()) + "s" : "...s";
    EV_INFO << "Consumption drawn for CEE: " << commandExecEngine->getConsumptionPerSecond() << "mAh/s * " << duration << endl;
}

/*
 * Update physical and logical state of the node based on the current CEE.
 * This method is normally called at every simulation step of the CEE execution life cycle.
 * Also update the visible sublabel in the visualization to reflect the current state of the UAV.
 */
void UAVNode::updateState()
{
    if (commandExecEngine == nullptr) throw cRuntimeError("updateState(): Command Engine missing.");

    //distance to move, based on simulation time passed since last update
    double stepSize = (simTime() - lastUpdate).dbl();
    commandExecEngine->updateState(stepSize);

    //update sublabel with maneuver and battery info
    std::ostringstream strs;
    strs << std::setprecision(1) << std::fixed << speed << " m/s";
    strs << " | " << (-1) * commandExecEngine->getConsumptionPerSecond() << " A";
    strs << " | " << battery.getRemainingPercentage() << " %";
    //strs << " | ";
    //(commandExecEngine->hasDeterminedDuration()) ? strs << commandExecEngine->getRemainingTime() : strs << "...";
    //strs << " s left";
    sublabelNode->setText(strs.str());
    par("stateSummary").setStringValue(labelNode->getText() + " | " + strs.str());
}

/**
 * Check whether or not the current CEE has reached its completion.
 * Depending on the command compares the current position and state of the node with the abort criterion of the command.
 */
bool UAVNode::commandCompleted()
{
    if (commandExecEngine == nullptr) throw cRuntimeError("commandCompleted(): Command Engine missing.");
    return commandExecEngine->commandCompleted();
}

/**
 * Get the time in seconds till the end of current command
 */
double UAVNode::nextNeededUpdate()
{
    if (commandExecEngine == nullptr) throw cRuntimeError("nextNeededUpdate(): Command Engine missing.");
    if (commandExecEngine->hasDeterminedDuration()) {
        return commandExecEngine->getRemainingTime();
    }
    else {
        //TODO unknown? this is just a first workaround!
        return 10;
    }
}

/**
 * Load a queue of commands, generate cees out of these and store them as the cees to be executed by the node.
 */
void UAVNode::loadCommands(CommandQueue commands, bool isMission)
{
    if (not cees.empty()) {
        EV_WARN << __func__ << "()" << "Replacing non-empty CEE queue." << endl;
        cees.clear();
    }

    for (u_int index = 0; index < commands.size(); ++index) {
        Command *command = commands.at(index);
        CommandExecEngine *cee = nullptr;

        if (WaypointCommand *cmd = dynamic_cast<WaypointCommand *>(command)) {
            cee = new WaypointCEE(this, cmd);
        }
        else if (TakeoffCommand *cmd = dynamic_cast<TakeoffCommand *>(command)) {
            cee = new TakeoffCEE(this, cmd);
        }
        else if (HoldPositionCommand *cmd = dynamic_cast<HoldPositionCommand *>(command)) {
            cee = new HoldPositionCEE(this, cmd);
        }
        else if (ChargeCommand *cmd = dynamic_cast<ChargeCommand *>(command)) {
            cee = new ChargeCEE(this, cmd);
        }
        else if (ExchangeCommand *cmd = dynamic_cast<ExchangeCommand *>(command)) {
            cee = new ExchangeCEE(this, cmd);
        }
        else {
            throw cRuntimeError("UAVNode::loadCommands(): invalid cast or unexpected command type.");
        }
        if (not isMission) cee->setPartOfMission(false);
        cee->setCommandId(index);
        cees.push_back(cee);
    }
    EV_INFO << __func__ << "(): " << commands.size() << " commands stored in node memory." << endl;
}

/**
 * Calculate the overall flight time of a CommandQueue.
 * This method will ignore the Repeat property.
 *
 * @return Time needed for the commands in the command queue
 */
double UAVNode::estimateCommandsDuration()
{
    double duration = 0;
    double fromX = this->getX();
    double fromY = this->getY();
    double fromZ = this->getZ();
    for (auto it = cees.begin(); it != cees.end(); ++it) {
        CommandExecEngine *nextCEE = *it;
        nextCEE->setFromCoordinates(fromX, fromY, fromZ);
        nextCEE->initializeCEE();
        duration += nextCEE->getOverallDuration();
        fromX = nextCEE->getX1();
        fromY = nextCEE->getY1();
        fromZ = nextCEE->getZ1();
    }
    return duration;
}

#define IDX_FUTURE_CMDS 0
#define IDX_CMD_ENERGY 1
#define IDX_RETURN_ENERGY 2
#define IDX_CMD_DURATION 3

#define HEURISTIC_LATEST_OPPOTUNITY 0
#define HEURISTIC_SHORTEST_RETURN 1
#define HEURISTIC_UTILIZATION_QUOTIENT 2

/**
 * Iterates over all future CEEs and predicts their consumptions.
 * The consumption plus the needed energy to go back to a charging station are then compared against the remaining battery capacity.
 * Result of the calculation is the feasible amount of commands and the place of last possible replacement.
 *
 * @return ReplacementData for the last point of replacement
 * @return 'nullptr' if a command that can't be estimated (CHARGE or EXCHANGE) is enqueued before depletion
 */
ReplacementData* UAVNode::endOfOperation()
{
    float energySum = 0;
    int nextCommands = 0;
    float nextCommandsDuration = 0;
    bool maxCommandsFeasibleReached = false;
    double fromX = this->getX();
    double fromY = this->getY();
    double fromZ = this->getZ();

    if (cees.empty()) {
        EV_WARN << "endOfOperation(): No CEEs scheduled for node. No end of operation predictable..." << endl;
        return nullptr;
    }

    //TODO Add current Execution Engine consumption
    energySum += 0;

    bool choseClosestCNForExchange = par("choseClosestCNForExchange").boolValue();

    // 0: latest opportunity heuristic
    // 1: shortest return heuristic
    // 2: utilization quotient heuristic
    int replacementMethod = par("replacementMethod");

    if (choseClosestCNForExchange) {

        /*
         *  find closest ChargingNode and Command
         */
        ClosestThings closest = findClosest();

        // exchange at closest command
        fromX = closest.cmd->getX();
        fromY = closest.cmd->getY();
        fromZ = closest.cmd->getZ();
        CommandExecEngine *nextCEE = nullptr;

        /*
         *  calculate energy consumption
         */

        // energy from closest command to closest charging station
        float energyCMDtoCN = estimateEnergy(                                  // estimate energy flying
                closest.cmd->getX(), closest.cmd->getY(), closest.cmd->getZ(), // from closest command
                closest.cn->getX(), closest.cn->getY(), closest.cn->getZ());   // to charging station
        energyCMDtoCN *= 1.2;

        // energy to closest command
        float energyToCMD = 0;
        if (not cmpCoord(*closest.cmd, this->getX(), this->getY(), this->getZ())) {
            do {
                nextCEE = cees.at(nextCommands % cees.size());
                nextCommands++;
                energyToCMD += energyForCEE(nextCEE);
                if (FLT_MAX == energyToCMD) return nullptr;
                nextCommandsDuration += nextCEE->getOverallDuration();
            } while (not cmpCoord(*(nextCEE->extractCommand()), *closest.cmd));
        }
        energyToCMD *= 1.2;

        // estimate energy for one complete mission run
        u_int startingPosition = nextCommands % cees.size();
        float energyMission = 0;
        float durationMission = 0;
        do {
            nextCommands++;
            nextCEE = cees.at(nextCommands % cees.size());
            energyMission += energyForCEE(nextCEE);
            if (FLT_MAX == energyMission) return nullptr;
            durationMission += nextCEE->getOverallDuration();
        } while ((nextCommands % cees.size()) != startingPosition);
        energyMission *= 1.2;

        energySum += energyToCMD + energyCMDtoCN;
        if (energySum > battery.getRemaining()) {
            EV_ERROR << __func__ << "(): " << "Cannot reach closest command and return to charging node." << endl;
            return nullptr; // TODO: implement algorithm to exchange at second closest
        }
        else {
            // how many mission runs can be done
            u_int missionRuns = 0;
            while (energySum + energyMission < battery.getRemaining()) {
                missionRuns++;
                energySum += energyMission;
                nextCommandsDuration += durationMission;
            }
            EV_INFO << __func__ << "(): " << missionRuns << " more missions possible." << endl;
        }
        //EV_INFO << "Finished endOfOperation calculation." << endl;
        ReplacementData *result = new ReplacementData();
        result->nodeToReplace = this;
        result->timeOfReplacement = simTime() + nextCommandsDuration;
        result->x = fromX;
        result->y = fromY;
        result->z = fromZ;
        return result;
    }
    else {
        // Iterates through all feasible future commands and build table of predictions

        /**
         * vector of vectors of {0: number of future commands,
         *                       1: predicted commands energy,
         *                       2: predicted return energy,
         *                       3: predicted commands duration}
         * element 0 initialized with 0s
         */
        std::vector<std::vector<float>> nextCEEsMatrix;
        nextCEEsMatrix.push_back(std::vector<float> { 0.0, 0.0, FLT_MAX, 0.0 });

        double tempFromX = x;
        double tempFromY = y;
        double tempFromZ = z;

        // Preliminary max feasible and energy prediction
        while (not maxCommandsFeasibleReached) {
            CommandExecEngine *nextCEE = cees.at(nextCommands % cees.size());

            nextCEE->setFromCoordinates(tempFromX, tempFromY, tempFromZ);
            tempFromX = nextCEE->getX1();
            tempFromY = nextCEE->getY1();
            tempFromZ = nextCEE->getZ1();

            float energyForNextCEE = energyForCEE(nextCEE);
            float energyToCNAfterCEE = energyToNearestCN(nextCEE->getX1(), nextCEE->getY1(), nextCEE->getZ1());

            //Special case: No end foreseeable
            if (energyForNextCEE == FLT_MAX) return nullptr;

            // Check if next command still feasible
            if (energySum + energyForNextCEE + energyToCNAfterCEE < battery.getRemaining()) {
                // prepare next while loop execution
                nextCommands++;
                energySum += energyForNextCEE;
                nextCommandsDuration += nextCEE->getOverallDuration();
                nextCEEsMatrix.push_back(std::vector<float> { (float) nextCommands, energySum, energyToCNAfterCEE, nextCommandsDuration });
                //EV_INFO << __func__ << "(): Added to list of predictions: matrix entry " << nextCEEsMatrix.size()-1 << ", command " << nextCEE->getCeeTypeString() << nextCommands << ", energy for this CEE " << energyForNextCEE << ", energyToCNAfter " << energyToCNAfterCEE << ", nextCommandsDuration " << nextCommandsDuration << endl;
            }
            else {
                // next command not feasible
                maxCommandsFeasibleReached = true;
            }
        }
        // At least one command has to be feasible
        if (nextCommands == 0) return nullptr;
        EV_INFO << __func__ << "(): " << nextCommands << " commands feasible at most." << endl;

        /**
         * Replacement Heuristics 0..2
         */

        // Replacement planning
        ReplacementData *result = new ReplacementData();
        result->nodeToReplace = this;
        CommandExecEngine *lastCEEofMission;

        switch (replacementMethod) {
            case HEURISTIC_LATEST_OPPOTUNITY: {
                int maxCommandsFeasible = (int) nextCEEsMatrix.back().at(IDX_FUTURE_CMDS);
                lastCEEofMission = cees.at((maxCommandsFeasible - 1) % cees.size());

                float duration = nextCEEsMatrix.back().at(IDX_CMD_DURATION);
                result->timeOfReplacement = simTime() + duration;

                EV_INFO << __func__ << "(): latest opportunity heuristic: " << nextCommands << " commands feasible." << endl;
            }
                break;
            case HEURISTIC_SHORTEST_RETURN: {
                float shortestReturnPathReturnEnergy = FLT_MAX;
                int shortestReturnPathMissionCommands = 0;
                float shortestReturnPathMissionDuration = 0;

                for (auto it = nextCEEsMatrix.begin(); it != nextCEEsMatrix.end(); ++it) {
                    if (it->at(IDX_RETURN_ENERGY) <= shortestReturnPathReturnEnergy) {
                        shortestReturnPathReturnEnergy = it->at(IDX_RETURN_ENERGY);
                        shortestReturnPathMissionCommands = it->at(IDX_FUTURE_CMDS);
                        shortestReturnPathMissionDuration = it->at(IDX_CMD_DURATION);
                    }
                }

                lastCEEofMission = cees.at((shortestReturnPathMissionCommands - 1) % cees.size());
                result->timeOfReplacement = simTime() + shortestReturnPathMissionDuration;

                EV_INFO << __func__ << "(): shortest return heuristic: " << shortestReturnPathMissionCommands << " commands feasible." << endl;
            }
                break;
            case HEURISTIC_UTILIZATION_QUOTIENT: {
                //find shortest
                float shortestReturnPathReturnEnergy = FLT_MAX;
                int shortestReturnPathMissionCommands = 0;
                float shortestReturnPathMissionDuration = 0;

                for (auto it = nextCEEsMatrix.begin(); it != nextCEEsMatrix.end(); ++it) {
                    if (it->at(IDX_RETURN_ENERGY) <= shortestReturnPathReturnEnergy) {
                        shortestReturnPathReturnEnergy = it->at(IDX_RETURN_ENERGY);
                        shortestReturnPathMissionCommands = it->at(IDX_FUTURE_CMDS);
                        shortestReturnPathMissionDuration = it->at(IDX_CMD_DURATION);
                    }
                }
                ASSERT(shortestReturnPathMissionCommands != 0);

                int bestQuotientCommands = 0;
                float bestQuotient = 0.35; // Minimum accepted quotient as initial value
                float bestQuotientDuration = 0;

                // calculate quotients
                for (u_int cmd = shortestReturnPathMissionCommands + 1; cmd < nextCEEsMatrix.size(); ++cmd) {
                    float additionalEnergy = nextCEEsMatrix[cmd].at(IDX_CMD_ENERGY) - shortestReturnPathReturnEnergy;
                    float quotient = nextCEEsMatrix[cmd].at(IDX_RETURN_ENERGY) / (additionalEnergy + nextCEEsMatrix[cmd].at(IDX_RETURN_ENERGY));
                    if (quotient < bestQuotient) {
                        bestQuotient = quotient;
                        bestQuotientCommands = nextCEEsMatrix[cmd].at(IDX_FUTURE_CMDS);
                        bestQuotientDuration = nextCEEsMatrix[cmd].at(IDX_CMD_DURATION);
                    }
                }

                // if no good quotient was found, use shortest (shortest was also latest OR quotient was too bad)
                if (bestQuotientCommands == 0) {
                    bestQuotientCommands = shortestReturnPathMissionCommands;
                    bestQuotientDuration = shortestReturnPathMissionDuration;
                    EV_INFO << __func__ << "(): utilization quotient heuristic: resorting to shortest return heuristic." << endl;
                }

                lastCEEofMission = cees.at((bestQuotientCommands - 1) % cees.size());
                result->timeOfReplacement = simTime() + bestQuotientDuration;

                EV_INFO << __func__ << "(): utilization quotient heuristic: " << bestQuotientCommands << " commands feasible." << endl;
            }
                break;
            default:
                throw omnetpp::cRuntimeError("Invalid replacementMethod selected.");
        }

        result->x = lastCEEofMission->getX1();
        result->y = lastCEEofMission->getY1();
        result->z = lastCEEofMission->getZ1();
        return result;
    }
}

/*
 * Determine the nearest charging node and predict the energy needed to go there.
 *
 * @param The origin coordinates
 * @return The current used by the UAV in [A]
 */
float UAVNode::energyToNearestCN(double fromX, double fromY, double fromZ)
{
// Get consumption for flight to nearest charging node
    ChargingNode *cn = findNearestCN(fromX, fromY, fromZ);
    if (nullptr == cn) throw omnetpp::cRuntimeError("No charging station available!");
    return estimateEnergy(fromX, fromY, fromZ, cn->getX(), cn->getY(), cn->getZ());
}

/**
 * Calculate the electrical consumption for one hover / hold position maneuver (no movement).
 * The calculation is based on predetermined statistical values and a derived gaussian normal distribution.
 *
 * @param duration Duration of the maneuver, in [s]
 * @param fromMethod 0: random  1: mean  2: predictionQuantile
 * @return The current used by the UAV, in [mAh]
 */
double UAVNode::getHoverConsumption(float duration, int fromMethod)
{
    if (duration == 0) return 0;

    double mean = HOVER_MEAN * duration / 3600;
    double var = getVarianceFromHFormula(-1, duration);
    double stddev = sqrt(var);

    double energy = 0;

    if (fromMethod == 0) {
        cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
        do {
            energy = omnetpp::normal(network->getRNG(0), mean, stddev);
        } while (abs(energy - mean) > 3 * stddev || energy < mean / 3);
    }
    else if (fromMethod == 1) {
        energy = mean;
    }
    else {
        energy = boost::math::quantile(boost::math::normal(mean, stddev), quantile);
    }
    return energy / VOLTAGE * 1000;
}

/**
 * Calculate the electrical consumption for one flight maneuver (movement).
 * The speed of the UAV is selected by the UAV and depends on internal parameters and the climb angle.
 * Consequently the power usage of the node is based on these factors.
 * This function will return the consumption based on real measurement values.
 *
 * @param angle The ascent/decline angle, range: -90..+90°
 * @param duration Duration of the maneuver, in [s]
 * @param fromMethod 0: random  1: mean  2: predictionQuantile
 * @return The energy used by the UAV in [mAh]
 */
double UAVNode::getMovementConsumption(float angle, float duration, int fromMethod)
{
    if (duration < 0.001) return 0;

    double mean = 0;
    double stddev = 0;
    double energy = 0;

    ASSERT(angle >= -90.0 && angle <= +90.0);

    for (u_int idx = 1; idx < NUM_ANGLES; idx++) {
        double angle0 = ANGLE2POWER[idx - 1][0];
        double angle1 = ANGLE2POWER[idx][0];

        if ((angle0 <= angle && angle <= angle1)) {
            double mean0 = ANGLE2POWER[idx - 1][1];
            double mean1 = ANGLE2POWER[idx][1];

            mean = mean0 + (mean1 - mean0) / (angle1 - angle0) * (angle - angle0);
            mean = mean * duration / 3600;

            double var0 = getVarianceFromHFormula(idx - 1, duration);
            double var1 = getVarianceFromHFormula(idx, duration);

            double var = var0 + (var1 - var0) / (angle1 - angle0) * (angle - angle0);
            stddev = sqrt(var);
            break;
        }
    }
    ASSERT(mean != 0 && stddev != 0);

    if (fromMethod == 0) {
        cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
        do {
            energy = omnetpp::normal(network->getRNG(0), mean, stddev);
        } while (abs(energy - mean) > 3 * stddev || energy < mean / 3);
    }
    else if (fromMethod == 1) {
        energy = mean;
    }
    else {
        energy = boost::math::quantile(boost::math::normal(mean, stddev), quantile);
    }
    return energy / VOLTAGE * 1000;
}

/**
 * The speed of the UAV is selected by the UAV and depends on internal parameters and the climb angle.
 * This function will return the speed of the node based on real measurement values and the angle the UAV in ascending/declining flight.
 *
 * @param the ascent/decline angle, range: -90..+90°
 * @return the speed of the UAV in [m/s]
 */
double UAVNode::getSpeed(double angle, int fromMethod)
{
    double mean = 0;
    double stddev = 0;
    double speed = 0;

    ASSERT(angle >= -90.0 && angle <= +90.0);

    for (u_int idx = 1; idx < NUM_ANGLES; idx++) {
        double angle0 = ANGLE2SPEED[idx - 1][0];
        double angle1 = ANGLE2SPEED[idx][0];

        if ((angle0 <= angle && angle <= angle1)) {
            double mean0 = ANGLE2SPEED[idx - 1][1];
            double mean1 = ANGLE2SPEED[idx][1];
            double stddev0 = ANGLE2SPEED[idx - 1][2];
            double stddev1 = ANGLE2SPEED[idx][2];

            mean = mean0 + (mean1 - mean0) / (angle1 - angle0) * (angle - angle0);
            stddev = abs(stddev0 + (stddev1 - stddev0) / (angle1 - angle0) * (angle - angle0));
            break;
        }
    }
    ASSERT(mean != 0 && stddev != 0);

    if (fromMethod == 0) {
        cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
        do {
            speed = omnetpp::normal(network->getRNG(0), mean, stddev);
        } while (abs(speed - mean) > 3 * stddev);
    }
    else if (fromMethod == 1) {
        speed = mean;
    }
    else {
        speed = boost::math::quantile(boost::math::normal(mean, stddev), quantile);
    }

    return speed;
}

void UAVNode::move()
{
//unused.
}

ClosestThings UAVNode::findClosest()
{
    ClosestThings result;
    result.cn = nullptr;
    result.cmd = nullptr;
    double distanceClosestCN = DBL_MAX;
    for (unsigned int index = 0; index < cees.size(); index++) {
        Command* command = cees.at(index)->extractCommand();
        ChargingNode* chargingNode = findNearestCN(command->getX(), command->getY(), command->getZ());

        // calculate distance between CN and command
        double distance = sqrt(                                              // distance = square_root of the sum of
                pow((command->getX() - chargingNode->getX()), 2)             //   (CMD_X - CN_X)^2
                + pow((command->getY() - chargingNode->getY()), 2)           //   (CMD_Y - CN_Y)^2
                        + pow((command->getZ() - chargingNode->getZ()), 2)); //  (CMD_Z - CN_Z)^2

        if (distance < distanceClosestCN) {
            result.cn = chargingNode;
            result.cmd = command;
            distanceClosestCN = distance;
        }
    }

    ASSERT(nullptr != result.cn);
    ASSERT(nullptr != result.cmd);

    return result;
}

/**
 * Compares the coordinates (i.e. x, y, z) and return <code>true</code> if and only if all of them
 * are equal, otherwise returns <code>false</code>.
 */
bool UAVNode::cmpCoord(const Command& cmd, const double X, const double Y, const double Z)
{
    return abs(cmd.getX() - X) < ERROR_MARGIN && abs(cmd.getY() - Y) < ERROR_MARGIN && abs(cmd.getZ() - Z) < ERROR_MARGIN;
}

/**
 * Compares the coordinates (i.e. x, y, z) and return <code>true</code> if and only if all of them
 * are equal, otherwise returns <code>false</code>.
 */
bool UAVNode::cmpCoord(const Command& cmd1, const Command& cmd2)
{
    return abs(cmd1.getX() - cmd2.getX()) < ERROR_MARGIN && abs(cmd1.getY() - cmd2.getY()) < ERROR_MARGIN && abs(cmd1.getZ() - cmd2.getZ()) < ERROR_MARGIN;
}

/**
 * Estimates/Predicts the energy consumption for the given CEE.
 */
float UAVNode::energyForCEE(CommandExecEngine* cee)
{
    if (cee->isCeeType(CeeType::WAIT)) {
        return FLT_MAX;
    }

    if (not cee->isPartOfMission()) {
        EV_WARN << __func__ << "(): non-mission command encountered before reaching depletion level. No end of operation predictable..." << endl;
        return FLT_MAX;
    }
//TODO remove the following if the above works
    if (cee->isCeeType(CeeType::CHARGE) || cee->isCeeType(CeeType::EXCHANGE)) {
        throw cRuntimeError("endOfOperation(): charge or exchange command encountered");
    }

// Get consumption for next command
//cee->setFromCoordinates(cee->getX0(), cee->getY0(), cee->getZ0());
//cee->setToCoordinates(cee->getX1(), cee->getY1(), cee->getZ1());
    cee->initializeCEE();
    return cee->predictFullConsumptionQuantile();
}

/**
 * Estimates/Predicts the energy consumption for a waypoint command
 * from the given coordinate (i.e. fromX, fromY, fromZ)
 * to the given coordinate (i.e. toX, toY, toZ). The fakeNode is required but not altered nor read.
 */
float UAVNode::estimateEnergy(double fromX, double fromY, double fromZ, double toX, double toY, double toZ)
{
    WaypointCommand estimateCommand(toX, toY, toZ);
    WaypointCEE estimateCEE(this, &estimateCommand);
    estimateCEE.setFromCoordinates(fromX, fromY, fromZ);
    estimateCEE.setPartOfMission(false);
    estimateCEE.initializeCEE();
    return estimateCEE.predictFullConsumptionQuantile();
}

/**
 * Estimates/Predicts the time needed for a waypoint command
 * from the given coordinate (i.e. fromX, fromY, fromZ)
 * to the given coordinate (i.e. toX, toY, toZ). The fakeNode is required but not altered nor read.
 */
double UAVNode::estimateDuration(double fromX, double fromY, double fromZ, double toX, double toY, double toZ)
{
    WaypointCommand estimateCommand(toX, toY, toZ);
    WaypointCEE estimateCEE(this, &estimateCommand);
    estimateCEE.setFromCoordinates(fromX, fromY, fromZ);
    estimateCEE.setPartOfMission(false);
    estimateCEE.initializeCEE();
    return estimateCEE.getOverallDuration();
}

#endif // WITH_OSG
