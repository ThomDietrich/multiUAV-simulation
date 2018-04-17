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

/**
 * Compares the coordinates (i.e. x, y, z) and return <code>true</code> if and only if all of them
 * are equal, otherwise returns <code>false</code>.
 */
bool cmpCoord(const Command& cmd, const double X, const double Y, const double Z);

/**
 * Compares the coordinates (i.e. x, y, z) and return <code>true</code> if and only if all of them
 * are equal, otherwise returns <code>false</code>.
 */
bool cmpCoord(const Command& cmd1, const Command& cmd2);

/**
 * Estimates/Predicts the energy consumption for the given CEE.
 */
float energyForCEE(CommandExecEngine* cee);

/**
 * Estimates/Predicts the energy consumption for a waypoint command
 * from the given coordinate (i.e. fromX, fromY, fromZ)
 * to the given coordinate (i.e. toX, toY, toZ). The fakeNode is required but not altered nor read.
 */
float estimateEnergy(UAVNode& fakeNode, double fromX, double fromY, double fromZ, double toX, double toY, double toZ);

/**
 * Estimates/Predicts the time needed for a waypoint command
 * from the given coordinate (i.e. fromX, fromY, fromZ)
 * to the given coordinate (i.e. toX, toY, toZ). The fakeNode is required but not altered nor read.
 */
double estimateDuration(UAVNode* fakeNode, double fromX, double fromY, double fromZ, double toX, double toY, double toZ);

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
            z = 2;
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
            clearCommands();
            missionId = receivedMissionMsg->getMissionId();
            commandsRepeat = receivedMissionMsg->getMissionRepeat();
            CommandQueue missionCommands = receivedMissionMsg->getMission();
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
            float energyCMDtoCN = estimateEnergy(*this,                             // estimate energy flying
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
        float energyForSheduled = scheduledCEE->predictFullConsumption(0.75);
        float energyToCNNow = energyToNearestCN(getX(), getY(), getZ());
        float energyToCNAfterScheduled = energyToNearestCN(scheduledCEE->getX1(), scheduledCEE->getY1(), scheduledCEE->getZ1());
        float energyRemaining = this->battery.getRemaining();

        if (scheduledCEE->getCeeType() == CeeType::CHARGE) {
            EV_INFO << "Energy Management: Recharging now." << endl;
        }
        else if (battery.isEmpty()) {
            EV_ERROR << "Energy Management: One of our precious UAVs just died :-(" << endl;
            //throw cRuntimeError("Energy Management: One of our precious UAVs just died :-(");
        }
        else if (energyRemaining >= energyForSheduled + energyToCNAfterScheduled) {
            EV_INFO << "Energy Management: OK. UAV has enough energy to continue";
            EV_INFO << " (" << std::setprecision(1) << std::fixed << this->battery.getRemainingPercentage() << "%)." << endl;
        }
        else {
            // Start Replacement Process now
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
    EV_INFO << "New command is " << commandExecEngine->getCeeTypeString() << ", remaining CEEs: " << cees.size() << endl;
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
    strs << " | " << commandExecEngine->getConsumptionPerSecond() << " A";
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
        EV_WARN << "Replacing non-empty CEE queue." << endl;
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

/**
 * Calculate the electrical consumption for one hover / hold position maneuver (no movement).
 * The calculation is based on predetermined statistical values and a derived gaussian normal distribution.
 * A specific percentile value can be calculated, if the parameter is left out a random value is drawn.
 *
 * @param duration Duration of the maneuver, in [s]
 * @param percentile The percentile in the range 0..1
 * @return The current used by the UAV, in [mAh]
 */
double UAVNode::getHoverConsumption(float duration, float percentile)
{
    double mean = 18.09;  // mean current, in [A]
    double stddev = 0.36; // deviation of the mean, in [A]
    if (isnan(percentile)) {
        cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
        return omnetpp::normal(network->getRNG(0), mean, stddev) * 1000 * duration / 3600;
    }
    else if (percentile < 0.0 || percentile > 1.0) {
        throw cRuntimeError("Invalid percentile outside range [0.0..1.0].");
    }
    else {
        return boost::math::quantile(boost::math::normal(mean, stddev), percentile) * 1000 * duration / 3600;
    }
}

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
    float energyToCNAfter = 0;
    int nextCommandsFeasible = 0;
    float durationOfCommands = 0;
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
        float energyCMDtoCN = estimateEnergy(*this,       // estimate energy flying
                closest.cmd->getX(), closest.cmd->getY(), closest.cmd->getZ(), // from closest command
                closest.cn->getX(), closest.cn->getY(), closest.cn->getZ());   // to charging station
        energyCMDtoCN *= 1.2;

        // energy to closest command
        float energyToCMD = 0;
        if (not cmpCoord(*closest.cmd, this->getX(), this->getY(), this->getZ())) {
            do {
                nextCEE = cees.at(nextCommandsFeasible % cees.size());
                nextCommandsFeasible++;
                energyToCMD += energyForCEE(nextCEE);
                if (FLT_MAX == energyToCMD) return nullptr;
                durationOfCommands += nextCEE->getOverallDuration();
            } while (not cmpCoord(*(nextCEE->extractCommand()), *closest.cmd));
        }
        energyToCMD *= 1.2;

        // estimate energy for one complete mission run
        u_int startingPosition = nextCommandsFeasible % cees.size();
        float energyMission = 0;
        float durationMission = 0;
        do {
            nextCommandsFeasible++;
            nextCEE = cees.at(nextCommandsFeasible % cees.size());
            energyMission += energyForCEE(nextCEE);
            if (FLT_MAX == energyMission) return nullptr;
            durationMission += nextCEE->getOverallDuration();
        } while ((nextCommandsFeasible % cees.size()) != startingPosition);
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
                durationOfCommands += durationMission;
            }
            EV_INFO << __func__ << "(): " << missionRuns << " more missions possible." << endl;
        }
    }
    else
        while (true) {
            CommandExecEngine *nextCEE = cees.at(nextCommandsFeasible % cees.size());

            float energyForNextCEE = energyForCEE(nextCEE);
            if (FLT_MAX == energyForNextCEE) return nullptr;

            // Get consumption for going back to the nearest Charging node
            energyToCNAfter = energyToNearestCN(nextCEE->getX1(), nextCEE->getY1(), nextCEE->getZ1());

            //EV_DEBUG << "Consumption Aggregated=" << energySum << "mAh" << endl;
            //EV_DEBUG << commandsFeasible + 1 << " Consumption Command " << nextCEE->getCommandId() << ": " << energyForNextCEE << "mAh" << endl;
            //EV_DEBUG << "Consumption GoToChargingNode=" << energyToCNAfter << "mAh" << endl;
            //EV_DEBUG << "Consumption Aggregated + Command + GoToChargingNode=" << energySum + energyForNextCEE + energyToCNAfter << "mAh" << endl;
            //EV_DEBUG << "Consumption Battery Remaining=" << battery.getRemaining() << "mAh" << endl;

            if (battery.getRemaining() < energySum + energyForNextCEE + energyToCNAfter) {
                EV_INFO << __func__ << "(): " << nextCommandsFeasible << " commands feasible." << endl;
                break;
            }
            nextCommandsFeasible++;
            energySum += energyForNextCEE;
            durationOfCommands += nextCEE->getOverallDuration();

            fromX = nextCEE->getX1();
            fromY = nextCEE->getY1();
            fromZ = nextCEE->getZ1();
        }

//EV_INFO << "Finished endOfOperation calculation." << endl;
    ReplacementData *result = new ReplacementData();
    result->nodeToReplace = this;
    result->timeOfReplacement = simTime() + durationOfCommands;
    result->x = fromX;
    result->y = fromY;
    result->z = fromZ;
    return result;
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
    return estimateEnergy(*this, fromX, fromY, fromZ, cn->getX(), cn->getY(), cn->getZ());
}

/**
 * Calculate the electrical consumption for one flight maneuver (movement).
 * The speed of the UAV is selected by the UAV and depends on internal parameters and the climb angle.
 * Consequently the power usage of the node is based on these factors.
 * This function will return the consumption based on real measurement values.
 * A specific percentile value can be calculated, if the parameter is left out a random value is drawn.
 *
 * @param angle The ascent/decline angle, range: -90..+90°
 * @param duration Duration of the maneuver, in [s]
 * @param percentile The percentile in the range 0..1
 * @return The current used by the UAV in [mAh]
 */
double UAVNode::getMovementConsumption(float angle, float duration, float percentile)
{
    double mean;
    double stddev;
    const u_int numAngles = 11;
    double samples[numAngles][3] = {
// angle [°], mean current [A], current deviation [A]
            { -90.0, 16.86701, 0.7651131 }, //
            { -75.6, 17.97695, 0.7196844 }, //
            { -57.9, 17.34978, 0.6684724 }, //
            { -34.8, 17.34384, 0.8729401 }, //
            { -15.6, 15.99054, 1.1767867 }, //
            { 000.0, 16.36526, 1.0290515 }, //
            { +15.6, 18.83829, 2.1043467 }, //
            { +34.8, 20.34726, 1.4018145 }, //
            { +57.9, 21.31561, 0.8680334 }, //
            { +75.6, 21.43493, 0.7625244 }, //
            { +90.0, 20.86530, 0.7350855 }  //
    };
//Catch exactly -90°
    if (angle == samples[0][0]) {
        mean = samples[0][1];
        stddev = samples[0][2];
    }
    else if (angle < -90.0 || angle > +90.0) {
        throw cRuntimeError("Invalid angle outside range [-90.0..+90.0].");
    }
    else {
        // simple linear interpolation
        for (u_int idx = 1; idx < numAngles; idx++) {
            if (samples[idx - 1][0] < angle && angle <= samples[idx][0]) {
                double mean_slope = (samples[idx][1] - samples[idx - 1][1]) / (samples[idx][0] - samples[idx - 1][0]);
                mean = samples[idx - 1][1] + mean_slope * (angle - samples[idx - 1][0]);

                double stddev_slope = (samples[idx][2] - samples[idx - 1][2]) / (samples[idx][0] - samples[idx - 1][0]);
                stddev = samples[idx - 1][2] + stddev_slope * (angle - samples[idx - 1][0]);
            }
        }
    }
    if (isnan(percentile)) {
        cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
        return omnetpp::normal(network->getRNG(0), mean, stddev) * 1000 * duration / 3600;
    }
    else if (percentile < 0.0 || percentile > 1.0) {
        throw cRuntimeError("Invalid percentile outside range [0.0..1.0].");
    }
    else {
        return boost::math::quantile(boost::math::normal(mean, stddev), percentile) * 1000 * duration / 3600;
    }
}

/**
 * The speed of the UAV is selected by the UAV and depends on internal parameters and the climb angle.
 * This function will return the speed of the node based on real measurement values and the angle the UAV in ascending/declining flight.
 *
 * @param the ascent/decline angle, range: -90..+90°
 * @return the speed of the UAV in [m/s]
 */
double UAVNode::getSpeed(double angle)
{
    const u_int numAngles = 11;
    double samples[numAngles][2] = {
// angle [°], mean speed [m/s]
            { -90.0, 1.837303 }, //
            { -75.6, 1.842921 }, //
            { -57.9, 2.013429 }, //
            { -34.8, 2.450476 }, //
            { -15.6, 3.583821 }, //
            { 000.0, 8.056741 }, //
            { +15.6, 6.020143 }, //
            { +34.8, 3.337107 }, //
            { +57.9, 2.822109 }, //
            { +75.6, 2.719016 }, //
            { +90.0, 2.719048 }  //
    };
//Catch exactly -90°
    if (angle == samples[0][0]) return samples[0][1];

// simple linear interpolation
    for (u_int idx = 1; idx < numAngles; idx++) {
        if (samples[idx - 1][0] < angle && angle <= samples[idx][0]) {
            double slope = (samples[idx][1] - samples[idx - 1][1]) / (samples[idx][0] - samples[idx - 1][0]);
            double interpol = samples[idx - 1][1] + slope * (angle - samples[idx - 1][0]);
            return interpol;
        }
    }

    std::stringstream ss;
    ss << "UAVNode::getSpeedFromAngle() unexpected angle passed: " << angle;
    const char* str = ss.str().c_str();

    throw cRuntimeError(str);
    return 0;
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
bool cmpCoord(const Command& cmd, const double X, const double Y, const double Z)
{
    return abs(cmd.getX() - X) < ERROR_MARGIN && abs(cmd.getY() - Y) < ERROR_MARGIN && abs(cmd.getZ() - Z) < ERROR_MARGIN;
}

/**
 * Compares the coordinates (i.e. x, y, z) and return <code>true</code> if and only if all of them
 * are equal, otherwise returns <code>false</code>.
 */
bool cmpCoord(const Command& cmd1, const Command& cmd2)
{
    return abs(cmd1.getX() - cmd2.getX()) < ERROR_MARGIN && abs(cmd1.getY() - cmd2.getY()) < ERROR_MARGIN && abs(cmd1.getZ() - cmd2.getZ()) < ERROR_MARGIN;
}

/**
 * Estimates/Predicts the energy consumption for the given CEE.
 */
float energyForCEE(CommandExecEngine* cee)
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
    cee->setFromCoordinates(cee->getX0(), cee->getY0(), cee->getZ0());
    cee->setToCoordinates(cee->getX1(), cee->getY1(), cee->getZ1());
    cee->initializeCEE();
    return cee->predictFullConsumption(0.75);
}

/**
 * Estimates/Predicts the energy consumption for a waypoint command
 * from the given coordinate (i.e. fromX, fromY, fromZ)
 * to the given coordinate (i.e. toX, toY, toZ). The fakeNode is required but not altered nor read.
 */
float estimateEnergy(UAVNode& fakeNode, double fromX, double fromY, double fromZ, double toX, double toY, double toZ)
{
    WaypointCommand estimateCommand(toX, toY, toZ);
    WaypointCEE estimateCEE(&fakeNode, &estimateCommand);
    estimateCEE.setFromCoordinates(fromX, fromY, fromZ);
    estimateCEE.setPartOfMission(false);
    estimateCEE.initializeCEE();
    return estimateCEE.predictFullConsumption(0.75);
}

double estimateDuration(UAVNode* fakeNode, double fromX, double fromY, double fromZ, double toX, double toY, double toZ)
{
    WaypointCommand estimateCommand(toX, toY, toZ);
    WaypointCEE estimateCEE(fakeNode, &estimateCommand);
    estimateCEE.setFromCoordinates(fromX, fromY, fromZ);
    estimateCEE.setPartOfMission(false);
    estimateCEE.initializeCEE();
    return estimateCEE.getOverallDuration();
}

#endif // WITH_OSG
