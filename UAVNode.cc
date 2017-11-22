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

    float energyForSheduled = scheduledCEE->predictFullConsumption(0.75);
    float energyToCNNow = energyToNearestCN(getX(), getY(), getZ());
    float energyToCNAfterScheduled = energyToNearestCN(scheduledCEE->getX1(), scheduledCEE->getY1(), scheduledCEE->getZ1());
    float energyRemaining = this->battery.getRemaining();

    //EV_INFO << "Remaining Energy in Battery=" << remaining << "mAh " << endl;

    // Elect and activate the next command/CEE
    if (scheduledCEE->getCeeType() == CeeType::CHARGE) {
        EV_INFO << "Energy Management: Recharging now." << endl;
    }
    else if (battery.isEmpty()) {
        EV_ERROR << "Energy Management: One of our precious UAVs just died :-(" << endl;
        //throw cRuntimeError("Energy Management: One of our precious UAVs just died :-(");
    }
    else if (energyRemaining >= energyForSheduled + energyToCNAfterScheduled) {
        EV_INFO << "Energy Management: OK. UAV has enough energy to continue (" << std::setprecision(1) << std::fixed << this->battery.getRemainingPercentage()
                << "%)." << endl;
    }
    else {
        // Go To Charging Node now
        if (energyRemaining < energyToCNNow) {
            EV_WARN << "Energy Management: Going to Charging Node. Attention! Energy insufficient (" << energyRemaining << " < " << energyToCNNow << " mAh)."
                    << endl;
        }
        else {
            EV_INFO << "Energy Management: Going to Charging Node (" << std::setprecision(1) << std::fixed << this->battery.getRemainingPercentage() << "%)."
                    << endl;
        }

        // Find nearest ChargingNode
        ChargingNode *cn = findNearestCN(getX(), getY(), getZ());

        // Generate WaypointCEE
        WaypointCommand *goToChargingNode = new WaypointCommand(cn->getX(), cn->getY(), cn->getZ());
        CommandExecEngine *goToChargingNodeCEE = new WaypointCEE(*this, *goToChargingNode);

        /* Todo:
         * Add values to parameter 1 (estimtaded arrival time)
         * Add real value to parameter 2 (estimated consumption till arrival at ChargingNode)
         * The reserve message is not needed for charging nodes, but will make the forecasts of the node more reliable.
         * Furthermore an earlier reservation will lead to an earlier charging.
         */
        // Generate and send reservation message to CN
//        cMessage *reserveSpot = new ReserveSpotMsg(simTime()+goToChargingNodeCEE->getDuration(), goToChargingNodeCEE->getProbableConsumption(), this);
//        this->send(reserveSpot, this->getOutputGateTo(cn));
// Generate ChargeCEE
        ChargeCommand *chargeCommand = new ChargeCommand(cn);
        ChargeCEE *chargeCEE = new ChargeCEE(*this, *chargeCommand);

        // Add WaypointCEE and ChargeCEE to the CEEs queue
        cees.push_front(chargeCEE);
        cees.push_front(goToChargingNodeCEE);
    }

    commandExecEngine = cees.front();
    commandExecEngine->setFromCoordinates(getX(), getY(), getZ());
    commandExecEngine->initializeCEE();
    cees.pop_front();

    // reinject command (if no charging or takeoff command)
    if (commandsRepeat && not (commandExecEngine->getCeeType() == CeeType::TAKEOFF) //
            && not (commandExecEngine->getCeeType() == CeeType::CHARGE) //
            && not (commandExecEngine->getCeeType() == CeeType::EXCHANGE) && not (commandExecEngine->getCeeType() == CeeType::WAIT)) {
        cees.push_back(commandExecEngine);
    }
}

/**
 * Initialize physical and logical state of the node based on the current CEE.
 * This method is normally called once at the beginning of the CEE execution life cycle.
 * Also update the visible label in the visualization to reflect the current command type of the UAV.
 */
void UAVNode::initializeState()
{
    if (commandExecEngine == nullptr) {
        throw cRuntimeError("initializeState(): Command Engine missing.");
    }
    commandExecEngine->initializeCEE();
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
    }
    labelNode->setText(text);
    std::string duration = (commandExecEngine->hasDeterminedDuration()) ? std::to_string(commandExecEngine->getDuration()) + "s" : "...s";
    EV_INFO << "Consumption drawn for CEE: " << commandExecEngine->getConsumptionPerSecond() << "mAh/s * " << duration << endl;
}

/*
 * Update physical and logical state of the node based on the current CEE.
 * This method is normally called at every simulation step of the CEE execution life cycle.
 * Also update the visible sublabel in the visualization to reflect the current state of the UAV.
 */
void UAVNode::updateState()
{
    //distance to move, based on simulation time passed since last update
    double stepSize = (simTime() - lastUpdate).dbl();
    commandExecEngine->updateState(stepSize);

    //update sublabel with maneuver and battery info
    std::ostringstream strs;
    strs << std::setprecision(1) << std::fixed << speed << " m/s";
    strs << " | " << commandExecEngine->getConsumptionPerSecond() << " A";
    strs << " | " << battery.getRemainingPercentage() << " %";
    strs << " | ";
    (commandExecEngine->hasDeterminedDuration()) ? strs << commandExecEngine->getRemainingTime() : strs << "...";
    strs << " s left";
    std::string str = strs.str();
    sublabelNode->setText(str);
}

/**
 * Check whether or not the current CEE has reached its completion.
 * Depending on the command compares the current position and state of the node with the abort criterion of the command.
 */
bool UAVNode::commandCompleted()
{
    return commandExecEngine->commandCompleted();
}

/**
 * Get the time in seconds till the end of current command
 */
double UAVNode::nextNeededUpdate()
{
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
void UAVNode::loadCommands(CommandQueue commands)
{
    if (not cees.empty()) {
        EV_WARN << "Replacing non-empty CEE queue." << endl;
        cees.clear();
    }

    for (u_int index = 0; index < commands.size(); ++index) {
        Command *command = commands.at(index);
        CommandExecEngine *cee = nullptr;

        if (WaypointCommand *cmd = dynamic_cast<WaypointCommand *>(command)) {
            cee = new WaypointCEE(*this, *cmd);
        }
        else if (TakeoffCommand *cmd = dynamic_cast<TakeoffCommand *>(command)) {
            cee = new TakeoffCEE(*this, *cmd);
        }
        else if (HoldPositionCommand *cmd = dynamic_cast<HoldPositionCommand *>(command)) {
            cee = new HoldPositionCEE(*this, *cmd);
        }
        else if (ChargeCommand *cmd = dynamic_cast<ChargeCommand *>(command)) {
            cee = new ChargeCEE(*this, *cmd);
        }
        else if (ExchangeCommand *cmd = dynamic_cast<ExchangeCommand *>(command)) {
            cee = new ExchangeCEE(*this, *cmd);
        }
        else {
            throw cRuntimeError("UAVNode::loadCommands(): invalid cast or unexpected command type.");
        }
        cee->setCommandId(index);
        cees.push_back(cee);
    }
}

/**
 * Delete the current commands/CEEs from nodes memory
 */
void UAVNode::clearCommands()
{
    cees.clear();
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
        duration += nextCEE->getDuration();
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
    int commandsFeasible = 0;
    float durrationOfCommands = 0;
    double fromX = this->getX();
    double fromY = this->getY();
    double fromZ = this->getZ();

    //TODO Add current Execution Engine consumption
    energySum += 0;

    while (true) {
        CommandExecEngine *nextCEE = cees.at(commandsFeasible % cees.size());

        if (nextCEE->getCeeType() == CeeType::CHARGE) {
            //throw cRuntimeError("endOfOperation(): charge command encountered");
            EV_WARN << "endOfOperation(): charge command encountered" << endl;
            return nullptr;
        }
        if (nextCEE->getCeeType() == CeeType::EXCHANGE) {
            EV_WARN << "endOfOperation(): exchange command encountered" << endl;
            return nullptr;
        }

        // Get consumption for next command
        nextCEE->setFromCoordinates(nextCEE->getX0(), nextCEE->getY0(), nextCEE->getZ0());
        nextCEE->setToCoordinates(nextCEE->getX1(), nextCEE->getY1(), nextCEE->getZ1());
        nextCEE->initializeCEE();

        float energyForNextCEE = nextCEE->predictFullConsumption(0.75);

        // Get consumption for going back to the nearest Charging node
        energyToCNAfter = energyToNearestCN(nextCEE->getX1(), nextCEE->getY1(), nextCEE->getZ1());

        //EV_DEBUG << "Consumption Aggregated=" << energySum << "mAh" << endl;
        EV_DEBUG << commandsFeasible + 1 << " Consumption Command " << nextCEE->getCommandId() << ": " << energyForNextCEE << "mAh" << endl;
        //EV_DEBUG << "Consumption GoToChargingNode=" << energyToCNAfter << "mAh" << endl;
        //EV_DEBUG << "Consumption Aggregated + Command + GoToChargingNode=" << energySum + energyForNextCEE + energyToCNAfter << "mAh" << endl;
        //EV_DEBUG << "Consumption Battery Remaining=" << battery.getRemaining() << "mAh" << endl;

        if (battery.getRemaining() < energySum + energyForNextCEE + energyToCNAfter) {
            EV_DEBUG << "Maximum distance exceeded. " << commandsFeasible << " commands feasible." << endl;
            break;
        }
        else {
            //EV_DEBUG << "Command " << commandsFeasible << " still feasible." << endl;
        }
        commandsFeasible++;
        energySum += energyForNextCEE;
        durrationOfCommands += nextCEE->getDuration();

        fromX = nextCEE->getX1();
        fromY = nextCEE->getY1();
        fromZ = nextCEE->getZ1();
    }
    //EV_INFO << "Finished endOfOperation calculation." << endl;
    ReplacementData *result = new ReplacementData();
    result->nodeToReplace = this;
    result->timeOfReplacement = simTime() + durrationOfCommands;
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
    WaypointCommand *goToChargingNode = new WaypointCommand(cn->getX(), cn->getY(), cn->getZ());
    CommandExecEngine *goToChargingNodeCEE = new WaypointCEE(*this, *goToChargingNode);
    goToChargingNodeCEE->setFromCoordinates(fromX, fromY, fromZ);
    goToChargingNodeCEE->initializeCEE();
    return goToChargingNodeCEE->predictFullConsumption(0.75);
}

void UAVNode::move()
{
    //unused.
}

#endif // WITH_OSG
