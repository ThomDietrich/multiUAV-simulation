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
#include "UAVNode.h"
#include "OsgEarthScene.h"
#include "ChannelController.h"
#include <fstream>
#include <iostream>
#include <sstream>

using namespace omnetpp;

Define_Module(UAVNode);

UAVNode::UAVNode() {
}

UAVNode::~UAVNode() {
}

/**
 * Simulation initialization with two stages, i.e. setup cycles
 * @param stage
 */
void UAVNode::initialize(int stage) {
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
void UAVNode::selectNextCommand() {

    if (commands.size() == 0) {
        throw cRuntimeError("selectNextCommand(): UAV has no commands left.");
    }

    double remaining = this->battery.getRemaining();
    //EV_INFO << "Remaining Energy in Battery=" << remaining << "mAh " << endl;
    
    //
    // Get consumption to go to CS now
    //
    ChargingNode *cn = findNearestCN(getX(), getY(), getZ());
    WaypointCommand *goToChargingNode = new WaypointCommand(cn->getX(), cn->getY(), cn->getZ());
    //EV_INFO << "Consumption GoToChargingNode UAV(" << getX() << "," << getY() << "," << getZ() << ") CN(" << cn->getX() << "," << cn->getY() << "," << cn->getZ() << ")" << endl;
    CommandExecEngine *goToChargingNodeCEE = new WaypointCEE(*this, *goToChargingNode);
    goToChargingNodeCEE->initializeCEE();
    double predGoToChargingNodeCEE = goToChargingNodeCEE->predictConsumption();
    //EV_INFO << "Consumption GoToChargingNode=" << predGoToChargingNodeCEE << "mAh " << endl;
    
    //
    // Select next Command -> CEE
    //
    CommandExecEngine *scheduledCEE = nullptr;
    if (WaypointCommand *command = dynamic_cast<WaypointCommand *>(commands.front())) {
        scheduledCEE = new WaypointCEE(*this, *command);
    }
    else if (TakeoffCommand *command = dynamic_cast<TakeoffCommand *>(commands.front())) {
        scheduledCEE = new TakeoffCEE(*this, *command);
    }
    else if (HoldPositionCommand *command = dynamic_cast<HoldPositionCommand *>(commands.front())) {
        scheduledCEE = new HoldPositionCEE(*this, *command);
    }
    else if (ChargeCommand *command = dynamic_cast<ChargeCommand *>(commands.front())) {
        scheduledCEE = new ChargeCEE(*this, *command);
    }
    else
    throw cRuntimeError("loadNextCommand(): invalid cast.");

    //
    // Get consumption for next Command
    //
    scheduledCEE->initializeCEE();
    double predScheduledCEE = scheduledCEE->predictConsumption();
    //EV_INFO << "Consumption "<< scheduledCEE->getCeeTypeString() << " command=" << predScheduledCEE << "mAh, " << endl;
    
    //
    // Get consumption for going to charging node after next command
    //
    ChargingNode *cn2 = findNearestCN(scheduledCEE->getX1(), scheduledCEE->getY1(), scheduledCEE->getZ1());
    WaypointCommand *goToChargingNode2 = new WaypointCommand(cn2->getX(), cn2->getY(), cn2->getZ());
    CommandExecEngine *goToChargingNodeCEE2 = new WaypointCEE(*this, *goToChargingNode2);
    goToChargingNodeCEE2->setFromCoordinates(scheduledCEE->getX1(), scheduledCEE->getY1(), scheduledCEE->getZ1());

    goToChargingNodeCEE2->initializeCEE();
    double predGoToChargingNodeCEE2 = goToChargingNodeCEE2->predictConsumption();
    //EV_INFO << "Consumption GoToChargingNode (after Command)=" << predGoToChargingNodeCEE2 << "mAh" << endl;
    //EV_INFO << "Consumption Command + GoToChargingNode=" << predScheduledCEE + predGoToChargingNodeCEE2 << "mAh" << endl;
    
    //
    // Elect and activate the next command/CEE
    //
    delete commandExecEngine;

    if (scheduledCEE->getCeeType() == CHARGE) {
        EV_INFO << "Energy Management: Recharging now." << endl;
    }
    else if (battery.isEmpty()) {
        EV_ERROR << "Energy Management: One of our precious UAVs just died :-(" << endl;
        //throw cRuntimeError("Energy Management: One of our precious UAVs just died :-(");
    }
    else if (remaining >= predScheduledCEE + predGoToChargingNodeCEE2) {
        // remaining energy for next command + going to charging node + more
        EV_INFO << "Energy Management: OK. UAV has enough energy to continue." << endl;
    }
    else {    //if (remaining >= predGoToChargingNodeCEE) {
              // remaining energy to at least go to the charging node
        EV_WARN << "Energy Management: UAV will go to charging station " << cn->getIndex() << " now." << endl;
              // Inject charging command after scheduled command
        ChargeCommand *chargeCommand = new ChargeCommand(cn);
        Command *scheduledCommand = commands.front();
        commands.pop_front();
        commands.push_front(chargeCommand);
        commands.push_front(scheduledCommand);
        scheduledCEE = goToChargingNodeCEE;
    }
    //else if (remaining < predGoToChargingNodeCEE) {
    //    EV_WARN << "Energy Management: Too late! The UAV is too far from the nearest ChargingNode. (" << remaining << "<" << predGoToChargingNodeCEE << " mAh)" << endl;
    //}
    //else {
    //    throw cRuntimeError("Energy Management: Electing next command based on energy consumption failed.");
    //}
    commandExecEngine = scheduledCEE;
    commands.pop_front();

}

void UAVNode::initializeState() {
    if (commandExecEngine == nullptr) {
        throw cRuntimeError("initializeState(): Command Engine missing.");
    }
    commandExecEngine->initializeCEE();
    commandExecEngine->setNodeParameters();

    std::string text(getFullName());
    switch (commandExecEngine->getCeeType()) {
        case WAYPOINT:
        text += " WP";
        break;
        case TAKEOFF:
        text += " TO";
        break;
        case HOLDPOSITION:
        text += " HP";
        break;
        case CHARGE:
        text += " CH";
        break;
    }
    labelNode->setText(text);
}

void UAVNode::updateState() {
//distance to move, based on simulation time passed since last update
    double stepSize = (simTime() - lastUpdate).dbl();
    commandExecEngine->updateState(stepSize);

//update sublabel with battery info
    std::ostringstream strs;
    strs << std::setprecision(1) << std::fixed << speed << " m/s | " << commandExecEngine->getCurrent() << " A | " << battery.getRemainingPercentage() << " % | " << commandExecEngine->getRemainingTime() << " s left";
    std::string str = strs.str();
    sublabelNode->setText(str);
}

bool UAVNode::commandCompleted() {
    return commandExecEngine->commandCompleted();
}

/**
 * Get the time in seconds till the end of current command
 */
double UAVNode::nextNeededUpdate() {
    return commandExecEngine->getRemainingTime();
}

/**
 * The speed of the UAV is selected by the UAV and depends on internal parameters and the climb angle
 * This function will return the speed of the node based on real measurement values and the angle the UAV in ascending/declining flight.
 * @return the speed of the UAV in [m/s]
 */
double UAVNode::getSpeedFromAngle(double angle) {
    double samples[11][2] = { //
        {   -90.0, 1.837303}, //
        {   -75.6, 1.842921}, //
        {   -57.9, 2.013429}, //
        {   -34.8, 2.450476}, //
        {   -15.6, 3.583821}, //
        {   000.0, 8.056741}, //
        {   +15.6, 6.020143}, //
        {   +34.8, 3.337107}, //
        {   +57.9, 2.822109}, //
        {   +75.6, 2.719016}, //
        {   +90.0, 2.719048}  //
    };

//Catch exactly -90°
    if (angle == samples[0][0]) return samples[0][1];

// simple linear interpolation
    for (int idx = 1; idx < sizeof(samples); ++idx) {
        if (samples[idx - 1][0] < angle && angle <= samples[idx][0]) {
            double slope = (samples[idx][1] - samples[idx - 1][1]) / (samples[idx][0] - samples[idx - 1][0]);
            double interpol = samples[idx - 1][1] + slope * (angle - samples[idx - 1][0]);
            //TODO add deviation
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
 * The speed of the UAV is selected by the UAV and depends on internal parameters and the climb angle
 * Consequently the power usage of the node is based on these factors.
 * This function will return the battery current drain based on real measurement values and the angle the UAV in ascending/declining flight.
 * @return the current used by the UAV in [A]
 */
double UAVNode::getCurrentFromAngle(double angle) {
    double samples[11][3] = { //
        {   -90.0, 16.86701, 0.7651131}, //
        {   -75.6, 17.97695, 0.7196844}, //
        {   -57.9, 17.34978, 0.6684724}, //
        {   -34.8, 17.34384, 0.8729401}, //
        {   -15.6, 15.99054, 1.1767867}, //
        {   000.0, 16.36526, 1.0290515}, //
        {   +15.6, 18.83829, 2.1043467}, //
        {   +34.8, 20.34726, 1.4018145}, //
        {   +57.9, 21.31561, 0.8680334}, //
        {   +75.6, 21.43493, 0.7625244}, //
        {   +90.0, 20.86530, 0.7350855}  //
    };

//Catch exactly -90°
    if (angle == samples[0][0]) return samples[0][1];

// simple linear interpolation
    for (int idx = 1; idx < sizeof(samples); ++idx) {
        if (samples[idx - 1][0] < angle && angle <= samples[idx][0]) {
            double mean_slope = (samples[idx][1] - samples[idx - 1][1]) / (samples[idx][0] - samples[idx - 1][0]);
            double mean = samples[idx - 1][1] + mean_slope * (angle - samples[idx - 1][0]);

            double stddev_slope = (samples[idx][2] - samples[idx - 1][2]) / (samples[idx][0] - samples[idx - 1][0]);
            double stddev = samples[idx - 1][2] + stddev_slope * (angle - samples[idx - 1][0]);

            cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
            double result = omnetpp::normal(network->getRNG(0), mean, stddev);

            return result;
        }
    }

    std::stringstream ss;
    ss << "UAVNode::getCurrentFromAngle() unexpected angle passed: " << angle;
    const char* str = ss.str().c_str();

    throw cRuntimeError(str);
    return 0;
}

double UAVNode::getCurrentHover() {
    double mean = 18.09;
    double stddev = 0.36;
    cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
    double result = omnetpp::normal(network->getRNG(0), mean, stddev);
    return result;
}

void UAVNode::move() {
//unused.
}

#endif // WITH_OSG
