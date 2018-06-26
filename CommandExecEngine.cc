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

#include "CommandExecEngine.h"
#include "UAVNode.h"

using namespace omnetpp;

void CommandExecEngine::setType(CeeType type)
{
    this->type = type;
}

/**
 * Waypoint Command Execution Engine
 */
WaypointCEE::WaypointCEE(UAVNode *boundNode, WaypointCommand *command)
{
    this->node = boundNode;
    this->command = command;
    this->setType(CeeType::WAYPOINT);
    setFromCoordinates(node->x, node->y, node->z);
    setToCoordinates(command->getX(), command->getY(), command->getZ());
}

bool WaypointCEE::isCommandCompleted()
{
    double distanceSum = fabs(x1 - node->x) + fabs(y1 - node->y) + fabs(z1 - node->z);
    return commandCompleted || (distanceSum < 1.e-10);
}

void WaypointCEE::initializeCEE()
{
    //absolute distance to next waypoint, in meters
    if (this->command == nullptr) {
        throw cRuntimeError("initializeCEE(): Command missing.");
    }
    double dx = x1 - x0;
    double dy = y1 - y0;
    double dz = z1 - z0;
    if (abs(dx) < 1.e-10) dx = 0;
    if (abs(dy) < 1.e-10) dy = 0;
    if (abs(dz) < 1.e-10) dz = 0;

    //update and store yaw, climbAngle and pitch angles
    yaw = atan2(dy, dx) / M_PI * 180;
    if (yaw < 0) yaw += 360;
    climbAngle = atan2(dz, sqrt(dx * dx + dy * dy)) / M_PI * 180;
    pitch = (-1) * climbAngle;

    //update speed based on flight angle
    speed = node->getSpeed(climbAngle);

    // draw probable random value for consumption of this CEE
    consumptionPerSecond = predictNormConsumptionRandom();
}

void WaypointCEE::setNodeParameters()
{
    node->yaw = yaw;
    node->pitch = pitch;
    node->climbAngle = climbAngle;
    node->speed = speed;
    timeExecutionStart = simTime();
}

void WaypointCEE::updateState(double stepSize)
{
    //distance to move, based on simulation time passed since last update (in [m])
    double stepDistance = stepSize * speed;

    //resulting movement broken down to x,y,z (in [m])
    double stepZ = stepDistance * sin(M_PI * climbAngle / 180);
    double stepXY = stepDistance * cos(M_PI * climbAngle / 180);
    double stepX = stepXY * cos(M_PI * yaw / 180);
    double stepY = stepXY * sin(M_PI * yaw / 180);
    node->x += stepX;
    node->y += stepY;
    node->z += stepZ;

    node->battery.discharge(consumptionPerSecond * stepSize);
}

double WaypointCEE::getOverallDuration()
{
    double dx = x1 - x0;
    double dy = y1 - y0;
    double dz = z1 - z0;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    if (distance < 1.e-10) distance = 0;
    return distance / speed;
}

double WaypointCEE::getRemainingTime()
{
    double dx = x1 - node->x;
    double dy = y1 - node->y;
    double dz = z1 - node->z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    if (distance < 1.e-10) distance = 0;
    return distance / speed;
}

double WaypointCEE::getProbableConsumption(bool normalized, int fromMethod)
{
    double dx = x1 - x0;
    double dy = y1 - y0;
    double dz = z1 - z0;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    if (distance < 1.e-10) distance = 0;
    double completeConsumption = node->getMovementConsumption(climbAngle, distance / speed, fromMethod);
    //EV_INFO << "Distance expected = " << sqrt(dx * dx + dy * dy + dz * dz) << "m, Time expected = " << duration << "s, fromMethod" << fromMethod << ", Consumption expected = " << completeConsumption << "mAh" << endl;
    if (normalized) {
        return completeConsumption / (distance / speed);
    }
    else {
        return completeConsumption;
    }
}

char* WaypointCEE::getCeeTypeString()
{
    return (char*) "Waypoint";
}

/**
 * Takeoff Command Execution Engine
 */
TakeoffCEE::TakeoffCEE(UAVNode *boundNode, TakeoffCommand *command)
{
    this->node = boundNode;
    this->command = command;
    this->setType(CeeType::TAKEOFF);
    setFromCoordinates(node->x, node->y, node->z);
    setToCoordinates(node->x, node->y, command->getZ());
}

bool TakeoffCEE::isCommandCompleted()
{
    double distanceSum = fabs(z1 - node->z);
    return (distanceSum < 1.e-10);
}

void TakeoffCEE::initializeCEE()
{
    pitch = 0;
    climbAngle = (z1 > z0) ? 90 : -90;

    //update speed based on flight angle
    speed = node->getSpeed(climbAngle);

    // draw probable value for consumption of this CEE
    consumptionPerSecond = predictNormConsumptionRandom();
}

void TakeoffCEE::setNodeParameters()
{
    //node->yaw = yaw;
    node->pitch = pitch;
    node->climbAngle = climbAngle;
    node->speed = speed;
    timeExecutionStart = simTime();
}

void TakeoffCEE::updateState(double stepSize)
{
    double stepDistance = speed * stepSize;
    if (z1 > node->z)
        node->z += stepDistance;
    else
        node->z -= stepDistance;

    node->battery.discharge(consumptionPerSecond * stepSize);
}

double TakeoffCEE::getOverallDuration()
{
    return fabs(z1 - z0) / speed;
}

double TakeoffCEE::getRemainingTime()
{
    return fabs(z1 - node->z) / speed;
}

double TakeoffCEE::getProbableConsumption(bool normalized, int fromMethod)
{
    double duration = fabs(z1 - z0) / speed;
    double completeConsumption = node->getMovementConsumption(climbAngle, duration, fromMethod);
    if (normalized) {
        return completeConsumption / duration;
    }
    else {
        return completeConsumption;
    }
}

char* TakeoffCEE::getCeeTypeString()
{
    return (char*) "Take Off";
}

/**
 * HoldPosition Command Execution Engine
 */
HoldPositionCEE::HoldPositionCEE(UAVNode *boundNode, HoldPositionCommand *command)
{
    this->node = boundNode;
    this->command = command;
    this->setType(CeeType::HOLDPOSITION);
    setFromCoordinates(node->x, node->y, node->z);
    setToCoordinates(command->getX(), command->getY(), command->getZ());
}

bool HoldPositionCEE::isCommandCompleted()
{
    if (simTime() > this->holdPositionTill) throw cRuntimeError("Unexpected situation: HoldPosition lasted longer than intended.");
    return commandCompleted || (simTime() == this->holdPositionTill) ? true : false;
}

void HoldPositionCEE::initializeCEE()
{
    this->holdPositionTill = simTime() + command->getHoldSeconds();

    // draw probable value for consumption of this CEE
    consumptionPerSecond = getProbableConsumption(true, 0);
}

void HoldPositionCEE::setNodeParameters()
{
    //node->yaw = yaw;
    node->pitch = 0;
    node->climbAngle = 0;
    node->speed = 0;
    timeExecutionStart = simTime();
}

void HoldPositionCEE::updateState(double stepSize)
{
    node->battery.discharge(consumptionPerSecond * stepSize);
}

double HoldPositionCEE::getOverallDuration()
{
    return (this->command->getHoldSeconds());
}

double HoldPositionCEE::getRemainingTime()
{
    return (this->holdPositionTill - simTime()).dbl();
}

double HoldPositionCEE::getProbableConsumption(bool normalized, int fromMethod)
{
    double duration = this->command->getHoldSeconds();
    double completeConsumption = node->getHoverConsumption(duration, fromMethod);
    if (normalized) {
        return completeConsumption / duration;
    }
    else {
        return completeConsumption;
    }
}

char* HoldPositionCEE::getCeeTypeString()
{
    return (char*) "Hold Position";
}

/**
 * Charge Command Execution Engine
 *
 * @param boundNode
 * @param command
 */
ChargeCEE::ChargeCEE(UAVNode *boundNode, ChargeCommand *command)
{
    this->node = boundNode;
    this->command = command;
    this->setType(CeeType::CHARGE);
    this->setFromCoordinates(node->x, node->y, node->z);
    this->setToCoordinates(node->x, node->y, node->z);
}

bool ChargeCEE::isCommandCompleted()
{
    return commandCompleted || (node->battery.isFull());
}

void ChargeCEE::initializeCEE()
{
}

void ChargeCEE::setNodeParameters()
{
    // simple hack to orient each UAV randomly
    node->yaw = (node->battery.getRemainingPercentage() / 10 * 360) % 360;
    node->pitch = 0;
    node->climbAngle = 0;
    node->speed = 0;
//    cMessage *request = new cMessage("startCharge");
//    node->send(request, node->getOutputGateTo(command->getChargingNode()));
    timeExecutionStart = simTime();
}

void ChargeCEE::updateState(double stepSize)
{
}

double ChargeCEE::getOverallDuration()
{
    // ToDo: should and can this integrate the forecast from charging station?
    throw cRuntimeError("ChargeCEE has no determined ending time");
    return 1;
}

double ChargeCEE::getRemainingTime()
{
    throw cRuntimeError("ChargeCEE has no determined ending time");
    return 1;
}

double ChargeCEE::getProbableConsumption(bool normalized, int fromMethod)
{
    return 0;
}

char* ChargeCEE::getCeeTypeString()
{
    return (char*) "Charge";
}

/**
 * Exchange Command Execution Engine
 */
ExchangeCEE::ExchangeCEE(UAVNode *boundNode, ExchangeCommand *command)
{
    this->node = boundNode;
    this->command = command;
    this->setType(CeeType::EXCHANGE);
    setFromCoordinates(node->x, node->y, node->z);
    setToCoordinates(node->x, node->y, node->z);
}

bool ExchangeCEE::isCommandCompleted()
{
    return commandCompleted;
}

void ExchangeCEE::initializeCEE()
{
    // draw probable value for consumption of this CEE
    consumptionPerSecond = predictNormConsumptionRandom();
}

void ExchangeCEE::setNodeParameters()
{
    // simple hack to orient each UAV randomly
    node->yaw = (node->battery.getRemainingPercentage() / 10 * 360) % 360;
    node->pitch = 0;
    node->climbAngle = 0;
    node->speed = 0;
    timeExecutionStart = simTime();
}

void ExchangeCEE::updateState(double stepSize)
{
    node->battery.discharge(consumptionPerSecond * stepSize);
}

double ExchangeCEE::getOverallDuration()
{
    throw cRuntimeError("ExchangeCEE has no determined ending time");
    return 1;
}

double ExchangeCEE::getRemainingTime()
{
    throw cRuntimeError("ExchangeCEE has no determined ending time");
    return 1;
}

double ExchangeCEE::getProbableConsumption(bool normalized, int fromMethod)
{
    //TODO duration unknown
    if (normalized == false) EV_WARN << __func__ << "(): non-normalized not supported for ExchangeCEE" << endl;

    int duration = 1;
    return node->getHoverConsumption(duration, 1);
}

char* ExchangeCEE::getCeeTypeString()
{
    return (char*) "Exchange";
}

void ExchangeCEE::performEntryActions()
{
    if (this->command->isRechargeRequested()) {
        if (not command->isOtherNodeKnown()) {
            EV_ERROR << __func__ << "(): No other node for " << node->getFullName() << "'s exchange command." << endl;
            return;
        }

        EV_INFO << __func__ << "(): Ready for exchange, sending data to other Node (" << command->getOtherNode()->getFullName() << ")" << endl;

        // Send an exchangeData message to the other node taking part in the exchange
        UAVNode *otherNode = check_and_cast<UAVNode *>(command->getOtherNode());
        node->transferMissionDataTo(otherNode);
    }

}

void ExchangeCEE::performExitActions()
{
    if (command->isRechargeRequested()) {
        // Find nearest ChargingNode
        ChargingNode *cn = UAVNode::findNearestCN(node->getX(), node->getY(), node->getZ());

        // Generate WaypointCEE
        WaypointCommand *goToChargingNodeCommand = new WaypointCommand(cn->getX(), cn->getY(), cn->getZ());
        WaypointCEE *goToChargingNodeCEE = new WaypointCEE(node, goToChargingNodeCommand);
        goToChargingNodeCEE->setPartOfMission(false);
        goToChargingNodeCEE->setNoReplacementNeeded();

        // Get the duration for the flight to ChargingNode
        // To get the information the CEE needs to be initialized
        goToChargingNodeCEE->initializeCEE();
        double goToChargingNodeDuration = goToChargingNodeCEE->getOverallDuration();

        // Generate and send reservation message to CN
        ReserveSpotMsg *msg = new ReserveSpotMsg("reserveSpot");
        msg->setEstimatedArrival(simTime() + goToChargingNodeDuration);
        msg->setConsumptionTillArrival(goToChargingNodeCEE->getProbableConsumption());
        msg->setTargetPercentage(100.0);
        node->send(msg, node->getOutputGateTo(cn));

        // Generate ChargeCEE
        ChargeCommand *chargeCommand = new ChargeCommand(cn);
        CommandExecEngine *chargeCEE = new ChargeCEE(node, chargeCommand);
        chargeCEE->setToCoordinates(cn->getX(), cn->getY(), cn->getZ());
        chargeCEE->setPartOfMission(false);
        chargeCEE->setNoReplacementNeeded();

        // Add WaypointCEE and ChargeCEE to the CEEs queue
        node->cees.push_front(chargeCEE);
        node->cees.push_front(goToChargingNodeCEE);

        EV_INFO << __func__ << "(): GoToChargingNode and Charge CEE added to node." << endl;
    }
}

GenericNode* ExchangeCEE::getOtherNode()
{
    return command->getOtherNode();
}

/**
 *  Todo: Review weather the WaitCEE should be simplyfied
 *  Currently there is no drawn comsumption during waiting
 */
IdleCEE::IdleCEE(MobileNode *boundNode, IdleCommand *command)
{
    this->node = boundNode;
    this->command = command;
    this->setType(CeeType::IDLE);
    this->setFromCoordinates(node->getX(), node->getY(), node->getZ());
    this->setToCoordinates(node->getX(), node->getY(), node->getZ());
}

bool IdleCEE::isCommandCompleted()
{
    return commandCompleted;
}

void IdleCEE::initializeCEE()
{
    // draw probable value for consumption of this CEE
    consumptionPerSecond = predictNormConsumptionRandom();
}

void IdleCEE::setNodeParameters()
{
    // simple hack to orient each UAV randomly
    //node->yaw = (node->battery.getRemainingPercentage() / 10 * 360) % 360;
    //node->pitch = 0;
    //node->climbAngle = 0;
    //node->speed = 0;
    timeExecutionStart = simTime();
}

void IdleCEE::updateState(double stepSize)
{
    node->getBattery()->discharge(consumptionPerSecond * stepSize);
}

double IdleCEE::getOverallDuration()
{
    throw cRuntimeError("IdleCEE has no determined ending time");
    return 1;
}

double IdleCEE::getRemainingTime()
{
    throw cRuntimeError("IdleCEE has no determined ending time");
    return 1;
}

double IdleCEE::getProbableConsumption(bool normalized, int fromMethod)
{
    return 0;
}

char* IdleCEE::getCeeTypeString()
{
    return (char*) "Idle";
}
