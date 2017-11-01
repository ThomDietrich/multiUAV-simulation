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
WaypointCEE::WaypointCEE(UAVNode &boundNode, WaypointCommand &command)
{
    this->node = &boundNode;
    this->command = &command;
    this->setType(CeeType::WAYPOINT);
    setFromCoordinates(node->x, node->y, node->z);
    setToCoordinates(command.getX(), command.getY(), command.getZ());
}

bool WaypointCEE::commandCompleted()
{
    double distanceSum = fabs(x1 - node->x) + fabs(y1 - node->y) + fabs(z1 - node->z);
    return (distanceSum < 1.e-10);
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

    //update and store yaw, climbAngle and pitch angles
    yaw = atan2(dy, dx) / M_PI * 180;
    if (yaw < 0) yaw += 360;
    climbAngle = atan2(dz, sqrt(dx * dx + dy * dy)) / M_PI * 180;
    pitch = (-1) * climbAngle;

    //update speed based on flight angle
    speed = node->getSpeed(climbAngle);

    // draw probable value for consumption of this CEE
    consumptionPerSecond = getProbableConsumption(true, NAN);
}

void WaypointCEE::setNodeParameters()
{
    node->yaw = yaw;
    node->pitch = pitch;
    node->climbAngle = climbAngle;
    node->speed = speed;
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

double WaypointCEE::getDuration()
{
    double dx = x1 - x0;
    double dy = y1 - y0;
    double dz = z1 - z0;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance / speed;
}

double WaypointCEE::getRemainingTime()
{
    double dx = x1 - node->x;
    double dy = y1 - node->y;
    double dz = z1 - node->z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance / speed;
}

double WaypointCEE::getProbableConsumption(bool normalized, float percentile)
{
    double dx = x1 - x0;
    double dy = y1 - y0;
    double dz = z1 - z0;
    double duration = sqrt(dx * dx + dy * dy + dz * dz) / speed;
    //EV_INFO << "Distance expected = " << sqrt(dx * dx + dy * dy + dz * dz) << "m, Time expected = " << duration << "s" << endl;
    double completeConsumption = node->getMovementConsumption(climbAngle, duration, percentile);
    if (normalized) {
        return completeConsumption / duration;
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
TakeoffCEE::TakeoffCEE(UAVNode& boundNode, TakeoffCommand& command)
{
    this->node = &boundNode;
    this->command = &command;
    this->setType(CeeType::TAKEOFF);
    setFromCoordinates(node->x, node->y, node->z);
    setToCoordinates(node->x, node->y, command.getZ());
}

bool TakeoffCEE::commandCompleted()
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
    consumptionPerSecond = getProbableConsumption(true, NAN);
}

void TakeoffCEE::setNodeParameters()
{
    node->pitch = pitch;
    node->climbAngle = climbAngle;
    node->speed = speed;
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

double TakeoffCEE::getDuration()
{
    return fabs(z1 - z0) / speed;
}

double TakeoffCEE::getRemainingTime()
{
    return fabs(z1 - node->z) / speed;
}

double TakeoffCEE::getProbableConsumption(bool normalized, float percentile)
{
    double duration = fabs(z1 - z0) / speed;
    double completeConsumption = node->getMovementConsumption(climbAngle, duration, percentile);
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
HoldPositionCEE::HoldPositionCEE(UAVNode& boundNode, HoldPositionCommand& command)
{
    this->node = &boundNode;
    this->command = &command;
    this->setType(CeeType::HOLDPOSITION);
    setFromCoordinates(node->x, node->y, node->z);
    setToCoordinates(node->x, node->y, node->z);
    this->holdPositionTill = simTime() + this->command->getHoldSeconds();
}

bool HoldPositionCEE::commandCompleted()
{
    if (simTime() > this->holdPositionTill) throw cRuntimeError("Unexpected situation: HoldPosition lasted longer than intended.");
    return (simTime() == this->holdPositionTill) ? true : false;
}

void HoldPositionCEE::initializeCEE()
{
    //yaw = yaw;
    pitch = 0;
    climbAngle = 0;

    // draw probable value for consumption of this CEE
    consumptionPerSecond = getProbableConsumption(true, NAN);
}

void HoldPositionCEE::setNodeParameters()
{
    //node->yaw = yaw;
    node->pitch = pitch;
    node->climbAngle = climbAngle;
}

void HoldPositionCEE::updateState(double stepSize)
{
    node->battery.discharge(consumptionPerSecond * stepSize);
}

double HoldPositionCEE::getDuration()
{
    return (this->command->getHoldSeconds());
}

double HoldPositionCEE::getRemainingTime()
{
    return (this->holdPositionTill - simTime()).dbl();
}

double HoldPositionCEE::getProbableConsumption(bool normalized, float percentile)
{
    double duration = this->command->getHoldSeconds();
    double completeConsumption = node->getHoverConsumption(duration, percentile);
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
ChargeCEE::ChargeCEE(UAVNode& boundNode, ChargeCommand& command)
{
    this->node = &boundNode;
    this->command = &command;
    this->setType(CeeType::CHARGE);
    this->setFromCoordinates(node->x, node->y, node->z);
    this->setToCoordinates(node->x, node->y, node->z);
}

bool ChargeCEE::commandCompleted()
{
    return (node->battery.isFull());
}

void ChargeCEE::initializeCEE()
{
    pitch = 0;
    climbAngle = 0;
    //TODO connect to Charging station
    //cMessage *request = new cMessage("startCharge");
    //this->command->getChargingNode()->scheduleAt(simTime(), request);
    // draw probable value for consumption of this CEE
    consumptionPerSecond = getProbableConsumption(true, NAN);
}

void ChargeCEE::setNodeParameters()
{
    node->pitch = pitch;
    node->climbAngle = climbAngle;
}

void ChargeCEE::updateState(double stepSize)
{
    float chargeAmount = fabs(consumptionPerSecond * stepSize);
    node->battery.charge(chargeAmount);

    // Charging state log report
    int statusReport = 20; // reported these much values between 0..100%
    float statusReportChargeSteps = (node->battery.getCapacity() / statusReport);
    //EV_DEBUG << "chargeAmount " << chargeAmount << " statusReportChargeSteps " << statusReportChargeSteps << " node->battery.getRemaining() "
    //        << node->battery.getRemaining() << " fmod " << fmod(node->battery.getRemaining(), statusReportChargeSteps) << endl;
    if (fmod(node->battery.getRemaining(), statusReportChargeSteps) < chargeAmount) {
        EV_INFO << "Energy Management: Recharging... " << node->battery.getRemainingPercentage() << "%" << endl;
    }
}

double ChargeCEE::getDuration()
{
    return node->battery.getCapacity() / fabs(consumptionPerSecond);
}

double ChargeCEE::getRemainingTime()
{
    return node->battery.getMissing() / fabs(consumptionPerSecond);
}

double ChargeCEE::getProbableConsumption(bool normalized, float percentile)
{
    if (normalized == false) EV_WARN << __func__ << "(): non-normalized currently not supported for ChargeCEE" << endl;
    if (isnormal(percentile)) EV_WARN << __func__ << "(): percentile not supported for ExchangeCEE" << endl;
    // 3DR Solo: 5200 mAh in 1.5h = 3.5A constant -> 3.5Ah = 3500 mAh -> 0,97222 mAh / s
    return -0.97222;
}

char* ChargeCEE::getCeeTypeString()
{
    return (char*) "Charge";
}

/**
 * Exchange Command Execution Engine
 */
ExchangeCEE::ExchangeCEE(UAVNode& boundNode, ExchangeCommand& command)
{
    this->node = &boundNode;
    this->command = &command;
    this->setType(CeeType::EXCHANGE);
    setFromCoordinates(node->x, node->y, node->z);
    setToCoordinates(node->x, node->y, node->z);
}

bool ExchangeCEE::commandCompleted()
{
    return false;
}

void ExchangeCEE::initializeCEE()
{
    //yaw = yaw;
    pitch = 0;
    climbAngle = 0;

    // draw probable value for consumption of this CEE
    consumptionPerSecond = getProbableConsumption(true, NAN);
}

void ExchangeCEE::setNodeParameters()
{
    //node->yaw = yaw;
    node->pitch = pitch;
    node->climbAngle = climbAngle;
}

void ExchangeCEE::updateState(double stepSize)
{
    node->battery.discharge(consumptionPerSecond * stepSize);
}

double ExchangeCEE::getDuration()
{
    throw cRuntimeError("ExchangeCEE has no determined ending time");
    return 1;
}

double ExchangeCEE::getRemainingTime()
{
    throw cRuntimeError("ExchangeCEE has no determined ending time");
    return 1;
}

double ExchangeCEE::getProbableConsumption(bool normalized, float percentile)
{
    //TODO duration unknown
    if (normalized == false) EV_WARN << __func__ << "(): non-normalized not supported for ExchangeCEE" << endl;
    if (isnormal(percentile)) EV_WARN << __func__ << "(): percentile not supported for ExchangeCEE" << endl;
    return node->getHoverConsumption(1, 0.5);
}

char* ExchangeCEE::getCeeTypeString()
{
    return (char*) "Exchange";
}

void ExchangeCEE::performEntryActions()
{
    EV_INFO << __func__ << "(): Ready for exchange, notifying other Node (" << command->getOtherNode()->getFullName() << ")" << endl;

    // Send an exchangeInitialize message to the other node taking part in the exchange
    cMessage *exInitMsg = new cMessage("exchangeInitialize");
    cGate* gateToNode = node->getOutputGateTo(command->getOtherNode());
    node->send(exInitMsg, gateToNode);
}

void ExchangeCEE::performExitActions()
{
}

GenericNode* ExchangeCEE::getOtherNode()
{
    return command->getOtherNode();
}
