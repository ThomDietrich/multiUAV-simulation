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

void CommandExecEngine::setType(ceeType type)
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
    this->setType(WAYPOINT);
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
    double dx = x1 - node->x;
    double dy = y1 - node->y;
    double dz = z1 - node->z;
    
    //update and store yaw, climbAngle and pitch angles
    yaw = atan2(dy, dx) / M_PI * 180;
    if (yaw < 0) yaw += 360;
    climbAngle = atan2(dz, sqrt(dx * dx + dy * dy)) / M_PI * 180;
    pitch = (-1) * climbAngle;
    
    //update speed based on flight angle
    speed = node->getSpeedFromAngle(climbAngle);
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
    
    node->battery.discharge(getCurrent() * 1000 * stepSize / 3600); // Q = I * t [mAh]
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

double WaypointCEE::getCurrent()
{
    return node->getCurrentFromAngle(climbAngle);
}

double WaypointCEE::predictConsumption()
{
    double dx = x1 - x0;
    double dy = y1 - y0;
    double dz = z1 - z0;
    double time = sqrt(dx * dx + dy * dy + dz * dz) / speed;
    //EV_INFO << "Distance expected = " << sqrt(dx * dx + dy * dy + dz * dz) << "m, Time expected = " << time << "s" << endl;
    return (getCurrent() * 1000 * time / 3600);
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
    this->setType(TAKEOFF);
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
    climbAngle = (z1 > node->z) ? 90 : -90;
    
    //update speed based on flight angle
    speed = node->getSpeedFromAngle(climbAngle);
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
    
    node->battery.discharge(getCurrent() * 1000 * stepSize / 3600);
}

double TakeoffCEE::getDuration()
{
    return fabs(z1 - z0) / speed;
}

double TakeoffCEE::getRemainingTime()
{
    return fabs(z1 - node->z) / speed;
}

double TakeoffCEE::getCurrent()
{
    return node->getCurrentFromAngle(climbAngle);
}

double TakeoffCEE::predictConsumption()
{
    double overallTime = fabs(z1 - z0) / speed;
    return (getCurrent() * 1000 * overallTime / 3600);
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
    this->setType(HOLDPOSITION);
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
}

void HoldPositionCEE::setNodeParameters()
{
    //node->yaw = yaw;
    node->pitch = pitch;
    node->climbAngle = climbAngle;
}

void HoldPositionCEE::updateState(double stepSize)
{
    node->battery.discharge(getCurrent() * 1000 * stepSize / 3600);
}

double HoldPositionCEE::getDuration()
{
    return (this->command->getHoldSeconds());
}

double HoldPositionCEE::getRemainingTime()
{
    return (this->holdPositionTill - simTime()).dbl();
}

double HoldPositionCEE::getCurrent()
{
    return node->getCurrentHover();
}

double HoldPositionCEE::predictConsumption()
{
    return (getCurrent() * 1000 * this->command->getHoldSeconds() / 3600);
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
    this->setType(CHARGE);
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
}

void ChargeCEE::setNodeParameters()
{
    node->pitch = pitch;
    node->climbAngle = climbAngle;
}

void ChargeCEE::updateState(double stepSize)
{
    float chargeAmount = getCurrent() * 1000 * stepSize / 3600;
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
    return node->battery.getCapacity() / (getCurrent() * 1000) * 3600;
}

double ChargeCEE::getRemainingTime()
{
    return node->battery.getMissing() / (getCurrent() * 1000) * 3600;
}

double ChargeCEE::getCurrent()
{
// 3DR Solo: 5200 mAh in 1.5h = 3.5A constant
    return 3.5;
}

char* ChargeCEE::getCeeTypeString()
{
    return (char*) "Charge";
}
