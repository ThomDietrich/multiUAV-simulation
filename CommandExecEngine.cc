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

#include <CommandExecEngine.h>
#include "UAVNode.h"

using namespace omnetpp;

void CommandExecEngine::setType(ceeType type) {
    this->type = type;
}

/**
 * Waypoint Command Execution Engine
 *
 * TODO: Take speed variable from command into account
 */
WaypointCEE::WaypointCEE(UAVNode &boundNode, WaypointCommand &command) {
    this->node = &boundNode;
    this->command = &command;
    this->setType(WAYPOINT);
    setFromCoordinates(node->x, node->y, node->z);
}

bool WaypointCEE::commandCompleted() {
    double distanceSum = fabs(command->getX() - node->getX()) + fabs(command->getY() - node->getY()) + fabs(command->getZ() - node->getZ());
    return (distanceSum < 1.e-10);
}

void WaypointCEE::initializeState() {
    //absolute distance to next waypoint, in meters
    if (this->command == nullptr) {
        throw cRuntimeError("initializeState(): Command missing.");
    }
    double dx = command->getX() - node->x;
    double dy = command->getY() - node->y;
    double dz = command->getZ() - node->z;
    
    //update and store yaw and pitch angles
    node->yaw = atan2(dx, -dy) / M_PI * 180;
    node->pitch = atan2(dz, sqrt(dx * dx + dy * dy)) / M_PI * 180;
}

void WaypointCEE::updateState(double stepSize) {
    //distance to move, based on simulation time passed since last update
    double stepDistance = node->speed * stepSize;
    
    //resulting movement broken down to x,y,z
    double stepZ = stepDistance * sin(M_PI * node->pitch / 180);
    double stepXY = stepDistance * cos(M_PI * node->pitch / 180);
    double stepX = stepXY * sin(M_PI * node->yaw / 180);
    double stepY = stepXY * -cos(M_PI * node->yaw / 180);
    node->x += stepX;
    node->y += stepY;
    node->z += stepZ;
    
    // TODO: testing energy consumption
    node->battery.discharge(getCurrent() * stepSize / 3600);
}
double WaypointCEE::getRemainingTime() {
    double dx = command->getX() - node->x;
    double dy = command->getY() - node->y;
    double dz = command->getZ() - node->z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance / this->node->speed;
}

double WaypointCEE::getCurrent() {
    //TODO depending on angle and speed
    return 15000;
}

double WaypointCEE::predictConsumption() {
    //TODO node Position may not be the starting point of this command!
    double dx = command->getX() - this->x0;
    double dy = command->getY() - this->y0;
    double dz = command->getZ() - this->z0;
    double remainingTime = sqrt(dx * dx + dy * dy + dz * dz) / this->node->speed;
    return (getCurrent() * remainingTime / 3600);
}

char* WaypointCEE::getCeeTypeString() {
    return "Waypoint";
}

/**
 * Takeoff Command Execution Engine
 */
TakeoffCEE::TakeoffCEE(UAVNode& boundNode, TakeoffCommand& command) {
    this->node = &boundNode;
    this->command = &command;
    this->setType(TAKEOFF);
    setFromCoordinates(node->x, node->y, node->z);
}

bool TakeoffCEE::commandCompleted() {
    double distanceSum = fabs(command->getZ() - node->z);
    return (distanceSum < 1.e-10);
}

void TakeoffCEE::initializeState() {
//node->yaw = node->yaw;
    node->pitch = 0;
}

void TakeoffCEE::updateState(double stepSize) {
    double stepDistance = node->speed * stepSize;
    if (command->getZ() > node->z)
        node->z += stepDistance;
    else
        node->z -= stepDistance;
    
//some animation, remove if irritating
    node->yaw = node->yaw + 5;
    
// TODO: testing energy consumption
    node->battery.discharge(getCurrent() * stepSize / 3600);
}

double TakeoffCEE::getRemainingTime() {
    return fabs(command->getZ() - node->z) / this->node->speed;
}

double TakeoffCEE::getCurrent() {
//TODO depending on angle and speed
    return 20000;
}

double TakeoffCEE::predictConsumption() {
//TODO node->z may not be the starting point of this command!
    double remainingTime = fabs(command->getZ() - this->z0) / this->node->speed;
    return (getCurrent() * remainingTime / 3600);
}

char* TakeoffCEE::getCeeTypeString() {
    return "Take Off";
}

/**
 * HoldPosition Command Execution Engine
 */
HoldPositionCEE::HoldPositionCEE(UAVNode& boundNode, HoldPositionCommand& command) {
    this->node = &boundNode;
    this->command = &command;
    this->setType(HOLDPOSITION);
    setFromCoordinates(node->x, node->y, node->z);
    this->holdPositionTill = simTime() + this->command->getHoldSeconds();
}

bool HoldPositionCEE::commandCompleted() {
    return (simTime() == this->holdPositionTill) ? true : false;
}

void HoldPositionCEE::initializeState() {
//node->yaw = node->yaw;
    node->pitch = 0;
}

void HoldPositionCEE::updateState(double stepSize) {
//some animation, remove if irritating
    node->yaw = node->yaw + 5;
    
    node->battery.discharge(getCurrent() * stepSize / 3600);
}

double HoldPositionCEE::getRemainingTime() {
    return (this->holdPositionTill - simTime()).dbl();
}

double HoldPositionCEE::getCurrent() {
//TODO just an example value
    return 12000;
}

double HoldPositionCEE::predictConsumption() {
    return (getCurrent() * this->command->getHoldSeconds() / 3600);
}

char* HoldPositionCEE::getCeeTypeString() {
    return "Hold Position";
}
