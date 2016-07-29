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

/**
 * Waypoint Command Execution Engine
 */
WaypointCEE::WaypointCEE(UAVNode &node, WaypointCommand &command)
{
    this->node = &node;
    this->command = &command;
}

bool WaypointCEE::commandCompleted()
{
    double distanceSum = fabs(command->getX() - node->getX()) + fabs(command->getY() - node->getY()) + fabs(command->getZ() - node->getZ());
    EV_INFO << "UAV #" << this->node->getIndex() << " distance: " << distanceSum << endl;
    return (distanceSum < 1.e-10);
}

void WaypointCEE::initializeState()
{
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

void WaypointCEE::updateState(double stepSize)
{
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
}
double WaypointCEE::getRemainingTime()
{
    double dx = command->getX() - node->x;
    double dy = command->getY() - node->y;
    double dz = command->getZ() - node->z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance / this->node->speed;
}

/**
 * Takeoff Command Execution Engine
 */
TakeoffCEE::TakeoffCEE(UAVNode& node, TakeoffCommand& command)
{
    this->node = &node;
    this->command = &command;
}

bool TakeoffCEE::commandCompleted()
{
    double distanceSum = fabs(command->getZ() - node->z);
    return (distanceSum < 1.e-10);
}

void TakeoffCEE::initializeState()
{
    //node->yaw = node->yaw;
    node->pitch = 0;
}

void TakeoffCEE::updateState(double stepSize)
{
    double stepDistance = node->speed * stepSize;
    if (command->getZ() > node->z)
        node->z += stepDistance;
    else
        node->z -= stepDistance;

    //some animation, remove if irritating
    node->yaw = node->yaw + 5;
}

double TakeoffCEE::getRemainingTime()
{
    return fabs(command->getZ() - node->z) / this->node->speed;
}

/**
 * HoldPosition Command Execution Engine
 */
HoldPositionCEE::HoldPositionCEE(UAVNode& node, HoldPositionCommand& command)
{
    this->node = &node;
    this->command = &command;
    this->holdPositionTill = simTime() + this->command->getHoldSeconds();
}

bool HoldPositionCEE::commandCompleted()
{
    return (simTime() == this->holdPositionTill) ? true : false;
}

void HoldPositionCEE::initializeState()
{
    //node->yaw = node->yaw;
    node->pitch = 0;
}

void HoldPositionCEE::updateState(double stepSize)
{
    //some animation, remove if irritating
    node->yaw = node->yaw + 5;
}

double HoldPositionCEE::getRemainingTime()
{
    return (this->holdPositionTill - simTime()).dbl();
}
