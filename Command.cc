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

#include "Command.h"

/**
 *
 */
Command::Command()
{
    x = 0;
    y = 0;
    z = 0;
    messageName = "";
}
Command::~Command()
{
    // TODO Auto-generated destructor stub
}

void Command::setX(double value)
{
    this->x = value;
}
void Command::setY(double value)
{
    this->y = value;
}
void Command::setZ(double value)
{
    this->z = value;
}
void Command::setMessageName(const char *name)
{
    this->messageName = name;
}

/**
 *
 * @param x
 * @param y
 * @param z
 */
WaypointCommand::WaypointCommand(double x, double y, double z)
{
    setX(x);
    setY(y);
    setZ(z);
    setMessageName("waypoint");
}

void WaypointCommand::setSpeed(double value)
{
    this->speed = value;
}

/**
 *
 * @param altitude
 */
TakeoffCommand::TakeoffCommand(double altitude)
{
    setX(0);
    setY(0);
    setZ(0);
    setAltitude(altitude);
    setMessageName("takeoff");
}

void TakeoffCommand::setAltitude(double value)
{
    setZ(value);
}

/**
 *
 * @param seconds
 */
HoldPositionCommand::HoldPositionCommand(double x, double y, double z, int seconds)
{
    setX(x);
    setY(y);
    setZ(z);
    setHoldSeconds(seconds);
    setMessageName("holdPosition");
}

void HoldPositionCommand::setHoldSeconds(int value)
{
    this->holdSeconds = value;
}

/**
 *
 * @param node
 */
ChargeCommand::ChargeCommand(ChargingNode* node)
{
    this->node = node;
    this->setMessageName("charge");
}

ChargingNode* ChargeCommand::getChargingNode()
{
    return this->node;
}

/**
 *
 */
ExchangeCommand::ExchangeCommand(GenericNode* otherNode, bool scheduleRechargeAfter, bool transmitData)
{
    this->otherNode = otherNode;
    this->scheduleRechargeAfter = scheduleRechargeAfter;
    this->thisNodeHasDataToExchange = transmitData;
    this->setMessageName("exchange");
}

void ExchangeCommand::setOtherNode(GenericNode* otherNode)
{
    this->otherNode = otherNode;
}

GenericNode* ExchangeCommand::getOtherNode()
{
    return this->otherNode;
}

bool ExchangeCommand::isOtherNodeKnown()
{
    return (otherNode == nullptr) ? false : true;
}

bool ExchangeCommand::isRechargeRequested()
{
    return scheduleRechargeAfter;
}

/**
 *
 * @param seconds
 */
IdleCommand::IdleCommand()
{
    this->setMessageName("idle");
}
