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

#ifndef COMMAND_H_
#define COMMAND_H_

#include <omnetpp.h>
#include <string>
#include <deque>

using namespace omnetpp;

class Command {
protected:
    const char *messageName;
    void setMessageName(const char*);
    double x, y, z;  // in meters, relative to playground origin, destination
public:
    Command();
    virtual ~Command();
    const char* getMessageName()
    {
        return messageName;
    }
    void setX(double);
    void setY(double);
    void setZ(double);
    double getX() const
    {
        return x;
    }
    double getY() const
    {
        return y;
    }
    double getZ() const
    {
        return z;
    }
};

/**
 * The vehicle will fly a straight line to the location specified as a lat, lon and altitude (in meters).
 * http://ardupilot.org/copter/docs/mission-command-list.html#waypoint
 */
class WaypointCommand : public Command {
protected:
    double speed; // in meters per second
public:
    WaypointCommand(double x, double y, double z);
    void setSpeed(double);
    double getSpeed() const
    {
        return speed;
    }
};

/**
 * The vehicle will climb straight up from its current location to the altitude specified (in meters).
 * http://ardupilot.org/copter/docs/mission-command-list.html#takeoff
 */
class TakeoffCommand : public Command {
public:
    TakeoffCommand(double altitude);
    void setAltitude(double);
};

/**
 * Part of Loiter_Time command http://ardupilot.org/copter/docs/mission-command-list.html#loiter-time
 * The vehicle will wait at the current location for the specified number of seconds.
 */
class HoldPositionCommand : public Command {
protected:
    int holdSeconds;
public:
    HoldPositionCommand(double x, double y, double z, int seconds);
    //HoldPositionCommand(Date till);
    void setHoldSeconds(int value);
    int getHoldSeconds() const
    {
        return holdSeconds;
    }
};

class ChargingNode;

/**
 * The vehicle will connect to a ChargingNode and initiate a charging procedure.
 * Charging ends when battery is fully charged.
 */
class ChargeCommand : public Command {
protected:
    ChargingNode *node;
public:
    ChargeCommand(ChargingNode *node);
    ChargingNode *getChargingNode();
};

class GenericNode;

/**
 * The vehicle will wait at the current location until further notice.
 * The command is reserved for Node to Node data transfer. Both Nodes have to be in the Exchange mode.
 * Exchange ends when Data was successfully transfered.
 */
class ExchangeCommand : public Command {
protected:
    GenericNode *otherNode;
    bool thisNodeHasDataToExchange;
    bool scheduleRechargeAfter;
public:
    ExchangeCommand(GenericNode *otherNode, bool scheduleRechargeAfter, bool transmitData);
    void setOtherNode(GenericNode* otherNode);
    GenericNode *getOtherNode();
    bool isOtherNodeKnown();
    bool isRechargeRequested();
};

/**
 * The vehicle will wait at the current location until further notice.
 */
class WaitCommand : public Command {
public:
    WaitCommand();
};

#endif /* COMMAND_H_ */
