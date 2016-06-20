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

#include <string>

class Command {
protected:
    std::string messageName;
    void setMessageName(std::string);
    double x, y, z;  // in meters, relative to playground origin
public:
    Command();
    virtual ~Command();
    std::string getMessageName() const { return messageName; }
    void setX(double);
    void setY(double);
    void setZ(double);
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }
};


/**
 * The vehicle will fly a straight line to the location specified as a lat, lon and altitude (in meters).
 * http://ardupilot.org/copter/docs/mission-command-list.html#waypoint
 */
class WaypointCommand :public Command {
protected:
    double speed; // in meters per second
public:
    WaypointCommand(double x, double y, double z);
    void setSpeed(double);
    double getSpeed() const { return speed; }
};


/**
 * The vehicle will climb straight up from it’s current location to the altitude specified (in meters).
 * http://ardupilot.org/copter/docs/mission-command-list.html#takeoff
 */
class TakeoffCommand :public Command {
public:
    TakeoffCommand(double altitude);
    void setAltitude(double);
};


/**
 * Part of Loiter_Time command http://ardupilot.org/copter/docs/mission-command-list.html#loiter-time
 * The vehicle will wait at the current location for the specified number of seconds.
 */
class HoldPositionCommand :public Command {
protected:
    int holdSeconds;
public:
    HoldPositionCommand(int seconds);
    //HoldPositionCommand(Date till);
    void setHoldSeconds(int value);
    int getHoldSeconds() const { return holdSeconds; }
};

#endif /* COMMAND_H_ */
