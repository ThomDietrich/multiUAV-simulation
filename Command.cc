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

#include <Command.h>

/**
 *
 */
Command::Command() {
    x = 0;
    y = 0;
    z = 0;
    messageName = "";
}
Command::~Command() {
    // TODO Auto-generated destructor stub
}

void Command::setX(double value) {
    this->x = value;
}
void Command::setY(double value) {
    this->y = value;
}
void Command::setZ(double value) {
    this->z = value;
}
void Command::setMessageName(std::string name) {
    this->messageName = name;
}

/**
 *
 */
WaypointCommand::WaypointCommand(double x, double y, double z) {
    Command::Command();
    setX(x);
    setY(y);
    setZ(z);
    setMessageName("waypoint");
}

void WaypointCommand::setSpeed(double value) {
    this->speed = value;
}

/**
 *
 */
TakeoffCommand::TakeoffCommand(double altitude) {
    Command::Command();
    setAltitude(altitude);
    setMessageName("takeoff");
}

void TakeoffCommand::setAltitude(double value) {
    setZ(value);
}

/**
 *
 */
HoldPositionCommand::HoldPositionCommand(int seconds) {
    Command::Command();
    setHoldSeconds(seconds);
    setMessageName("holdPosition");
}

void HoldPositionCommand::setHoldSeconds(int value) {
    this->holdSeconds = value;
}
