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

Command::Command() {
    // TODO Auto-generated constructor stub
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


WaypointCommand::WaypointCommand(double x, double y, double z) {
    this->setX(x);
    this->setY(y);
    this->setZ(z);
}

void WaypointCommand::setSpeed(double value) {
    this->speed = value;
}


TakeoffCommand::TakeoffCommand(double altitude) {
    this->setX(0);
    this->setY(0);
    this->setZ(0);
    this->setAltitude(altitude);
}

void TakeoffCommand::setAltitude(double value) {
    this->setZ(value);
}


HoldPositionCommand::HoldPositionCommand(int seconds) {
    this->setHoldSeconds(seconds);
}

void HoldPositionCommand::setHoldSeconds(int value) {
    this->holdSeconds = value;
}
