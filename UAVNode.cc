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

#ifdef WITH_OSG
#include "UAVNode.h"
#include "OsgEarthScene.h"
#include "ChannelController.h"
#include <fstream>
#include <iostream>
#include <sstream>

using namespace omnetpp;

Define_Module(UAVNode);

UAVNode::UAVNode() {
}

UAVNode::~UAVNode() {
}

void UAVNode::initialize(int stage) {
    MobileNode::initialize(stage);
    switch (stage) {
        case 0:
            // fill the track
            for (int _ = 0; _ < 100; ++_) {
                //Dirty little hack for enough waypoints
                readWaypointsFromFile(par("trackFile"));
            }
            // initial position
            x = par("startX");
            y = par("startY");
            z = 2;
            speed = par("speed");
            break;
    }
}

void UAVNode::readWaypointsFromFile(const char *fileName) {
    std::ifstream inputFile(fileName);
    while (true) {
        std::string commandName;
        double param1, param2, param3;
        inputFile >> commandName >> param1 >> param2 >> param3;
        if (inputFile.fail()) {
            break;
        }
        if (commandName == "WAYPOINT") {
            commands.push_back(new WaypointCommand(OsgEarthScene::getInstance()->toX(param2), OsgEarthScene::getInstance()->toY(param1), param3));
        }
        else if (commandName == "TAKEOFF") {
            commands.push_back(new TakeoffCommand(param3));
        }
        else if (commandName == "HOLDPOSITION") {
            commands.push_back(new HoldPositionCommand(param3));
        }
        else {
            throw cRuntimeError("readWaypointsFromFile(): Unexpected file content.");
        }
    }
}

void UAVNode::loadNextCommand() {
    delete commandExecEngine;

    if (commands.size() == 0) {
        throw cRuntimeError("loadNextCommand(): UAV has no commands left.");
    }

    if (WaypointCommand *command = dynamic_cast<WaypointCommand *>(commands.front())) {
        commandExecEngine = new WaypointCEE(*this, *command);
    }
    else if (TakeoffCommand *command = dynamic_cast<TakeoffCommand *>(commands.front())) {
        commandExecEngine = new TakeoffCEE(*this, *command);
    }
    else if (HoldPositionCommand *command = dynamic_cast<HoldPositionCommand *>(commands.front())) {
        commandExecEngine = new HoldPositionCEE(*this, *command);
    }
    else
        throw cRuntimeError("loadNextCommand(): invalid cast.");
    commands.pop_front();
}

void UAVNode::initializeState() {
    if (commandExecEngine == nullptr) {
        throw cRuntimeError("initializeState(): Command Engine missing.");
    }
    commandExecEngine->initializeState();

    std::string text(getFullName());
    switch (commandExecEngine->getCeeType()) {
        case WAYPOINT:
            text += " WP";
            break;
        case TAKEOFF:
            text += " TO";
            break;
        case HOLDPOSITION:
            text += " HP";
            break;
    }
    labelNode->setText(text);
}

void UAVNode::updateState() {
    //distance to move, based on simulation time passed since last update
    double stepSize = (simTime() - lastUpdate).dbl();
    updateState(stepSize);

    //update sublabel with battery info
    std::ostringstream strs;
    strs << battery.getRemainingPercentage() << " %";
    std::string str = strs.str();
    sublabelNode->setText(str);
}

void UAVNode::updateState(double stepSize) {
    commandExecEngine->updateState(stepSize);
}

bool UAVNode::commandCompleted() {
    return commandExecEngine->commandCompleted();
}

/**
 * Get the time in seconds till the end of current command
 */
double UAVNode::nextNeededUpdate() {
    return commandExecEngine->getRemainingTime();
}

//obsolete
int UAVNode::normalizeAngle(int angle) {
    int newAngle = angle;
    while (newAngle <= -180)
        newAngle += 360;
    while (newAngle > 180)
        newAngle -= 360;
    return newAngle;
}

#endif // WITH_OSG
