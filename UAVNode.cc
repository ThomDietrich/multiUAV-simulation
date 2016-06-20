//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 2015 OpenSim Ltd.
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifdef WITH_OSG
#include "UAVNode.h"
#include "OsgEarthScene.h"
#include "ChannelController.h"
#include <fstream>
#include <iostream>

using namespace omnetpp;

Define_Module(UAVNode);

UAVNode::UAVNode()
{
}

UAVNode::~UAVNode()
{
}

void UAVNode::initialize(int stage)
{
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
        angularSpeed = 0;
        break;
    }
}

void UAVNode::readWaypointsFromFile(const char *fileName)
{
    std::ifstream inputFile(fileName);
    while (true) {
       double longitude, latitude, altitude;
       inputFile >> longitude >> latitude >> altitude;
       if (!inputFile.fail()) {
           commands.push_back(WaypointCommand(OsgEarthScene::getInstance()->toX(latitude), OsgEarthScene::getInstance()->toY(longitude), altitude));
       } else {
           break;
       }
    }
}

void UAVNode::move()
{
    //distance to move, based on simulation time passed since last update
    double stepSize = (simTime() - lastUpdate).dbl();
    double stepDistance = speed * stepSize;

    //resulting movement broken down to x,y,z
    double stepZ = stepDistance * sin(M_PI * pitch / 180);
    double stepXY = stepDistance * cos(M_PI * pitch / 180);
    double stepX = stepXY * sin(M_PI * yaw / 180);
    double stepY = stepXY * -cos(M_PI * yaw / 180);
    x += stepX;
    y += stepY;
    z += stepZ;
}

bool UAVNode::commandCompleted() {
    Command cmd = commands.front();
    if (cmd.getMessageName() == "waypoint") {
        double distanceSum = fabs(cmd.getX() - x) + fabs(cmd.getY() - y) + fabs(cmd.getZ() - z);
        return (distanceSum < 1.e-10);
    } else if (cmd.getMessageName() == "takeoff") {
        double distanceSum = fabs(cmd.getZ() - z);
        return (distanceSum < 1.e-10);
    } else if (cmd.getMessageName() == "holdPosition") {
        return 0;
    } else {
        EV_INFO << "command: " << cmd.getMessageName() << endl;
        throw cRuntimeError("commandCompleted(): Unknown command type");
    }
}

void UAVNode::updateCommand() {
    if (commandCompleted()) {
        EV_INFO << "UAV #" << this->getIndex() << " completed its current command! Selecting next command." << endl;
        commands.pop_front();
        if (commands.size() == 0) {
            throw cRuntimeError("updateCommand(): UAV has no commands left. TODO: holdPosition?");
        }
    }
}

void UAVNode::updateState() {
    Command cmd = commands.front();

    if (cmd.getMessageName() == "waypoint") {
        //absolute distance to next waypoint, in meters
        double dx = cmd.getX() - x;
        double dy = cmd.getY() - y;
        double dz = cmd.getZ() - z;

        //update and store yaw and pitch angles
        yaw = atan2(dx, -dy) / M_PI * 180;
        pitch = atan2(dz, sqrt(dx*dx + dy*dy)) / M_PI * 180;

    } else if (cmd.getMessageName() == "takeoff") {
        yaw = this->yaw;
        pitch = 0;

    } else if (cmd.getMessageName() == "holdPosition") {
        yaw = this->yaw;
        pitch = 0;

    } else {
        throw cRuntimeError("updateCommand(): Unknown command type");
    }
}

double UAVNode::getNextStepSize() {
    Command cmd = commands.front();
    double dx = cmd.getX() - x;
    double dy = cmd.getY() - y;
    double dz = cmd.getZ() - z;
    double remainingTime = sqrt(dx*dx + dy*dy + dz*dz) / speed;
    return (timeStep == 0 || remainingTime < timeStep) ? remainingTime : timeStep;
}

//obsolete
int UAVNode::normalizeAngle(int angle)
{
    int newAngle = angle;
    while (newAngle <= -180) newAngle += 360;
    while (newAngle > 180) newAngle -= 360;
    return newAngle;
}

#endif // WITH_OSG
