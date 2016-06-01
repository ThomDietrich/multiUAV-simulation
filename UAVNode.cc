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
        readWaypointsFromFile(par("trackFile"));
        // initial location
        targetPointIndex = 0;
        x = waypoints[targetPointIndex].x;
        y = waypoints[targetPointIndex].y;
        z = waypoints[targetPointIndex].z;
        speed = par("speed");
        waypointProximity = par("waypointProximity");
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
       if (!inputFile.fail())
           waypoints.push_back(Waypoint(OsgEarthScene::getInstance()->toX(latitude), OsgEarthScene::getInstance()->toY(longitude), altitude, 0.0));
       else
           break;
    }
}

void UAVNode::move()
{
    //Waypoint target = waypoints[targetPointIndex];
    //absolute distance to next waypoint, in meters
    //double dx = target.x - x;
    //double dy = target.y - y;
    //double dz = target.z - z;
    //double totalDistance = sqrt(dx*dx + dy*dy + dz*dz);
    //EV_INFO << "dx=" << dx << " dy=" << dy << " dz=" << dz << " totalDistance=" << totalDistance << endl;

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

void UAVNode::updateState() {
    //TODO handle waypoint commands as well as hold position commands
    if (commandCompleted()) {
        EV_INFO << "Current command completed! Selecting next command." << endl;
        updateCommand();
    }
}

bool UAVNode::commandCompleted() {
    // TODO: adopt for other command types

    //move command completed if "very close" to waypoint
    Waypoint target = waypoints[targetPointIndex];
    double distanceSum = fabs(target.x - x) + fabs(target.y - y) + fabs(target.z - z);
    return (distanceSum < 1.e-10);
}

void UAVNode::updateCommand() {
    //TODO: adopt for other command types

    //update next waypoint
    targetPointIndex = (targetPointIndex+1) % waypoints.size();
    Waypoint target = waypoints[targetPointIndex];

    //absolute distance to next waypoint, in meters
    double dx = target.x - x;
    double dy = target.y - y;
    double dz = target.z - z;

    //update and store yaw and pitch angles
    yaw = atan2(dx, -dy) / M_PI * 180;
    pitch = atan2(dz, sqrt(dx*dx + dy*dy)) / M_PI * 180;
}

double UAVNode::getNextStepSize() {
    Waypoint target = waypoints[targetPointIndex];
    double dx = target.x - x;
    double dy = target.y - y;
    double dz = target.z - z;
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
