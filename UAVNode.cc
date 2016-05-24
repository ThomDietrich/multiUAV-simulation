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
        yaw = 0;
        pitch = 0;
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
    Waypoint target = waypoints[targetPointIndex];

    //absolute distance to next waypoint, in meters
    double dx = target.x - x;
    double dy = target.y - y;
    double dz = target.z - z;

    //update and store yaw and pitch angles
    double angleXY = atan2(dx, -dy) / M_PI * 180;
    double angleZ = atan2(dz, sqrt(dx*dx + dy*dy)) / M_PI * 180;
    if (0) {
        //travel curves in circular motion
        angularSpeed = normalizeAngle(angleXY - yaw) * 3;
        yaw += angularSpeed * timeStep;
        angularSpeed = normalizeAngle(angleZ - pitch) * 3;
        pitch += angularSpeed * timeStep;
    } else {
        //travel waypoint to waypoint
        yaw = angleXY;
        pitch = angleZ;
    }

    //calculate resulting movement, corresponding to speed and simulation time
    double stepDistance = speed * timeStep;
    double stepZ = stepDistance * sin(M_PI * pitch / 180);
    double stepXY = stepDistance * cos(M_PI * pitch / 180);
    double stepX = stepXY * sin(M_PI * yaw / 180);
    double stepY = stepXY * -cos(M_PI * yaw / 180);
    x += stepX;
    y += stepY;
    z += stepZ;

    // waypoint reached within defined proximity
    if ((dx*dx + dy*dy + dz*dz) < waypointProximity*waypointProximity) {
        targetPointIndex = (targetPointIndex+1) % waypoints.size();
        EV_INFO << "waypoint reached." << endl;
    }
}

double UAVNode::getArrivalTime() {
    Waypoint target = waypoints[targetPointIndex];
    double dx = target.x - x;
    double dy = target.y - y;
    double dz = target.z - z;
    double remainingDistance = sqrt(dx*dx + dy*dy + dz*dz);
    double remainingTime = remainingDistance / speed;
    return remainingTime;
}

int UAVNode::normalizeAngle(int angle)
{
    int newAngle = angle;
    while (newAngle <= -180) newAngle += 360;
    while (newAngle > 180) newAngle -= 360;
    return newAngle;
}

#endif // WITH_OSG
