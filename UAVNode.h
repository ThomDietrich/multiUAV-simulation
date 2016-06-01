//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 2015 OpenSim Ltd.
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifndef __UAVNODE_H__
#define __UAVNODE_H__

#include <vector>
#include <string>
#include <omnetpp.h>
#include "MobileNode.h"

using namespace omnetpp;

// a structure to store time coded waypoints
struct Waypoint
{
    Waypoint(double x, double y, double z, double timestamp) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->timestamp = timestamp;
    }
    double x;
    double y;
    double z;
    double timestamp;
};

typedef std::vector<Waypoint> WaypointVector;

/**
 * A mobile node that follows a predefined track.
 */
class UAVNode : public MobileNode
{
  protected:
    // configuration
    WaypointVector waypoints;

    double  waypointProximity;
    double angularSpeed;
    int targetPointIndex;

  private:
    static int normalizeAngle(int angle);

  public:
    UAVNode();
    virtual ~UAVNode();

  protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return 2; }
    void readWaypointsFromFile(const char *fileName);
    virtual void move() override;
    virtual void updateState() override;
    virtual bool commandCompleted();
    virtual void updateCommand();
    virtual double getNextStepSize() override;
};

#endif
