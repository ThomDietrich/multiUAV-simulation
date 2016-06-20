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
#include "Command.h"

using namespace omnetpp;

typedef std::deque<Command> CommandQueue;

/**
 * A mobile node that follows a predefined track.
 */
class UAVNode : public MobileNode
{
  protected:
    CommandQueue commands;

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
    virtual bool commandCompleted();
    virtual void updateCommand() override;
    virtual void updateState() override;
    virtual double getNextStepSize() override;
};

#endif
